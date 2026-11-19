#!/usr/bin/env python3
"""Read-only, model-free parcel candidates on a manually marked tabletop.

OpenCV uses BGR throughout. No ROS, SSH, simulator state, or robot commands.
The configuration belongs to ONE fixed camera/table arrangement. Re-mark it
after either moves. An accepted contour is a visual candidate, not a grasp.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np


def polygon_mask(shape, points):
    mask = np.zeros(shape[:2], np.uint8)
    cv2.fillPoly(mask, [np.rint(points).astype(np.int32)], 255)
    return mask


def checked_polygon(points, name):
    points = np.asarray(points, np.float32)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3:
        raise ValueError(f'{name}: need at least three pixel points')
    if not np.isfinite(points).all() or abs(cv2.contourArea(points)) < 10:
        raise ValueError(f'{name}: degenerate polygon')
    return points


def homography(config):
    """Optional surveyed image/table point pairs; does NOT use arm joint arrays."""
    cal = config.get('table_calibration')
    if not cal:
        return None
    src = np.asarray(cal['image_points_px'], np.float64)
    dst = np.asarray(cal['table_points_m'], np.float64)
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 2 or len(src) < 4:
        raise ValueError('Calibration requires >=4 matching non-collinear XY pairs')
    if not np.isfinite(src).all() or not np.isfinite(dst).all():
        raise ValueError('Nonfinite calibration points')
    h, _ = cv2.findHomography(src, dst, 0)
    if h is None or abs(np.linalg.det(h)) < 1e-14:
        raise ValueError('Degenerate table calibration')
    back = cv2.perspectiveTransform(dst[None], np.linalg.inv(h))[0]
    if np.max(np.linalg.norm(src - back, axis=1)) > 3:
        raise ValueError('Table calibration residual exceeds 3px')
    return h


def detect(image, config, background_image=None):
    height, width = image.shape[:2]
    if config.get('image_size_wh') != [width, height]:
        raise ValueError('Image size differs from ROI calibration; re-mark for this image')
    workspace = checked_polygon(config['workspace_px'], 'workspace_px')
    allowed = polygon_mask(image.shape, workspace)
    exclusions = [checked_polygon(p, 'exclude_px') for p in config.get('exclude_px', [])]
    for points in exclusions:
        allowed[polygon_mask(image.shape, points) > 0] = 0
    if np.count_nonzero(allowed) < 500:
        raise ValueError('Too little visible workspace')
    sample = allowed.copy()
    if config.get('background_sample_px'):
        sample &= polygon_mask(image.shape, checked_polygon(config['background_sample_px'], 'background_sample_px'))
    if np.count_nonzero(sample) < 100:
        raise ValueError('Need >=100 visible tablecloth/background sample pixels')

    smooth = cv2.GaussianBlur(image, (5, 5), 0)
    lab = cv2.cvtColor(smooth, cv2.COLOR_BGR2LAB).astype(np.float32)
    hsv = cv2.cvtColor(smooth, cv2.COLOR_BGR2HSV)
    # Median chroma tolerates modest shadows on a mostly uniform warm table.
    # Cool white bags differ from warm wood/cloth in a/b, dark bags in L.
    background = np.median(lab[sample > 0], axis=0)
    chroma = np.linalg.norm(lab[:, :, 1:] - background[1:], axis=2)
    params = config.get('thresholds', {})
    hh, ss = hsv[:, :, 0], hsv[:, :, 1]
    lightness = lab[:, :, 0]
    # Keep colour families separate: a pale, brightly lit patch of yellow cloth
    # must not join a purple bag through an indiscriminate colour-distance mask.
    families = {
        'purple': (hh >= 115) & (hh <= 165) & (ss > 30) & (lightness > 90),
        'green': (hh >= 35) & (hh <= 95) & (ss > 70) & (lightness > 65),
        'dark': (lightness < params.get('dark_lightness', 105)) &
                (background[0] - lightness > params.get('dark_delta', 55)),
        'light_or_other': (chroma > params.get('chroma_distance', 18)) &
                          (lightness > 110) & (ss < 85) &
                          (lab[:, :, 2] < params.get('light_max_lab_b', 140)),
    }
    if background_image is not None:
        if background_image.shape != image.shape:
            raise ValueError('Empty-background image size differs from input')
        reference = cv2.GaussianBlur(background_image, (5,5), 0)
        changed = np.max(cv2.absdiff(smooth, reference), axis=2) > params.get('background_delta', 30)
        families = {label: mask & changed for label,mask in families.items()}
    kernel = np.ones((3, 3), np.uint8)
    raw = np.zeros_like(allowed)
    contours = []
    occupied = np.zeros_like(allowed)
    for label, foreground in families.items():
        mask = foreground.astype(np.uint8) * 255
        if label == 'light_or_other':
            mask[cv2.dilate(occupied, np.ones((5,5),np.uint8)) > 0] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7,7),np.uint8))
        mask &= allowed
        found, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours.extend((label, contour) for contour in found)
        if label != 'light_or_other':
            cv2.drawContours(occupied, found, -1, 255, -1)
        raw |= mask
    boundary = (allowed > 0) & (cv2.erode(allowed, np.ones((5, 5), np.uint8)) == 0)
    scale = width * height / (848 * 480)
    h = homography(config)
    results = []
    for label, contour in contours:
        area = cv2.contourArea(contour)
        if area < params.get('min_area_px_at_848x480', 650) * scale:
            continue
        obj_mask = np.zeros((height, width), np.uint8)
        cv2.drawContours(obj_mask, [contour], -1, 255, -1)
        moments = cv2.moments(contour)
        center = [moments['m10'] / moments['m00'], moments['m01'] / moments['m00']]
        dims = cv2.minAreaRect(contour)[1]
        aspect = max(dims) / max(min(dims), 1e-6)
        solidity = area / max(cv2.contourArea(cv2.convexHull(contour)), 1)
        contact = int(np.count_nonzero((obj_mask > 0) & boundary))
        reasons = []
        if contact > 2:
            reasons.append('workspace_or_exclusion_contact')
        if np.any(contour[:, 0, 0] <= 1) or np.any(contour[:, 0, 0] >= width - 2) or np.any(contour[:, 0, 1] <= 1) or np.any(contour[:, 0, 1] >= height - 2):
            reasons.append('image_edge_contact')
        if area > params.get('max_area_px_at_848x480', 25000) * scale:
            reasons.append('oversized_or_merged')
        if aspect > params.get('max_aspect', 3.0) or solidity < params.get('min_solidity', .72):
            reasons.append('irregular_or_occluded')
        # An interior point gives margin to visible edges. This does not establish
        # finger clearance, bag height, friction, or grasp success.
        distance = cv2.distanceTransform(obj_mask, cv2.DIST_L2, 5)
        _, radius, _, inside = cv2.minMaxLoc(distance)
        if radius < params.get('min_interior_radius_px', 9) * np.sqrt(scale):
            reasons.append('thin_or_fragmented')
        x, y, w, ht = cv2.boundingRect(contour)
        item = dict(label=label, status='candidate' if not reasons else 'review_required',
                    reasons=reasons, center_px=center, interior_point_px=list(inside),
                    interior_radius_px=float(radius), area_px=float(area),
                    bbox_xywh=[x, y, w, ht], solidity=float(solidity), aspect=float(aspect),
                    contour_px=contour[:, 0, :].tolist())
        if h is not None:
            # Project the WHOLE contour before computing its planar centroid:
            # projective transforms do not preserve image centroids.
            planar = cv2.perspectiveTransform(contour.astype(np.float32), h)
            pm = cv2.moments(planar)
            if abs(pm['m00']) < 1e-12:
                raise ValueError('Projected contour is degenerate')
            xy = [pm['m10'] / pm['m00'], pm['m01'] / pm['m00']]
            item['table_plane_center_m'] = xy
            item['table_plane_center_backprojected_px'] = cv2.perspectiveTransform(np.array([[xy]], np.float32), np.linalg.inv(h))[0, 0].tolist()
            item['table_plane_interior_point_m'] = cv2.perspectiveTransform(np.array([[inside]], np.float32), h)[0, 0].tolist()
            item['plane_projection_warning'] = 'Bag height causes parallax; not a measured surface XYZ or robot target'
        results.append(item)
    results.sort(key=lambda r: (r['center_px'][0], r['center_px'][1]))
    return dict(schema_version=1, motion_authorized=False, image_size_wh=[width, height],
                background_lab=background.tolist(), candidates=results,
                accepted_count=sum(r['status'] == 'candidate' for r in results),
                review_count=sum(r['status'] != 'candidate' for r in results),
                calibration_present=h is not None,
                empty_background_used=background_image is not None), raw


def annotate(image, result, config):
    out = image.copy()
    cv2.polylines(out, [np.asarray(config['workspace_px'], np.int32)], True, (255, 210, 0), 2)
    for p in config.get('exclude_px', []):
        cv2.polylines(out, [np.asarray(p, np.int32)], True, (100, 100, 255), 1)
    for index, obj in enumerate(result['candidates'], 1):
        color = (0, 240, 0) if obj['status'] == 'candidate' else (220, 0, 220)
        cv2.polylines(out, [np.asarray(obj['contour_px'], np.int32)], True, color, 2)
        c = tuple(np.rint(obj['center_px']).astype(int))
        cv2.drawMarker(out, c, color, cv2.MARKER_CROSS, 18, 2)
        cv2.circle(out, tuple(obj['interior_point_px']), 4, (0, 180, 255), -1)
        x, y, _, _ = obj['bbox_xywh']
        text = f'{index} {obj["label"]} ({c[0]},{c[1]})'
        if obj['status'] != 'candidate':
            text += ' REVIEW'
        cv2.putText(out, text, (max(0, x - 5), max(38, y - 7)), cv2.FONT_HERSHEY_SIMPLEX, .42, color, 1, cv2.LINE_AA)
    cv2.rectangle(out, (0, 0), (out.shape[1], 26), (0, 0, 0), -1)
    cv2.putText(out, f'RGB ONLY | candidates={result["accepted_count"]} review={result["review_count"]} | NO MOTION',
                (8, 18), cv2.FONT_HERSHEY_SIMPLEX, .46, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def pick_polygon(image, title):
    points = []
    def mouse(event, x, y, flags, data):
        if event == cv2.EVENT_LBUTTONDOWN:
            points.append([x, y])
        elif event == cv2.EVENT_RBUTTONDOWN and points:
            points.pop()
    cv2.namedWindow(title)
    cv2.setMouseCallback(title, mouse)
    try:
        while True:
            view = image.copy()
            if points:
                cv2.polylines(view, [np.array(points, np.int32)], len(points) >= 3, (0, 255, 0), 2)
            cv2.putText(view, 'LMB:point RMB:undo Enter:save Esc:cancel', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255), 1)
            cv2.imshow(title, view)
            key = cv2.waitKey(30) & 255
            if key == 27 or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                raise KeyboardInterrupt('ROI marking cancelled')
            if key in (10, 13) and len(points) >= 3:
                return checked_polygon(points, title).astype(int).tolist()
    finally:
        cv2.destroyWindow(title)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    mark = sub.add_parser('mark', help='Manually mark a fixed-view workspace and exclusions')
    mark.add_argument('--image', required=True, type=Path)
    mark.add_argument('--config', required=True, type=Path)
    mark.add_argument('--exclusions', type=int, default=0, help='Number of robot/scale/bin polygons to exclude')
    run = sub.add_parser('run', help='Process an image or video, or watch a repeatedly updated JPEG')
    run.add_argument('--input', required=True, type=Path)
    run.add_argument('--config', required=True, type=Path)
    run.add_argument('--output', required=True, type=Path)
    run.add_argument('--show', action='store_true')
    run.add_argument('--watch', action='store_true', help='Poll a local JPEG/PNG; no camera/robot connection')
    run.add_argument('--max-frames', type=int)
    args = parser.parse_args()
    if args.command == 'mark' or getattr(args, 'show', False):
        if any('GUI:' in line and 'NONE' in line for line in cv2.getBuildInformation().splitlines()):
            parser.error('This Python has headless OpenCV. Use /usr/bin/python3 for mark/--show, or omit --show for file output.')
    if args.command == 'mark':
        if args.config.exists():
            raise FileExistsError('Use a new config path to preserve existing calibration')
        image = cv2.imread(str(args.image))
        if image is None:
            raise ValueError(f'Cannot read {args.image}')
        config = dict(image_size_wh=[image.shape[1], image.shape[0]],
                      workspace_px=pick_polygon(image, '1: tabletop working region'),
                      exclude_px=[pick_polygon(image, f'2: exclude {i+1}') for i in range(args.exclusions)])
        config['background_sample_px'] = pick_polygon(image, '3: bare table sample (no objects)')
        args.config.parent.mkdir(parents=True, exist_ok=True)
        args.config.write_text(json.dumps(config, indent=2) + '\n')
        print(f'Saved {args.config}. Re-mark after camera/table/robot exclusion positions change.')
        return
    if args.max_frames is not None and args.max_frames < 1:
        parser.error('--max-frames must be positive')
    config = json.loads(args.config.read_text())
    background = None
    if config.get('background_image'):
        bg_path = Path(config['background_image'])
        if not bg_path.is_absolute():
            bg_path = args.config.parent / bg_path
        background = cv2.imread(str(bg_path))
        if background is None:
            raise ValueError(f'Cannot read background image {bg_path}')
    args.output.mkdir(parents=True, exist_ok=True)
    is_image = args.input.suffix.lower() in {'.png', '.jpg', '.jpeg', '.bmp'}
    if args.watch and not is_image:
        parser.error('--watch requires a JPEG/PNG path')
    cap = None if is_image else cv2.VideoCapture(str(args.input))
    if cap is not None and not cap.isOpened():
        raise ValueError(f'Cannot open video {args.input}')
    writer = None
    count = 0
    try:
        with (args.output / 'detections.jsonl').open('w') as report:
            last_stamp = None
            while True:
                if args.watch:
                    stamp = args.input.stat().st_mtime_ns if args.input.exists() else None
                    if stamp is None or stamp == last_stamp:
                        if args.show and cv2.waitKey(50) & 255 in (27, ord('q')):
                            break
                        time.sleep(.1)
                        continue
                    last_stamp = stamp
                image = cv2.imread(str(args.input)) if is_image else cap.read()[1]
                if image is None:
                    if args.watch:
                        last_stamp = None
                        time.sleep(.1)
                        continue
                    if is_image:
                        raise ValueError(f'Cannot read image {args.input}')
                    break
                start = time.perf_counter()
                result, mask = detect(image, config, background)
                result.update(frame_index=count, processing_ms=(time.perf_counter()-start)*1000)
                if not is_image:
                    result['video_time_s'] = count / max(cap.get(cv2.CAP_PROP_FPS), 1)
                view = annotate(image, result, config)
                report.write(json.dumps(result) + '\n')
                report.flush()
                if is_image:
                    cv2.imwrite(str(args.output / 'annotated.jpg'), view)
                    cv2.imwrite(str(args.output / 'mask.png'), mask)
                    (args.output / 'latest.json').write_text(json.dumps(result, indent=2) + '\n')
                else:
                    if writer is None:
                        writer = cv2.VideoWriter(str(args.output / 'replay.mp4'), cv2.VideoWriter_fourcc(*'mp4v'),
                                                 max(cap.get(cv2.CAP_PROP_FPS), 1), (image.shape[1], image.shape[0]))
                        if not writer.isOpened():
                            raise ValueError('Cannot open video writer')
                    writer.write(view)
                count += 1
                if args.show:
                    cv2.imshow('Parcel centers: preview only', view)
                    key = cv2.waitKey(0 if is_image and not args.watch else 1) & 255
                    if key in (27, ord('q')):
                        break
                if (is_image and not args.watch) or (args.max_frames and count >= args.max_frames):
                    break
    finally:
        if cap is not None:
            cap.release()
        if writer is not None:
            writer.release()
        if args.show:
            cv2.destroyAllWindows()
    print(json.dumps(dict(frames=count, output=str(args.output), motion_authorized=False)))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Small real-image feasibility check. Hand-drawn outlines are approximate.

ROI/exclusion masks are manually supplied per view; this does NOT benchmark
automatic hand segmentation, unseen scenes, or metric robot positioning.
"""
import argparse
import html
import json
import time
from pathlib import Path

import cv2
import numpy as np

from kuavo_parcel_centers import detect, annotate


CASES = {
    '01_center_down': dict(
        workspace_px=[[272,241],[665,269],[805,479],[141,431]],
        exclude_px=[[[437,304],[517,309],[520,397],[432,389]],
                    [[210,282],[329,278],[336,324],[394,327],[393,376],[336,395],[339,479],[147,479],[213,401]],
                    [[597,374],[666,371],[707,479],[528,479],[535,448],[600,440]]],
        targets=[('purple',[[318,259],[387,250],[390,304],[312,305]]),
                 ('dark',[[580,280],[648,277],[695,348],[613,350]])]),
    '02_yaw_positive': dict(
        workspace_px=[[39,307],[489,228],[615,379],[335,479],[0,479],[0,379]],
        exclude_px=[[[290,306],[361,305],[385,389],[304,403]],
                    [[64,339],[139,336],[150,373],[200,365],[237,366],[233,415],[183,445],[199,479],[29,479]],
                    [[445,335],[482,329],[541,394],[590,479],[393,479],[393,422],[451,410]]],
        targets=[('purple',[[132,306],[213,280],[231,337],[135,361]]),
                 ('dark',[[421,257],[480,241],[521,292],[460,311]])]),
    '03_yaw_negative': dict(
        workspace_px=[[401,227],[847,339],[847,479],[491,479],[282,367]],
        exclude_px=[[[567,328],[646,349],[643,445],[549,417]],
                    [[428,275],[465,278],[465,317],[514,334],[513,371],[458,389],[435,431],[349,479],[205,479],[335,372],[372,326]]],
        targets=[('purple',[[448,252],[519,258],[513,310],[436,298]])],
        clipped_target=[[755,342],[847,365],[847,458],[760,426]]),
    '04_pitch_shallow': dict(
        workspace_px=[[272,269],[669,297],[785,479],[141,459]],
        exclude_px=[[[442,336],[516,339],[524,429],[438,421]],
                    [[210,315],[329,311],[339,355],[395,359],[395,402],[338,422],[338,479],[166,479],[212,433]],
                    [[604,403],[665,400],[701,479],[569,479]]],
        targets=[('purple',[[317,283],[388,279],[393,333],[318,338]]),
                 ('dark',[[593,310],[665,319],[700,398],[613,387]])]),
    '05_pitch_steep': dict(
        workspace_px=[[269,175],[669,203],[794,414],[157,351]],
        exclude_px=[[[445,238],[519,243],[523,327],[440,320]],
                    [[211,216],[330,211],[340,258],[398,259],[397,300],[340,321],[339,402],[187,390],[210,334]],
                    [[604,307],[664,302],[701,415],[529,403],[531,371],[607,370]]],
        targets=[('purple',[[324,193],[393,183],[395,236],[323,239]]),
                 ('dark',[[600,214],[671,219],[695,298],[610,288]])]),
}


def center(poly):
    m = cv2.moments(np.array(poly, np.float32))
    return np.array([m['m10']/m['m00'], m['m01']/m['m00']])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, default=Path('/mnt/e/Data/Picture/multiview_20260921_200838'))
    p.add_argument('--output', type=Path, default=Path('outputs/analysis/parcel_centers_20261016'))
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rows, tiles, errors = [], [], []
    for name, case in CASES.items():
        folder = args.output/name
        folder.mkdir(exist_ok=True)
        image = cv2.imread(str(args.source/name/'rgb.jpg'))
        if image is None:
            raise ValueError(f'Missing {name}')
        config = {k:v for k,v in case.items() if k in ('workspace_px','exclude_px')}
        config.update(image_size_wh=[848,480], source_image=str(args.source/name/'rgb.jpg'),
                      note='Hand-marked per-view table and static arm/scale exclusions; not automatic robot masking')
        (folder/'roi.json').write_text(json.dumps(config,indent=2)+'\n')
        start = time.perf_counter()
        result, mask = detect(image, config)
        result['processing_ms'] = (time.perf_counter()-start)*1000
        view = annotate(image, result, config)
        matches = []
        used = set()
        for label, polygon in case['targets']:
            target = center(polygon)
            cv2.polylines(view,[np.array(polygon,np.int32)],True,(255,120,0),1)
            cv2.drawMarker(view,tuple(np.rint(target).astype(int)),(255,120,0),cv2.MARKER_TILTED_CROSS,12,1)
            possible = [(float(np.linalg.norm(np.array(obj['center_px'])-target)),i,obj)
                        for i,obj in enumerate(result['candidates']) if i not in used and obj['label']==label]
            best = min(possible, default=None, key=lambda t:t[0])
            match = dict(label=label, approximate_reference_center_px=target.tolist())
            if best is not None and best[0] < 45:
                error, i, obj = best
                used.add(i)
                errors.append(error)
                match.update(found=True, candidate_index=i, center_error_px=error, status=obj['status'])
            else:
                match.update(found=False)
            matches.append(match)
        if case.get('clipped_target'):
            cv2.polylines(view,[np.array(case['clipped_target'],np.int32)],True,(255,120,0),1)
        cv2.imwrite(str(folder/'annotated.jpg'),view)
        cv2.imwrite(str(folder/'mask.png'),mask)
        (folder/'detections.json').write_text(json.dumps(result,indent=2)+'\n')
        (folder/'manual_reference.json').write_text(json.dumps(case,indent=2)+'\n')
        rows.append(dict(view=name,match_results=matches,**{k:v for k,v in result.items() if k!='candidates'},
                         unmatched_candidates=[dict(label=o['label'],status=o['status'],center_px=o['center_px'],reasons=o['reasons'])
                                               for i,o in enumerate(result['candidates']) if i not in used]))
        tile=np.zeros((280,424,3),np.uint8)
        tile[:240]=cv2.resize(view,(424,240))
        cv2.putText(tile,name,(8,260),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
        tiles.append(tile)
    canvas=np.zeros((840,848,3),np.uint8)
    for i,tile in enumerate(tiles):
        canvas[i//2*280:(i//2+1)*280,i%2*424:(i%2+1)*424]=tile
    cv2.imwrite(str(args.output/'contact_sheet.jpg'),canvas)
    all_matches=[m for row in rows for m in row['match_results']]
    summary=dict(scope='Five archival real views; some bags partly occluded; hand-selected tabletop and static exclusion masks; approximate manual outlines, no blind accuracy test',
                 manually_outlined_reference_bags=9, matched_reference_bags=len(errors),
                 candidate_reference_bags=sum(m.get('status')=='candidate' for m in all_matches),
                 review_required_reference_bags=sum(m.get('status')=='review_required' for m in all_matches),
                 mean_approximate_center_error_px=float(np.mean(errors)) if errors else None,
                 max_approximate_center_error_px=max(errors,default=None),
                 no_metric_or_grasp_validation=True, motion_authorized=False, views=rows)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    body=''.join(f'<h2>{html.escape(name)}</h2><img src="{name}/annotated.jpg"><p><a href="{name}/roi.json">ROI config</a> | <a href="{name}/detections.json">detections</a></p>' for name in CASES)
    reference=Path('outputs/analysis/beijing_kuaidi_reference_20261016').resolve()
    if (reference/'first_cycle_ep000_t010.0.jpg').exists():
        folder=args.output/'real_video_scale'
        folder.mkdir(exist_ok=True)
        config=dict(image_size_wh=[848,480], workspace_px=[[334,210],[538,210],[538,365],[334,365]],
                    exclude_px=[],background_sample_px=[[502,216],[532,216],[532,236],[502,236]],
                    background_image=str(reference/'focus_ep000_t0000.0.jpg'),
                    thresholds=dict(chroma_distance=7,background_delta=18),
                    note='Scale-region illustration ONLY, fixed camera. Does not distinguish an entering robot hand. Arm-clear interlock required before sampling. Not a validated grasp controller.')
        (folder/'roi.json').write_text(json.dumps(config,indent=2)+'\n')
        bg=cv2.imread(config['background_image'])
        for stem in ['focus_ep000_t0000.0','first_cycle_ep000_t010.0','focus_ep000_t0057.8']:
            image=cv2.imread(str(reference/(stem+'.jpg')))
            result,mask=detect(image,config,bg)
            cv2.imwrite(str(folder/(stem+'_annotated.jpg')),annotate(image,result,config))
            cv2.imwrite(str(folder/(stem+'_mask.png')),mask)
            (folder/(stem+'.json')).write_text(json.dumps(result,indent=2)+'\n')
        body+='<h2>银白包裹：流程视频单帧示例</h2><p>使用同视角空秤背景作差；仅处理秤上区域。相机或秤移动后背景失效；手臂进入时仍可能误检。示例像素中心约 (428,286)，不是机器人 XYZ。</p><img src="real_video_scale/first_cycle_ep000_t010.0_annotated.jpg"><p><a href="real_video_scale/replay/replay_h264.mp4">完整流程视频检测回放（含手臂干扰，不能直接用于抓取）</a></p><video controls preload="metadata" style="width:100%" src="real_video_scale/replay/replay_h264.mp4"></video>'
    body+='''<h2>现场使用顺序</h2><ol>
<li>固定头部相机、桌子和曝光，双臂退出取图区；保存一张当前 RGB 图。包裹分开放置、轮廓之间留空隙。</li>
<li>用 mark 命令圈选桌面取图区、秤/箱/机械臂排除区域和一块裸露桌布作颜色参照。每个视角单独配置；静态机械臂排除区域不支持追踪活动手臂。</li>
<li>用 run 读取新图或视频。绿色轮廓为待验证候选；紫色为触边、遮挡或形状异常。橙色点只表示距当前可见轮廓边缘较远，尚未验证抓取。</li>
<li>浅色包裹与桌布对比不足时，可在配置中设置 background_image 为同相机姿态的空桌照片路径；相机/桌子/秤移动后重新采集背景。</li>
<li>需要桌面米制坐标时，在配置 table_calibration 中提供 image_points_px 和 table_points_m 两组至少四个非共线对应点；应校正镜头畸变并检查独立测量点误差。代码会先投影完整轮廓再计算桌面中心。包裹厚度造成视差，这不是实测表面高度。</li>
<li>AB 两点用于建立桌面到机器人坐标关系，不能单独确定任意视角的图像透视变换。不需要机械臂接触不可达的 CD；图像中的已知平面对应点可用桌边标记获得。</li>
<li>先独立验收视觉、再验证悬停点。视觉候选、颜色判定和手指闭合均不能证明抓取成功。</li></ol>
<p>像素原点在图像左上角，x 向右、y 向下。本次仅测试历史图片/视频；绿色包裹只有合成颜色检查，尚无实拍验证。九处人工参照来自同批照片与参数调整过程，不能据此宣称比赛准确率。</p>
<h2>运行命令</h2><pre>/usr/bin/python3 tools/kuavo_parcel_centers.py mark --image /tmp/head.jpg --config outputs/parcel_roi.json --exclusions 2
/usr/bin/python3 tools/kuavo_parcel_centers.py run --input /tmp/head.jpg --config outputs/parcel_roi.json --output outputs/parcel_live --watch --show</pre>
<p>上述 /tmp/head.jpg 需要现有相机采集程序持续更新。watch 只读取本地文件，不连接机器人。mark 中依次左键选多边形顶点、右键撤销、回车保存、Esc 取消；两个排除区可分别圈选秤和箱。已有排除区配置必须随物体/机械臂移动而更新。系统 Python 支持窗口；letools 环境的 OpenCV 为无窗口版本，可省略 --show 输出文件。</p>'''
    (args.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Parcel centers: offline preview</title><style>body{background:#181818;color:#eee;font:16px sans-serif;max-width:950px;margin:auto}img{width:100%}a{color:#8cf}</style><h1>无 YOLO 包裹中心定位：离线检查</h1><p>绿色：轮廓候选；紫色：触边/遮挡等需要复核；橙色点：轮廓内部候选点；蓝色细线：人工粗略参照。ROI 和机械臂/秤排除区域均按视角手工圈选。没有机器人动作，没有验证毫米精度或抓取成功率。</p>'+body)
    print(json.dumps({k:v for k,v in summary.items() if k!='views'},ensure_ascii=False,indent=2))
    for row in rows:
        print(row['view'], [(m['label'],round(m.get('center_error_px',-1),1),m.get('status')) for m in row['match_results']],row['unmatched_candidates'])


if __name__=='__main__':
    main()

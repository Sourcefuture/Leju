#!/usr/bin/env python3
"""Repair only the unreadable title row of a completed simulation replay."""
import argparse,json
from pathlib import Path
import cv2
from render_kuavo_planar_task import caption_strip
from kuavo_video_codec import finalize as finalize_video


def fix(video,font=None):
    meta=video.with_suffix('.json');report=json.loads(meta.read_text())
    if report.get('caption_font') or not report.get('sensor_panel'):return False
    text=f'{report.get("playback_speed",1):g}x simulation time | '+report.get('label',video.parent.name)
    strip,chosen=caption_strip(text,font)
    if strip is None:raise ValueError('A CJK font is required to repair UTF-8 captions')
    capture=cv2.VideoCapture(str(video));fps=capture.get(cv2.CAP_PROP_FPS)
    width=int(capture.get(cv2.CAP_PROP_FRAME_WIDTH));height=int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (width,height)!=(1280,720):raise ValueError('Unexpected replay dimensions')
    temporary=video.with_name(video.stem+'_caption_work.mp4')
    writer=cv2.VideoWriter(str(temporary),cv2.VideoWriter_fourcc(*'mp4v'),fps,(width,height));count=0;last_frame=None
    if not writer.isOpened():raise RuntimeError('Caption video writer failed')
    try:
        while True:
            ok,frame=capture.read()
            if not ok:break
            frame[42:74]=strip;writer.write(frame);count+=1;last_frame=frame
    finally:capture.release();writer.release()
    if count!=report['frames']:raise ValueError('Incomplete original replay; do not replace it')
    temporary.replace(video);report.update(caption_font=chosen,
        caption_repair='Only title row 42..73 replaced; all motion stages/frame timing preserved, video re-encoded')
    report.update(finalize_video(video,count))
    meta.write_text(json.dumps(report,indent=2)+'\n')
    cv2.imwrite(str(video.with_suffix('.png')),last_frame if count else strip)
    return True


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--video',type=Path,required=True)
    p.add_argument('--font',type=Path);a=p.parse_args();print({'video':str(a.video),'changed':fix(a.video,a.font)})


if __name__=='__main__':main()

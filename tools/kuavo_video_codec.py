#!/usr/bin/env python3
"""Finalize local replay videos as browser-compatible H.264/YUV420 MP4."""
import argparse,json
from fractions import Fraction
from pathlib import Path
import av


def finalize(video,expected_frames=None):
    video=Path(video);temporary=video.with_name(video.stem+'_h264_work.mp4')
    with av.open(str(video)) as source:
        stream=source.streams.video[0]
        if stream.codec_context.name=='h264':return {'video_codec':'h264','video_pixel_format':stream.codec_context.format.name}
        rate=stream.average_rate or Fraction(5,1);count=0
        with av.open(str(temporary),'w',format='mp4',options={'movflags':'+faststart'}) as target:
            encoded=target.add_stream('libx264',rate=rate)
            encoded.width=stream.width;encoded.height=stream.height;encoded.pix_fmt='yuv420p'
            encoded.codec_context.thread_count=1;encoded.options={'preset':'veryfast','crf':'23'}
            for frame in source.decode(stream):
                frame=frame.reformat(width=encoded.width,height=encoded.height,format='yuv420p')
                frame.pts=count;frame.time_base=1/rate
                for packet in encoded.encode(frame):target.mux(packet)
                count+=1
            for packet in encoded.encode():target.mux(packet)
    if expected_frames is not None and count!=expected_frames:
        raise ValueError('Replay frame count changed: preserve original video')
    with av.open(str(temporary)) as test:
        if test.streams.video[0].codec_context.name!='h264':raise ValueError('H.264 encoder did not finalize')
    temporary.replace(video)
    return {'video_codec':'h264','video_pixel_format':'yuv420p','encoded_frames':count,
            'container_faststart':True,'transcode_scope':'same frames and timing, lossy browser-compatible encoding'}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--video',type=Path,required=True);a=p.parse_args()
    meta=a.video.with_suffix('.json');report=json.loads(meta.read_text()) if meta.is_file() else {}
    result=finalize(a.video,report.get('frames'));report.update(result)
    meta.write_text(json.dumps(report,indent=2)+'\n');print(result)


if __name__=='__main__':main()

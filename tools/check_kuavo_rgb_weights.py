#!/usr/bin/env python3
"""Independent replay of archived RGB OCR and comparison with private load-cell evidence."""
import argparse,json
from pathlib import Path
import cv2
from kuavo_scale_vision import read_scale_sequence


def check(state):
    result=json.loads((state/'closed_loop_result.json').read_text());rows=[];errors=[]
    for entry in result.get('results',[]):
        if entry['result'].get('status')!='executed':continue
        for recorded in entry['result'].get('weights',[]):
            bag=recorded['bag'];folder=Path(recorded['rgb_evidence_file']).parent
            images=sorted(folder.glob(bag+'_rgb_*.png'),key=lambda p:int(p.stem.rsplit('_',1)[-1]))
            frames=[]
            for file in images:
                bgr=cv2.imread(str(file))
                if bgr is None:raise ValueError('Unreadable archived RGB: '+str(file))
                frames.append(cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB))
            reading,_=read_scale_sequence(frames,minimum=3)
            # Private measurement is used by this scorer only, never OCR/planning.
            private=json.loads((folder/(bag+'_device_diagnostic.json')).read_text())
            difference=abs(reading['measured_kg']-private['measured_kg'])*1000
            passed=difference<=.11 and reading['digits']==recorded['digits']
            row={'bag':bag,'passed':passed,'recorded_grams':recorded['display_grams'],
                 'redecoded_grams':reading['display_grams'],'private_measured_grams':private['measured_kg']*1000,
                 'difference_grams':difference,'accepted_frame_indices':reading['accepted_frame_indices'],
                 'selected_rgb_file':str(images[reading['selected_frame_index']].resolve()),
                 'evidence_directory':str(folder.resolve())}
            rows.append(row)
            if not passed:errors.append(row)
    return {'passed':bool(rows) and not errors,'readings':rows,'violations':errors,
            'scope':'redecode saved input RGB without expected weight; private simulated load cell used solely for independent scoring, not real scale accuracy'}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--state-dir',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();result=check(a.state_dir)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 2)


if __name__=='__main__':main()

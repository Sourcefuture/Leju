#!/usr/bin/env python3
"""Export exact-size vector PDF marker sheets using the experiment dictionary."""
import argparse
from pathlib import Path
import cv2

PT=72/25.4


def sheet(ids,title):
    commands=['0 g']
    def text(x,y,value,size=9):
        escaped=value.replace('\\','\\\\').replace('(','\\(').replace(')','\\)')
        commands.append(f'BT /F1 {size} Tf {x*PT:.4f} {y*PT:.4f} Td ({escaped}) Tj ET')
    text(18,292,title,11)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for index,ident in enumerate(ids):
        x=18+(index%2)*99;y=210-(index//2)*93
        bitmap=cv2.aruco.generateImageMarker(dictionary,ident,180)
        for row in range(6):
            for col in range(6):
                if bitmap[row*30+15,col*30+15]<128:
                    left=x+7.5+col*10;bottom=y+7.5+(5-row)*10
                    commands.append(f'{left*PT:.4f} {bottom*PT:.4f} {10*PT:.4f} {10*PT:.4f} re f')
        text(x,y-5,f'ID {ident} | black 60 mm | paper patch 75 mm',8)
    if len(ids)<6:
        text(18,98,'Device markers must follow the object on a rigid extension base.',9)
        commands.append(f'{18*PT:.4f} {76*PT:.4f} m {68*PT:.4f} {76*PT:.4f} l S')
        text(18,71,'50 mm ruler: verify print scale before calibration.',9)
    text(18,8,'Print at 100 percent / Actual size. Do not use fit-to-page.',9)
    return '\n'.join(commands).encode('ascii')


def write_pdf(path,table_ids=None):
    table_ids=[0,1,2,3,4,5] if table_ids is None else table_ids
    pages=[sheet(table_ids[i:i+6],'Kuavo tabletop reference markers') for i in range(0,len(table_ids),6)]
    pages.append(sheet([10,12,11,13],'Kuavo scale (10,12) and bin (11,13) base markers'))
    kids=' '.join(f'{4+index*2} 0 R' for index in range(len(pages)))
    objects=[b'<< /Type /Catalog /Pages 2 0 R >>',
             f'<< /Type /Pages /Kids [{kids}] /Count {len(pages)} >>'.encode('ascii'),
             b'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>']
    for index,content in enumerate(pages):
        stream_id=5+index*2
        objects.append(f'<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {210*PT:.4f} {297*PT:.4f}] /Resources << /Font << /F1 3 0 R >> >> /Contents {stream_id} 0 R >>'.encode('ascii'))
        objects.append(f'<< /Length {len(content)} >>\nstream\n'.encode('ascii')+content+b'\nendstream')
    output=bytearray(b'%PDF-1.4\n');offsets=[0]
    for index,obj in enumerate(objects,1):
        offsets.append(len(output));output.extend(f'{index} 0 obj\n'.encode('ascii')+obj+b'\nendobj\n')
    xref=len(output);output.extend(f'xref\n0 {len(offsets)}\n0000000000 65535 f \n'.encode('ascii'))
    for offset in offsets[1:]:output.extend(f'{offset:010d} 00000 n \n'.encode('ascii'))
    output.extend(f'trailer\n<< /Size {len(offsets)} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n'.encode('ascii'))
    path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(output)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('outputs/kuavo_sim/planar_rgb_weighing/markers_A4.pdf'))
    parser.add_argument('--table-ids',type=int,nargs='+',default=[0,1,2,3,4,5])
    args=parser.parse_args()
    if any(i<0 or i>=50 or i in [10,11,12,13] for i in args.table_ids):raise ValueError('Invalid or device-reserved table marker ID')
    write_pdf(args.output,args.table_ids);print(args.output)

#!/usr/bin/env python3
import argparse,json,struct
from pathlib import Path
def pgm_size(p):
    with open(p,'rb') as f:
        magic=f.readline().strip(); dims=f.readline().split(); maxv=f.readline().strip()
    return magic.decode(),int(dims[0]),int(dims[1]),int(maxv)
def main():
    a=argparse.ArgumentParser(); a.add_argument('--dataset',required=True); a.add_argument('--predictions'); x=a.parse_args(); d=Path(x.dataset); rows=[json.loads(z) for z in (d/'ground_truth.jsonl').read_text().splitlines() if z.strip()]
    errors=[]
    for r in rows:
        p=d/r['image']
        try: m,w,h,mv=pgm_size(p)
        except Exception as e: errors.append(f'{p}: {e}'); continue
        if (m,w,h,mv)!=('P5',640,360,255): errors.append(f'{p}: invalid PGM {m,w,h,mv}')
    if errors: print('\n'.join(errors)); raise SystemExit(2)
    print(f'fixture_integrity: PASS ({len(rows)} frames, {len(set(r["expected_quality"] for r in rows))} quality classes)')
    if x.predictions:
        pred=[json.loads(z) for z in Path(x.predictions).read_text().splitlines() if z.strip()]; by={r.get('image'):r for r in pred}; found=0; matched=0
        for r in rows:
            q=by.get(r['image']);
            if q and q.get('found'): found+=1; matched+=abs(float(q.get('center_x',320))-320)<=max(2,float(q.get('bbox_w',10))*.03)
        print(f'predictions: {len(pred)} found={found} center_within_3pct={matched}/{found or 1}')
if __name__=='__main__': main()

#!/usr/bin/env python3
"""Deterministic synthetic six-component marker dataset (stdlib only)."""
import argparse, json, math, random
from pathlib import Path

W,H=640,360
# Coordinates in mm, origin at board centre. Three large Ls, one small L, two squares.
# Coordinates recovered from 3D/Objects/object_4.model (x,z -> x,y image
# plane). The three large L bounding boxes are [-40,-10]x[10,40],
# [10,40]x[-40,-10], [-40,-10]x[-40,-10]; the asymmetric components are
# [26,40]x[26,40], [10,18]x[32,40], [32,40]x[10,18].
GEOMETRY=[{"name":"large_l_0","kind":"L","x":-25,"y":25,"size":30},
 {"name":"large_l_1","kind":"L","x":25,"y":-25,"size":30},
 {"name":"large_l_2","kind":"L","x":-25,"y":-25,"size":30},
 {"name":"small_l","kind":"L","x":33,"y":33,"size":14},
 {"name":"square_0","kind":"square","x":14,"y":36,"size":8},
 {"name":"square_1","kind":"square","x":36,"y":14,"size":8}]

def inside_component(x,y,c):
    # L is two bars; coordinates are component centre. This intentionally permits glow.
    s=c['size']; u=x-c['x']+s/2; v=y-c['y']+s/2
    if c['kind']=='square': return 0<=u<s and 0<=v<s
    t=max(2.5,s*.27); return (0<=u<s and 0<=v<t) or (0<=u<t and 0<=v<s)
def frame(seed, angle, scale, bright, blur, noise, stripe, dropout, black_bg):
    rng=random.Random(seed); a=math.radians(angle); ca,sa=math.cos(a),math.sin(a)
    # 115mm board projected to a convenient synthetic bbox; metadata retains scale.
    cx,cy=320,180; px_per_mm=scale
    img=[[0 if black_bg else 18 for _ in range(W)] for __ in range(H)]
    # Physical black body: approximately 115 x 120 mm.  On a non-black
    # background it is deliberately dimmer than the room, while the
    # black-background variant exercises the documented board/background
    # ambiguity and leaves LED topology as the only usable cue.
    if not black_bg:
        for yy in range(H):
            for xx in range(W):
                X=(xx-cx)/px_per_mm; Y=(yy-cy)/px_per_mm
                bx=ca*X+sa*Y; by=-sa*X+ca*Y
                if -57.5<=bx<=57.5 and -60<=by<=60: img[yy][xx]=5
    components=[]
    for idx,c in enumerate(GEOMETRY):
        if dropout and idx in dropout: continue
        pts=[]
        r=max(1,int(c['size']*px_per_mm*.72)); ox=c['x']*px_per_mm; oy=c['y']*px_per_mm
        for yy in range(max(0,int(c['y']*px_per_mm+cy-r)),min(H,int(c['y']*px_per_mm+cy+r+2))):
            for xx in range(max(0,int(c['x']*px_per_mm+cx-r)),min(W,int(c['x']*px_per_mm+cx+r+2))):
                X=(xx-cx)/px_per_mm; Y=(yy-cy)/px_per_mm
                x=ca*X+sa*Y; y=-sa*X+ca*Y
                if inside_component(x,y,c):
                    val=bright
                    if stripe and ((yy+seed)%stripe)<max(1,stripe//3): val=int(bright*.25)
                    img[yy][xx]=max(img[yy][xx],val); pts.append((xx,yy))
        if pts: components.append({"name":c['name'],"pixels":len(pts),"bbox":[min(x for x,y in pts),min(y for x,y in pts),max(x for x,y in pts)+1,max(y for x,y in pts)+1]})
    # box blur, bounded and deterministic
    for _ in range(max(0,min(3,blur))):
        src=img; out=[[0]*W for __ in range(H)]
        for y in range(H):
            for x in range(W):
                vals=[src[yy][xx] for yy in range(max(0,y-1),min(H,y+2)) for xx in range(max(0,x-1),min(W,x+2))]
                out[y][x]=sum(vals)//len(vals)
        img=out
    for y in range(H):
        for x in range(W): img[y][x]=max(0,min(255,int(img[y][x]+rng.gauss(0,noise))))
    return img,components
def write_pgm(path,img):
    with open(path,'wb') as f:
        f.write(f'P5\n{W} {H}\n255\n'.encode()); f.write(bytes(v for row in img for v in row))
def main():
    p=argparse.ArgumentParser(); p.add_argument('--out',required=True); p.add_argument('--count',type=int,default=48); p.add_argument('--seed',type=int,default=7); args=p.parse_args()
    out=Path(args.out); out.mkdir(parents=True,exist_ok=True); rng=random.Random(args.seed); rows=[]
    for i in range(args.count):
        angle=rng.uniform(-25,25); scale=rng.uniform(.8,2.3); bright=rng.choice([150,200,245]); blur=rng.choice([0,0,1,2]); noise=rng.choice([0,2,5,10]); stripe=rng.choice([0,0,7,11]); dropout=[] if rng.random()>.15 else [rng.randrange(3,6)]; black=rng.random()<.35
        img,comps=frame(args.seed+i,angle,scale,bright,blur,noise,stripe,dropout,black); name=f'{i:04d}.pgm'; write_pgm(out/name,img)
        xs=[q['bbox'][0] for q in comps]; ys=[q['bbox'][1] for q in comps]; xe=[q['bbox'][2] for q in comps]; ye=[q['bbox'][3] for q in comps]
        rows.append({"image":name,"width":W,"height":H,"center":[320,180],"angle_deg":angle,"scale_px_per_mm":scale,"brightness":bright,"blur_passes":blur,"noise_sigma":noise,"stripe_period":stripe,"dropped_components":dropout,"black_background":black,"components":comps,"expected_quality":"FULL_ID" if len(comps)==6 else "TRACKABLE"})
    (out/'ground_truth.jsonl').write_text('\n'.join(json.dumps(x) for x in rows)+'\n',encoding='utf-8'); (out/'geometry.json').write_text(json.dumps(GEOMETRY,indent=2),encoding='utf-8'); print(f'generated {len(rows)} frames in {out}')
if __name__=='__main__': main()

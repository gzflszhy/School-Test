#!/usr/bin/env python3
"""Offline detector benchmark adapter.

The adapter command receives one PGM path and must print one JSON object with
processing_ms, capture_to_output_latency_ms, found and (optionally) center_x.
This keeps the benchmark independent of a particular detector executable.
"""
import argparse, json, statistics, subprocess, time
from pathlib import Path
def pct(v,p):
    if not v:return None
    q=(len(v)-1)*p; lo=int(q); hi=min(lo+1,len(v)-1); return v[lo]+(v[hi]-v[lo])*(q-lo)
def main():
    a=argparse.ArgumentParser(); a.add_argument('--dataset',required=True); a.add_argument('--adapter',nargs='+',help='command with {image} placeholder'); a.add_argument('--output'); x=a.parse_args(); d=Path(x.dataset); gt=[json.loads(z) for z in (d/'ground_truth.jsonl').read_text().splitlines()]
    if not x.adapter: print(json.dumps({'frames':len(gt),'note':'no adapter supplied; fixture-only benchmark'})); return
    out=[]; start=time.perf_counter()
    for r in gt:
        cmd=[s.replace('{image}',str(d/r['image'])) for s in x.adapter]; raw=subprocess.check_output(cmd,text=True).strip().splitlines()[-1]; out.append(json.loads(raw))
    elapsed=(time.perf_counter()-start)*1000; proc=sorted(float(r.get('processing_ms',0)) for r in out); lat=sorted(float(r.get('capture_to_output_latency_ms',0)) for r in out); found=sum(bool(r.get('found')) for r in out)
    result={'frames':len(out),'found':found,'wall_ms':elapsed,'effective_fps':len(out)/(elapsed/1000) if elapsed else None,'processing_ms':{'mean':statistics.mean(proc),'p50':pct(proc,.5),'p95':pct(proc,.95),'p99':pct(proc,.99)} if proc else None,'latency_ms':{'mean':statistics.mean(lat),'p95':pct(lat,.95)} if lat else None}
    text=json.dumps(result,indent=2); print(text)
    if x.output: Path(x.output).write_text(text+'\n',encoding='utf-8')
if __name__=='__main__':main()

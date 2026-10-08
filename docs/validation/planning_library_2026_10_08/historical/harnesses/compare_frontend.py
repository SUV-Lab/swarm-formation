import argparse, hashlib, json
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument('before');p.add_argument('after');p.add_argument('output')
a=p.parse_args()
def clean(v):
    if isinstance(v,dict): return {k:clean(x) for k,x in v.items() if k!='stamp'}
    if isinstance(v,list): return [clean(x) for x in v]
    return v
before=json.loads(Path(a.before).read_text());after=json.loads(Path(a.after).read_text())
result={'ignored_fields':'ROS header timestamps only','topics':{}}
for topic in ['front_end_paths','front_end_tubes']:
    x=[clean(m['message']) for m in before[topic]]
    y=[clean(m['message']) for m in after[topic]]
    result['topics'][topic]={'before_count':len(x),'after_count':len(y),'exact':x==y and len(x)>0,'sha256':hashlib.sha256(json.dumps(y,sort_keys=True).encode()).hexdigest()}
result['pass']=all(v['exact'] for v in result['topics'].values())
Path(a.output).write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
raise SystemExit(0 if result['pass'] else 1)

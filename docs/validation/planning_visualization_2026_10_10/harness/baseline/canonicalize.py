import json,sys,math,collections,hashlib,struct
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict

def canonical(v,key=''):
 if isinstance(v,dict):
  return {k:({'sec':0,'nanosec':0} if key=='header' and k=='stamp' else canonical(x,k)) for k,x in v.items()}
 if isinstance(v,(list,tuple)):return [canonical(x) for x in v]
 if isinstance(v,float):return {'float_hex':v.hex(),'binary64':struct.pack('>d',v).hex()}
 return v
root=Path(sys.argv[1]);outputs={};stats={}
for part in ['events','late_full','late_clear']:
 rows=[]
 for line in (root/part/'index.tsv').read_text().splitlines():
  phase,topic,typ,name=line.split('\t');raw=(root/part/name).read_bytes();m=deserialize_message(raw,get_message(typ));d=canonical(message_to_ordereddict(m));rows.append({'phase':phase,'topic':topic,'type':typ,'payload':d})
 # Topic receipt order is not a semantic cross-topic order guarantee.
 groups=collections.defaultdict(list)
 for row in rows:groups[row['phase']+'|'+row['topic']].append(row['payload'])
 outputs[part]=dict(sorted(groups.items()));stats[part]={'messages':len(rows),'topics':dict(collections.Counter(x['topic'] for x in rows))}
 (root/(part+'_canonical.json')).write_text(json.dumps(outputs[part],sort_keys=True,separators=(',',':'))+'\n')
# Late subscriber values must equal last live output before/after clears.
rows=[]
for line in (root/'events/index.tsv').read_text().splitlines():
 p,t,typ,name=line.split('\t'); rows.append((p,t,canonical(message_to_ordereddict(deserialize_message((root/'events'/name).read_bytes(),get_message(typ))))))
last={};before={}
for p,t,d in rows:
 if p=='trajectory_clear' and not before:before=dict(last)
 last[t]=d
checks={}
for phase,expected in [('late_full',before),('late_clear',last)]:
 actual={key.split('|',1)[1]:value[-1] for key,value in outputs[phase].items()}
 checks[phase+'_complete']=set(actual)==set(expected) and all(actual[t]==d for t,d in expected.items())
 checks[phase+'_topic_count']=len(actual)
expected_topics = 0 if '--disconnected' in sys.argv[2:] else 11
checks['all_expected_topics']=len(stats['events']['topics'])==expected_topics
checks['expected_topic_count']=expected_topics
(root/'canonical_summary.json').write_text(json.dumps({'stats':stats,'checks':checks,'normalization':'Only header.stamp sec/nanosec zeroed; all other fields retained. Float values retain hex and IEEE binary64 bits (lossless promotion for finite float32; NaN sign/payload retained by Python conversion). Receipt order retained within phase/topic.'},indent=2)+'\n')
print(json.dumps({'stats':stats,'checks':checks},indent=2))
assert checks['all_expected_topics'] and checks['late_full_complete'] and checks['late_clear_complete']

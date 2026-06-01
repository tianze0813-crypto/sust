import json
from pathlib import Path
kw = ('rotation','yaw','direction','orientation','angle','euler','heading')
d = json.loads(Path('graphify-out/.graphify_ast.json').read_text(encoding='utf-8'))
nodes = d.get('nodes', [])
edges = d.get('edges', [])
match_nodes = [n for n in nodes if any(k in (str(n.get('label','')) + ' ' + str(n.get('id',''))).lower() for k in kw)]
node_ids = {n['id'] for n in match_nodes}
match_edges = [e for e in edges if e.get('source') in node_ids or e.get('target') in node_ids or any(k in str(e.get('relation','')).lower() for k in kw)]
print('MATCH_NODES', len(match_nodes))
for n in match_nodes[:80]:
    print(f"NODE|{n.get('label','')}|{n.get('source_file','')}")
print('MATCH_EDGES', len(match_edges))
for e in match_edges[:120]:
    print(f"EDGE|{e.get('source')}|{e.get('relation')}|{e.get('target')}|{e.get('source_file','')}")

import json
from pathlib import Path
from graphify.extract import extract
code_files = [Path('main.py'), Path('scene_reader.py')]
code_files += sorted(Path('algos').glob('*.py'))
code_files += sorted(Path('tools').glob('*.py'))
code_files += sorted(Path('public/js').glob('*.js'))
result = extract(code_files)
Path('graphify-out/.graphify_ast.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
print('AST nodes=%d edges=%d' % (len(result.get('nodes', [])), len(result.get('edges', []))))

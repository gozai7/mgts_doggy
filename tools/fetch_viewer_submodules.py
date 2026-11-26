#!/usr/bin/env python3
"""Build-time source fetch: exact commits from the parent tree, no branch heads."""
import json, subprocess
from pathlib import Path
root=Path(__file__).resolve().parents[1]
for item in json.loads((root/'docker/competition/viewer-submodules.lock.json').read_text()):
 dest=root/'vendor/iridescence'/item['path']
 if (dest/'.git').exists():
  rev=subprocess.check_output(['git','-C',str(dest),'rev-parse','HEAD'],text=True).strip()
  if rev!=item['commit']:raise RuntimeError(f'Unexpected existing revision at {dest}')
  continue
 dest.mkdir(parents=True,exist_ok=True)
 subprocess.run(['git','init','-q',str(dest)],check=True)
 subprocess.run(['git','-C',str(dest),'remote','add','origin',item['url']],check=True)
 subprocess.run(['git','-C',str(dest),'fetch','--depth','1','origin',item['commit']],check=True)
 subprocess.run(['git','-C',str(dest),'checkout','--detach',item['commit']],check=True)

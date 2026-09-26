#!/usr/bin/env python3
"""Create an allowlisted, history-free release snapshot; never copies .git or deletes input."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import time

ROOT_FILES=['README.md','.gitignore','NOTICE.md']
ACCEPTANCE=['conftest.py','test_teleop_core.py','test_panel_runtime.py','teleop_integration.py',
            'joy_calibrate.py','teleop_physical_acceptance.py']
TOOLS=['prepare_release.py','check_release.py']
SKIP={'__pycache__','.pytest_cache','build','install','log','.git'}

def selected(root):
    for name in ROOT_FILES: yield root/name
    for package in ['mobile_manipulator_description','mobile_manipulator_gazebo']:
        folder=root/'src'/package
        for p in sorted(folder.rglob('*')):
            if not p.is_file() or any(part in SKIP for part in p.relative_to(folder).parts): continue
            if p.name=='README.md': continue  # Historical subdirectory notes; current public guide is at root.
            yield p
    for p in sorted((root/'docs').rglob('*')):
        if p.is_file() and not any(part in SKIP for part in p.relative_to(root).parts): yield p
    for name in ACCEPTANCE: yield root/'tools/acceptance'/name
    for name in TOOLS: yield root/'tools'/name
    for name in ['stage5_room_20260927.yaml','stage5_room_20260927.pgm']: yield root/'maps'/name

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    root=args.source.resolve()
    dest=(args.output or root/'release'/('candidate_'+time.strftime('%Y%m%d_%H%M%S'))).resolve()
    if dest==root or root.is_relative_to(dest): parser.error('Output must not be the source or an ancestor.')
    if dest.exists(): parser.error('Output already exists; choose a new directory. No files were removed.')
    paths=list(selected(root))
    for p in paths:
        if not p.is_file(): parser.error('Missing release file: '+str(p.relative_to(root)))
        if p.is_symlink() or not p.resolve().is_relative_to(root): parser.error('External/symlink input: '+str(p))
    dest.mkdir(parents=True)
    manifest={}
    for source in paths:
        relative=source.relative_to(root)
        target=dest/relative;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source,target)
        manifest[str(relative)]={'bytes':target.stat().st_size,'sha256':hashlib.sha256(target.read_bytes()).hexdigest()}
    (dest/'RELEASE_MANIFEST.json').write_text(json.dumps({'format':1,'contains_git_history':False,'files':manifest},indent=2)+'\n')
    print(dest)
    print(f'{len(manifest)} allowlisted files; no .git, local handoff, raw validation or runtime sessions copied.')

if __name__=='__main__': main()

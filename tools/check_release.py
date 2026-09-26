#!/usr/bin/env python3
"""Static release audit. Report file/line locations, never matching secret values."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

PATTERNS={
    'private_key':r'-----BEGIN (?:RSA |OPENSSH |EC |DSA )?PRIVATE KEY-----',
    'credential_token':r'\b(?:gh[pousr]_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|sk-[A-Za-z0-9_-]{24,})\b',
    'credential_assignment':r'''(?i)(?:password|access_token|api_key|secret_key)\s*[:=]\s*["'][^"'\s]{8,}["']''',
    'personal_home':r'/(?:home|Users)/[A-Za-z0-9_.-]+',
    'windows_user_path':r'[A-Za-z]:[\\/](?:Users|chatgpt)[\\/]',
    'private_ipv4':r'\b(?:192\.168\.\d{1,3}\.\d{1,3}|10\.\d{1,3}\.\d{1,3}\.\d{1,3})\b',
    'ssh_connection':r'\bssh\s+(?:[A-Za-z0-9_-]+@|my-ros-)',
}
EXCLUDE={'build','install','log','__pycache__','.pytest_cache','.git','release'}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path,nargs='?',default=Path.cwd())
    parser.add_argument('--report',type=Path)
    args=parser.parse_args();root=args.root.resolve()
    issues=[];files=[];total=0
    for path in sorted(root.rglob('*')):
        rel=path.relative_to(root)
        if any(part in EXCLUDE for part in rel.parts): continue
        if path.is_symlink(): issues.append({'file':str(rel),'reason':'symlink'});continue
        if not path.is_file(): continue
        files.append(path);total+=path.stat().st_size
        if path.stat().st_size>10*1024*1024: issues.append({'file':str(rel),'reason':'file exceeds 10 MiB review limit'})
        if path.name in ['AGENTS.md','HANDOFF.md'] or 'validation' in rel.parts:
            issues.append({'file':str(rel),'reason':'local history/raw validation should not be in release'})
        raw=path.read_bytes()
        if path.suffix.lower() in ['.stl','.png','.pgm']:
            # Scan printable metadata/header text only, not arbitrary numeric/binary payload.
            text=raw[:256].decode('ascii',errors='ignore')
        else:
            try: text=raw.decode('utf-8-sig')
            except UnicodeDecodeError:
                issues.append({'file':str(rel),'reason':'unexpected binary type'});continue
        for kind,pattern in PATTERNS.items():
            for m in re.finditer(pattern,text):
                issues.append({'file':str(rel),'line':text.count('\n',0,m.start())+1,'reason':kind})
        if path.suffix=='.md':
            for target in re.findall(r'!?\[[^\]]*\]\(([^)]+)\)',text):
                if '://' in target or target.startswith('#'): continue
                target=target.split('#')[0]
                if target and not (path.parent/target).exists():
                    issues.append({'file':str(rel),'reason':'broken local documentation link','target':target})
    assets=json.loads((root/'docs/evidence/assets.json').read_text())['assets']
    actual={str(p.relative_to(root)) for p in files if p.suffix.lower()=='.stl'}
    if actual!={a['file'] for a in assets}: issues.append({'reason':'STL provenance inventory mismatch'})
    for a in assets:
        p=root/a['file']
        if not p.exists() or hashlib.sha256(p.read_bytes()).hexdigest()!=a['sha256']:
            issues.append({'file':a['file'],'reason':'asset hash mismatch'})
    for p in (root/'src').glob('*/package.xml'):
        try: ET.parse(p)
        except ET.ParseError: issues.append({'file':str(p.relative_to(root)),'reason':'invalid package XML'})
    for p in (root/'src').rglob('*.py'):
        try: compile(p.read_text(),str(p),'exec')
        except SyntaxError: issues.append({'file':str(p.relative_to(root)),'reason':'Python syntax error'})
    for launch in ['mapping_demo.launch.py','sim.launch.py','slam.launch.py','teleop.launch.py']:
        if not (root/'src/mobile_manipulator_gazebo/launch'/launch).exists(): issues.append({'reason':'missing launch','file':launch})
    manifest_path=root/'RELEASE_MANIFEST.json'
    if manifest_path.exists():
        manifest=json.loads(manifest_path.read_text())['files']
        actual_files={str(p.relative_to(root)) for p in files if p!=manifest_path}
        if actual_files!=set(manifest): issues.append({'reason':'release manifest file inventory mismatch'})
        for name,info in manifest.items():
            p=root/name
            if not p.is_file() or p.stat().st_size!=info['bytes'] or hashlib.sha256(p.read_bytes()).hexdigest()!=info['sha256']:
                issues.append({'file':name,'reason':'release manifest hash mismatch'})
    result={'pass':not issues,'files':len(files),'total_bytes':total,
            'largest_files':[{'file':str(p.relative_to(root)),'bytes':p.stat().st_size} for p in sorted(files,key=lambda p:p.stat().st_size,reverse=True)[:5]],
            'asset_count':len(assets),'issues':issues,
            'scope':'Pattern scan and curated asset inventory; not a guarantee that every possible secret can be detected.',
            'excluded_generated_directories':sorted(EXCLUDE),'git_history_audited':False}
    if args.report:
        args.report.parent.mkdir(parents=True,exist_ok=True);args.report.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['pass'] else 1)

if __name__=='__main__': main()

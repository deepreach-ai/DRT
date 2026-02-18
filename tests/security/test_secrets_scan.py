import re
from pathlib import Path
import subprocess

SECRET_PATTERNS = [
    re.compile(r'AKIA[0-9A-Z]{16}'),
    re.compile(r'(?i)secret[_-]?key\s*=\s*["\'][^"\']+["\']'),
    re.compile(r'-----BEGIN (?:RSA|EC|DSA|OPENSSH|PRIVATE) KEY-----'),
    re.compile(r'ghp_[A-Za-z0-9]{36}'),
]

EXCLUDE_DIRS = {'.git', '.venv', 'node_modules', 'build', 'dist', '.trae', 'librealsense'}
EXCLUDE_FILES = {'cacert.pem'}
TEXT_SUFFIXES = {'.py', '.ts', '.js', '.json', '.yml', '.yaml', '.env', '.toml', '.md', '.txt', '.html', '.css', '.sh', '.ini', '.pem', '.key'}
SELF_FILE = Path(__file__).resolve()

def scan_file(path: Path):
    findings = []
    try:
        text = path.read_text(errors='ignore')
    except Exception:
        return findings
    for pat in SECRET_PATTERNS:
        for m in pat.finditer(text):
            findings.append(f'{path}:{m.start()}:{pat.pattern}')
    return findings

def should_skip_path(p: Path) -> bool:
    parts = set(p.parts)
    if SELF_FILE == p.resolve():
        return True
    if parts & EXCLUDE_DIRS:
        return True
    return False

def get_tracked_files(root: Path):
    try:
        out = subprocess.run(["git", "ls-files"], cwd=root, capture_output=True, text=True, check=True)
        files = [root / Path(line.strip()) for line in out.stdout.splitlines() if line.strip()]
        files = [p for p in files if p.exists()]
        return files
    except Exception:
        return [p for p in root.rglob('*') if p.is_file()]

def collect_findings(root: Path):
    findings = []
    for p in get_tracked_files(root):
        if should_skip_path(p):
            continue
        if p.name in EXCLUDE_FILES:
            continue
        if p.suffix in TEXT_SUFFIXES or p.name in {'Dockerfile', 'Makefile'}:
            findings.extend(scan_file(p))
    return findings

def test_no_plaintext_secrets():
    root = Path(__file__).resolve().parents[2]
    findings = collect_findings(root)
    assert not findings, '发现潜在明文敏感信息:\n' + '\n'.join(findings)

def test_no_pem_or_key_files_in_repo():
    root = Path(__file__).resolve().parents[2]
    bad = []
    for p in get_tracked_files(root):
        if should_skip_path(p):
            continue
        if p.suffix in {'.pem', '.key'} and p.name not in EXCLUDE_FILES:
            bad.append(str(p))
    assert not bad, '仓库中不允许提交 .pem/.key 文件:\n' + '\n'.join(bad)

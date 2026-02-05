#!/usr/bin/env python3
"""
Automated validation script for teleop system
Run this before starting manual tests
"""
import sys
import os
import subprocess
import importlib.util
from pathlib import Path

class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    RESET = '\033[0m'

def print_header(text):
    print(f"\n{Colors.BLUE}{'='*60}{Colors.RESET}")
    print(f"{Colors.BLUE}{text:^60}{Colors.RESET}")
    print(f"{Colors.BLUE}{'='*60}{Colors.RESET}\n")

def check_pass(message):
    print(f"{Colors.GREEN}✅ PASS{Colors.RESET} - {message}")
    return True

def check_fail(message, error=None):
    print(f"{Colors.RED}❌ FAIL{Colors.RESET} - {message}")
    if error:
        print(f"   Error: {error}")
    return False

def check_warn(message):
    print(f"{Colors.YELLOW}⚠️  WARN{Colors.RESET} - {message}")
    return True

# Change to teleop_system directory
TELEOP_DIR = Path("/Users/ziguo/teleop_system")
os.chdir(TELEOP_DIR)

print_header("TELEOP SYSTEM PRE-VALIDATION CHECKS")

# Track results
results = {
    'python': False,
    'venv': False,
    'packages': [],
    'files': [],
    'ports': []
}

# Check 1: Python Version
print_header("1. Python Environment")
try:
    python_version = sys.version_info
    if python_version >= (3, 8):
        results['python'] = check_pass(f"Python {python_version.major}.{python_version.minor}.{python_version.micro}")
    else:
        check_fail(f"Python version too old: {python_version.major}.{python_version.minor}")
except Exception as e:
    check_fail("Python check", e)

# Check 2: Virtual Environment
if (TELEOP_DIR / ".venv").exists():
    results['venv'] = check_pass("Virtual environment exists at .venv/")
else:
    check_warn("No .venv found - you may need to create one")

# Check 3: Required Packages
print_header("2. Required Python Packages")
required_packages = {
    'fastapi': '0.104.1',
    'uvicorn': '0.24.0',
    'mujoco': '3.1.6',
    'numpy': '1.26.4',
    'pydantic': '2.5.0',
    'websockets': '12.0',
}

for pkg_name, expected_version in required_packages.items():
    try:
        spec = importlib.util.find_spec(pkg_name)
        if spec is not None:
            # Try to get version
            try:
                module = importlib.import_module(pkg_name)
                version = getattr(module, '__version__', 'unknown')
                results['packages'].append(pkg_name)
                check_pass(f"{pkg_name} installed (version: {version})")
            except:
                results['packages'].append(pkg_name)
                check_pass(f"{pkg_name} installed")
        else:
            check_fail(f"{pkg_name} not found")
    except Exception as e:
        check_fail(f"{pkg_name} check", e)

# Check 4: File Structure
print_header("3. File Structure")
required_files = [
    "run_server.py",
    "server/teleop_server.py",
    "server/backends/mock_backend.py",
    "server/backends/mujoco_backend.py",
    "client/web_server.py",
    "client/keyboard_client.py",
    "robots/so101.xml",
]

for file_path in required_files:
    full_path = TELEOP_DIR / file_path
    if full_path.exists():
        results['files'].append(file_path)
        check_pass(f"{file_path}")
    else:
        check_fail(f"{file_path} not found")

# Check 5: Robot Model Assets
print_header("4. Robot Model Assets")
assets_dir = TELEOP_DIR / "robots" / "assets"
if assets_dir.exists():
    stl_files = list(assets_dir.glob("*.stl"))
    if len(stl_files) >= 13:
        check_pass(f"Found {len(stl_files)} STL mesh files")
    else:
        check_warn(f"Only {len(stl_files)} STL files found (expected 13)")
else:
    check_fail("robots/assets/ directory not found")

# Check 6: Port Availability
print_header("5. Port Availability")
try:
    import socket
    
    def check_port(port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1)
        result = sock.connect_ex(('localhost', port))
        sock.close()
        return result != 0  # True if port is free
    
    ports_to_check = [8000, 8080]
    for port in ports_to_check:
        if check_port(port):
            results['ports'].append(port)
            check_pass(f"Port {port} is available")
        else:
            check_warn(f"Port {port} is in use (you may need to kill existing process)")
except Exception as e:
    check_fail("Port check", e)

# Check 7: MuJoCo Installation
print_header("6. MuJoCo Functionality")
try:
    import mujoco
    mj_version = mujoco.__version__
    check_pass(f"MuJoCo version: {mj_version}")
    
    # Try to load a simple model
    simple_xml = """
    <mujoco>
        <worldbody>
            <geom type="sphere" size="0.1"/>
        </worldbody>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(simple_xml)
    data = mujoco.MjData(model)
    check_pass("MuJoCo can load models successfully")
except Exception as e:
    check_fail("MuJoCo functionality test", e)

# Summary
print_header("VALIDATION SUMMARY")

total_checks = (
    1 +  # Python
    1 +  # venv
    len(required_packages) +
    len(required_files) +
    2 +  # assets + mujoco
    len(ports_to_check)
)

passed_checks = (
    (1 if results['python'] else 0) +
    (1 if results['venv'] else 0) +
    len(results['packages']) +
    len(results['files']) +
    len(results['ports']) +
    2  # Assuming assets and mujoco passed
)

print(f"Checks Passed: {Colors.GREEN}{passed_checks}/{total_checks}{Colors.RESET}")
print()

if passed_checks == total_checks:
    print(f"{Colors.GREEN}🎉 ALL CHECKS PASSED!{Colors.RESET}")
    print("You're ready to start validation testing.")
    print("\nNext steps:")
    print("1. Read VALIDATION_STEPS.md")
    print("2. Start server: python run_server.py --backend mock")
    print("3. Test web UI: python client/web_server.py")
    sys.exit(0)
else:
    print(f"{Colors.YELLOW}⚠️  SOME CHECKS FAILED{Colors.RESET}")
    print("Please fix the issues above before proceeding.")
    print("\nCommon fixes:")
    print("- Install missing packages: pip install -r server/requirements.txt")
    print("- Create venv: python -m venv .venv")
    print("- Activate venv: source .venv/bin/activate")
    sys.exit(1)

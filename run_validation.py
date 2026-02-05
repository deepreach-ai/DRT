#!/usr/bin/env python3
"""
Simple test runner for teleop system validation
No changes to code - just runs validation tests
"""

import subprocess
import sys
import time
from pathlib import Path

# ANSI colors
GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
BOLD = '\033[1m'
RESET = '\033[0m'

def print_section(title):
    print(f"\n{BOLD}{BLUE}{'='*70}{RESET}")
    print(f"{BOLD}{BLUE}{title:^70}{RESET}")
    print(f"{BOLD}{BLUE}{'='*70}{RESET}\n")

def print_step(step_num, title):
    print(f"\n{BOLD}Step {step_num}: {title}{RESET}")
    print("-" * 70)

def print_success(msg):
    print(f"{GREEN}✅ {msg}{RESET}")

def print_error(msg):
    print(f"{RED}❌ {msg}{RESET}")

def print_warning(msg):
    print(f"{YELLOW}⚠️  {msg}{RESET}")

def print_info(msg):
    print(f"{BLUE}ℹ️  {msg}{RESET}")

def wait_for_user(msg="Press Enter to continue..."):
    input(f"\n{YELLOW}{msg}{RESET}")

def check_command_exists(cmd):
    """Check if a command exists"""
    try:
        subprocess.run([cmd, "--version"], capture_output=True, check=True)
        return True
    except:
        return False

def main():
    print_section("TELEOP SYSTEM VALIDATION - NO CHANGES MODE")
    
    teleop_dir = Path("/Users/ziguo/teleop_system")
    
    if not teleop_dir.exists():
        print_error(f"Teleop directory not found: {teleop_dir}")
        sys.exit(1)
    
    print_info(f"Working directory: {teleop_dir}")
    
    # Step 1: Prerequisites
    print_step(1, "Check Prerequisites")
    print("Running automated prerequisite checks...")
    
    prereq_script = teleop_dir / "validate_prereqs.py"
    if prereq_script.exists():
        result = subprocess.run([sys.executable, str(prereq_script)])
        if result.returncode != 0:
            print_error("Prerequisites check failed")
            print_warning("Fix issues above before continuing")
            sys.exit(1)
        print_success("Prerequisites check passed!")
    else:
        print_warning("validate_prereqs.py not found, skipping...")
    
    wait_for_user()
    
    # Step 2: What to validate
    print_step(2, "Choose Validation Mode")
    print("\nWhat would you like to validate?")
    print(f"{BOLD}1.{RESET} Mock Backend Only (quickest)")
    print(f"{BOLD}2.{RESET} MuJoCo Backend Only")
    print(f"{BOLD}3.{RESET} Both Backends")
    print(f"{BOLD}4.{RESET} Full System (Backends + Web UI)")
    
    choice = input(f"\n{YELLOW}Enter choice [1-4]:{RESET} ").strip()
    
    # Validation instructions based on choice
    if choice == "1":
        print_section("MOCK BACKEND VALIDATION")
        print_info("This will test the server with a simulated robot (no MuJoCo needed)")
        print("\nIn your terminal, run:")
        print(f"{BOLD}cd /Users/ziguo/teleop_system{RESET}")
        print(f"{BOLD}python run_server.py --backend mock{RESET}")
        print("\nExpected output:")
        print("  - Server starts on http://0.0.0.0:8000")
        print("  - No errors in terminal")
        print("  - Backend status: 'connected'")
        
        print("\nTo test:")
        print("  1. Open new terminal")
        print("  2. Run: curl http://localhost:8000/health")
        print("  3. Run: curl http://localhost:8000/api/v1/statistics")
        
        wait_for_user("Press Enter after you've tested the server...")
        
    elif choice == "2":
        print_section("MUJOCO BACKEND VALIDATION")
        print_info("This will test the server with MuJoCo physics simulation")
        print("\nIn your terminal, run:")
        print(f"{BOLD}cd /Users/ziguo/teleop_system{RESET}")
        print(f"{BOLD}python run_server.py --backend mujoco \\{RESET}")
        print(f"{BOLD}  --mujoco-xml robots/so101.xml \\{RESET}")
        print(f"{BOLD}  --mujoco-ee gripperframe{RESET}")
        
        print("\nExpected output:")
        print("  - Server starts on http://0.0.0.0:8000")
        print("  - MuJoCo model loads successfully")
        print("  - May see 'Rendering failed' (NORMAL on macOS)")
        print("  - Backend status: 'connected'")
        
        print("\nTo test:")
        print("  1. Open new terminal")
        print("  2. Run: curl http://localhost:8000/api/v1/statistics")
        print("  3. Check 'current_position' is not null")
        print("  4. Robot should be at [x, y, z] position")
        
        wait_for_user("Press Enter after you've tested the server...")
        
    elif choice == "3":
        print_section("BOTH BACKENDS VALIDATION")
        print_info("Test both Mock and MuJoCo backends")
        
        print("\n📝 Test 1: Mock Backend")
        print(f"{BOLD}python run_server.py --backend mock{RESET}")
        print("→ Verify server starts and responds to API calls")
        print("→ Press Ctrl+C to stop")
        
        wait_for_user("Press Enter when Mock test is done...")
        
        print("\n📝 Test 2: MuJoCo Backend")
        print(f"{BOLD}python run_server.py --backend mujoco --mujoco-xml robots/so101.xml --mujoco-ee gripperframe{RESET}")
        print("→ Verify server starts and loads robot model")
        print("→ Check API returns robot position")
        print("→ Press Ctrl+C to stop")
        
        wait_for_user("Press Enter when MuJoCo test is done...")
        
    elif choice == "4":
        print_section("FULL SYSTEM VALIDATION")
        print_info("Test backends + web UI + client")
        
        print("\n📝 Step 1: Start Teleop Server")
        print("Terminal 1:")
        print(f"{BOLD}python run_server.py --backend mock{RESET}")
        
        wait_for_user("Start server, then press Enter...")
        
        print("\n📝 Step 2: Start Web UI Server")
        print("Terminal 2:")
        print(f"{BOLD}python client/web_server.py{RESET}")
        
        wait_for_user("Start web server, then press Enter...")
        
        print("\n📝 Step 3: Test in Browser")
        print("Open browser to: http://localhost:8080")
        print("\nTest checklist:")
        print("  [ ] Page loads without errors")
        print("  [ ] Login form appears")
        print("  [ ] Login with operator/operator works")
        print("  [ ] WebSocket connects (check browser console)")
        print("  [ ] Press '1' to activate safety")
        print("  [ ] Press W/A/S/D keys to move")
        print("  [ ] Statistics update in real-time")
        
        wait_for_user("Test browser UI, then press Enter...")
        
        print("\n📝 Step 4: Test Keyboard Client (optional)")
        print("Terminal 3:")
        print(f"{BOLD}python client/keyboard_client.py{RESET}")
        print("  [ ] Client connects to server")
        print("  [ ] Keyboard controls work")
        print("  [ ] Position updates shown")
        
        wait_for_user("Done testing? Press Enter...")
    
    else:
        print_error("Invalid choice")
        sys.exit(1)
    
    # Summary
    print_section("VALIDATION COMPLETE")
    print("Please confirm the following:")
    
    questions = [
        "Did the server start without errors?",
        "Did API endpoints return valid JSON?",
        "Were you able to send commands successfully?",
        "Did the system run for at least 5 minutes without crashing?",
    ]
    
    if choice in ["3", "4"]:
        questions.extend([
            "Did the web UI load and connect?",
            "Did keyboard/mouse controls work?",
        ])
    
    passed = 0
    for i, question in enumerate(questions, 1):
        answer = input(f"\n{i}. {question} [y/n]: ").strip().lower()
        if answer == 'y':
            passed += 1
            print_success("Passed")
        else:
            print_error("Failed")
    
    print("\n" + "="*70)
    print(f"Results: {passed}/{len(questions)} tests passed")
    
    if passed == len(questions):
        print_success("🎉 ALL VALIDATION TESTS PASSED!")
        print("\nYour system is working correctly!")
        print("\nNext steps:")
        print("  1. Document any observations")
        print("  2. Measure latency if needed")
        print("  3. Prepare demo for stakeholders")
    else:
        print_warning("⚠️  SOME TESTS FAILED")
        print("\nPlease review:")
        print("  1. Server terminal logs")
        print("  2. Browser console (F12)")
        print("  3. Error messages above")
        print("\nRefer to VALIDATION_STEPS.md for troubleshooting")
    
    print("\n" + "="*70)
    print("Validation session complete!")
    print(f"Documentation: {teleop_dir}/VALIDATION_STEPS.md")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n\n{YELLOW}Validation interrupted by user{RESET}")
        sys.exit(0)

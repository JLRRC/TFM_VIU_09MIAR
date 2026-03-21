#!/usr/bin/env python3
"""
Complete PICK workflow validation script.
Tests: select_object → pick_object → state transitions → button availability
"""
import sys
import time
import subprocess

# Add panel module to path
sys.path.insert(0, "./install/ur5_qt_panel/lib/python3.12/site-packages")

from ur5_qt_panel.panel_objects import (
    get_object_state,
    has_any_carried_objects,
    are_all_objects_released,
    ObjectLogicalState,
)

def print_section(title):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print('='*60)

def check_object_state(name):
    """Get and print current state of object."""
    state = get_object_state(name)
    return state.logical_state.value if state else "NO_STATE"

def call_service(service_name, service_type, request_json):
    """Call ROS 2 service and return result."""
    try:
        cmd = ["ros2", "service", "call", service_name, service_type, request_json, "--cli-args", "-w", "0.1"]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        return result.returncode == 0, result.stdout
    except Exception as e:
        return False, str(e)

def test_pick_workflow():
    """Execute complete PICK workflow test."""
    
    print_section("TEST 1: Initial State Check")
    obj_name = "pick_demo"
    state_initial = check_object_state(obj_name)
    carried_initial = has_any_carried_objects()
    print(f"Object '{obj_name}' state: {state_initial}")
    print(f"has_any_carried_objects(): {carried_initial}")
    print(f"✓ Button should be: DISABLED (no carried objects)")
    assert not carried_initial, "Expected no carried objects at startup"
    
    print_section("TEST 2: Select Object")
    select_ok, select_out = call_service(
        "/panel/select_object",
        "ur5_panel_interfaces/srv/SelectObject",
        "{objname: 'pick_demo'}"
    )
    print(f"select_object call: {'SUCCESS' if select_ok else 'FAILED'}")
    if not select_ok:
        print(f"Output: {select_out[:200]}")
    
    time.sleep(2)
    
    print_section("TEST 3: Pre-PICK State")
    state_pre = check_object_state(obj_name)
    print(f"Object '{obj_name}' state: {state_pre}")
    assert state_pre in ["ON_TABLE", "SELECTED"], f"Expected ON_TABLE or SELECTED, got {state_pre}"
    print(f"✓ Object ready for PICK")
    
    print_section("TEST 4: Trigger PICK Object")
    pick_ok, pick_out = call_service(
        "/panel/pick_object",
        "std_srvs/srv/Trigger",
        "{}"
    )
    print(f"pick_object call: {'SUCCESS' if pick_ok else 'FAILED'}")
    if not pick_ok:
        print(f"Output: {pick_out[:200]}")
    
    # Wait for PICK sequence to complete (approach + grasp + carry)
    print("\nWaiting for PICK sequence to complete...")
    time.sleep(15)
    
    print_section("TEST 5: Post-PICK State")
    state_post = check_object_state(obj_name)
    carried_post = has_any_carried_objects()
    print(f"Object '{obj_name}' state: {state_post}")
    print(f"has_any_carried_objects(): {carried_post}")
    print(f"✓ Button should be: {'ENABLED' if carried_post else 'DISABLED'}")
    
    if state_post == "CARRIED":
        print(f"✓ SUCCESS: Object is CARRIED, button logic working correctly")
        return True
    else:
        print(f"✗ PARTIAL: Object state is {state_post}, not CARRIED yet")
        if carried_post:
            print(f"  (but has_any_carried_objects() = True, so button would be enabled)")
        return False

if __name__ == "__main__":
    try:
        success = test_pick_workflow()
        print_section("FINAL RESULT")
        if success:
            print("✓ Complete PICK workflow test PASSED")
            sys.exit(0)
        else:
            print("✗ Complete PICK workflow test INCOMPLETE (check logs)")
            sys.exit(1)
    except Exception as e:
        print_section("TEST ERROR")
        print(f"✗ Exception: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(2)

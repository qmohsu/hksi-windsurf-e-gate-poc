#!/usr/bin/env python3
"""
Platform verification and diagnostic script.

Checks:
- Platform detection
- Native library availability
- Python fallback functionality
- Basic trilateration accuracy

Run this script on target Raspberry Pi to verify deployment readiness.
"""

import sys
import os
import platform
from typing import Tuple

# Fix Windows console encoding for Unicode characters
if sys.platform == 'win32':
    import locale
    sys.stdout.reconfigure(encoding='utf-8')
    sys.stderr.reconfigure(encoding='utf-8')

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from trilateration_wrapper import (
    TrilaterationWrapper,
    PythonTrilateration,
    Vec3D,
    create_trilateration_wrapper,
)


def print_header(title: str):
    """Print formatted section header."""
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def print_status(check: str, passed: bool, details: str = ""):
    """Print check status with formatting."""
    status = "✓ PASS" if passed else "✗ FAIL"
    color = "\033[92m" if passed else "\033[91m"
    reset = "\033[0m"
    
    print(f"{color}{status}{reset} {check}")
    if details:
        print(f"       {details}")


def calculate_distance_3d(p1: Tuple[float, float, float], 
                         p2: Tuple[float, float, float]) -> float:
    """Calculate 3D Euclidean distance."""
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)**0.5


def verify_platform_detection():
    """Verify platform detection."""
    print_header("Platform Detection")
    
    wrapper = TrilaterationWrapper()
    info = wrapper.platform_info
    
    print(f"System:       {info['system']}")
    print(f"Machine:      {info['machine']}")
    print(f"Architecture: {info['arch']}")
    print(f"Is Linux:     {info['is_linux']}")
    print(f"Is Windows:   {info['is_windows']}")
    print(f"Is RPi:       {info['is_rpi']}")
    
    # Check consistency
    is_consistent = (
        (info['is_linux'] and info['system'] == 'Linux') or
        (info['is_windows'] and info['system'] == 'Windows') or
        (not info['is_linux'] and not info['is_windows'])
    )
    
    print_status("Platform detection", is_consistent, 
                 f"Detected {info['system']} {info['arch']}")
    
    return is_consistent


def verify_native_library():
    """Verify native library loading."""
    print_header("Native Library Loading")
    
    wrapper = TrilaterationWrapper()
    default_path = wrapper._get_default_library_path()
    
    print(f"Default library path: {default_path}")
    
    if default_path and os.path.exists(default_path):
        print(f"Library file exists: YES")
        print(f"File size: {os.path.getsize(default_path)} bytes")
    else:
        print(f"Library file exists: NO")
    
    result = wrapper.load_dll()
    
    print_status("Native library loading", result, 
                 "Library loaded successfully" if result else 
                 "Library not available (will use Python fallback)")
    
    return result


def verify_python_fallback():
    """Verify Python fallback implementation."""
    print_header("Python Fallback Implementation")
    
    # Test anchors (equilateral triangle)
    anchors = [
        (0.0, 0.0, 2.0),
        (10.0, 0.0, 2.0),
        (5.0, 8.66, 2.0),
    ]
    
    # Known tag position
    tag_pos = (5.0, 4.0, 0.5)
    
    # Calculate distances
    distances = [calculate_distance_3d(tag_pos, a) for a in anchors]
    
    print(f"Test anchors: {len(anchors)}")
    print(f"True tag position: ({tag_pos[0]:.2f}, {tag_pos[1]:.2f}, {tag_pos[2]:.2f})")
    print(f"Distances: {[f'{d:.3f}m' for d in distances]}")
    
    # Test Python trilateration
    result = PythonTrilateration.trilaterate_3d(anchors, distances)
    
    if result:
        error_xy = ((result[0] - tag_pos[0])**2 + (result[1] - tag_pos[1])**2)**0.5
        error_z = abs(result[2] - tag_pos[2])
        
        print(f"Computed position: ({result[0]:.2f}, {result[1]:.2f}, {result[2]:.2f})")
        print(f"XY error: {error_xy:.3f}m")
        print(f"Z error:  {error_z:.3f}m")
        
        passed = error_xy < 0.5  # 50cm tolerance
        print_status("Python fallback accuracy", passed,
                     f"XY error {error_xy:.3f}m " + 
                     ("within tolerance" if passed else "exceeds 0.5m limit"))
        
        return passed
    else:
        print_status("Python fallback", False, "Trilateration failed")
        return False


def verify_wrapper_integration():
    """Verify integrated wrapper (native or fallback)."""
    print_header("Integrated Wrapper Test")
    
    wrapper = create_trilateration_wrapper()
    
    # Test anchors
    anchors = [
        (0.0, 0.0, 2.0),
        (10.0, 0.0, 2.0),
        (5.0, 8.66, 2.0),
    ]
    
    tag_pos = (5.0, 4.0, 0.5)
    distances_m = [calculate_distance_3d(tag_pos, a) for a in anchors]
    distances_mm = [int(d * 1000) for d in distances_m] + [-1] * 5  # Pad to 8
    
    wrapper.set_anchor_positions(anchors)
    wrapper.set_distances(distances_mm)
    
    # Try native first
    result = wrapper.calculate_location()
    used_native = result is not None
    
    # Fallback to Python if native failed
    if result is None:
        print("Native calculation unavailable, using Python fallback")
        result = PythonTrilateration.trilaterate_3d(anchors, distances_m)
        if result:
            result = Vec3D(x=result[0], y=result[1], z=result[2])
    
    if result:
        error_xy = ((result.x - tag_pos[0])**2 + (result.y - tag_pos[1])**2)**0.5
        
        method = "Native library" if used_native else "Python fallback"
        print(f"Method used: {method}")
        print(f"Computed position: ({result.x:.2f}, {result.y:.2f}, {result.z:.2f})")
        print(f"XY error: {error_xy:.3f}m")
        
        passed = error_xy < 1.0
        print_status("Integrated wrapper", passed,
                     f"XY error {error_xy:.3f}m " + 
                     ("acceptable" if passed else "too large"))
        
        return passed
    else:
        print_status("Integrated wrapper", False, "All methods failed")
        return False


def main():
    """Run all verification checks."""
    print("\n" + "=" * 70)
    print("  HKSI UWB Platform Verification")
    print("  Target: Ubuntu on Raspberry Pi")
    print("=" * 70)
    
    print(f"\nPython version: {sys.version}")
    print(f"Platform: {platform.platform()}")
    
    results = {
        'platform': verify_platform_detection(),
        'native_lib': verify_native_library(),
        'fallback': verify_python_fallback(),
        'integration': verify_wrapper_integration(),
    }
    
    # Summary
    print_header("Verification Summary")
    
    total = len(results)
    passed = sum(results.values())
    
    for check, result in results.items():
        print_status(check.replace('_', ' ').title(), result)
    
    print(f"\nResult: {passed}/{total} checks passed")
    
    if passed == total:
        print("\n✓ System ready for deployment")
        return 0
    elif results['fallback']:
        print("\n⚠ Native library unavailable but Python fallback working")
        print("  For production, build native library:")
        print("    ./scripts/build_trilateration.sh arm64")
        return 1
    else:
        print("\n✗ Critical failures detected")
        return 2


if __name__ == "__main__":
    sys.exit(main())

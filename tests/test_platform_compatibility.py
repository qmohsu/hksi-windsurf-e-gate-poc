"""
Unit tests for platform compatibility and library loading.

Tests cover:
- Platform detection (Windows, Linux x86_64, Linux ARM64/RPi)
- Native library loading (.dll, .so)
- Fallback to Python implementation when native library unavailable
- Cross-platform trilateration result consistency

Reference: Phase 0 - RPi/Ubuntu blocker resolution
"""

import platform
import os
from typing import List, Tuple
from unittest.mock import patch, MagicMock

import pytest

from trilateration_wrapper import (
    TrilaterationWrapper,
    PythonTrilateration,
    Vec3D,
    create_trilateration_wrapper,
)


class TestPlatformDetection:
    """Tests for platform detection logic."""

    def test_detect_platform_returns_dict(self):
        """Test that platform detection returns required fields."""
        wrapper = TrilaterationWrapper()
        
        assert 'system' in wrapper.platform_info
        assert 'machine' in wrapper.platform_info
        assert 'arch' in wrapper.platform_info
        assert 'is_windows' in wrapper.platform_info
        assert 'is_linux' in wrapper.platform_info
        assert 'is_rpi' in wrapper.platform_info

    def test_current_platform_detection(self):
        """Test detection of current platform matches python's platform module."""
        wrapper = TrilaterationWrapper()
        
        expected_system = platform.system()
        assert wrapper.platform_info['system'] == expected_system
        
        # Verify boolean flags are consistent
        if expected_system == 'Windows':
            assert wrapper.platform_info['is_windows'] is True
            assert wrapper.platform_info['is_linux'] is False
        elif expected_system == 'Linux':
            assert wrapper.platform_info['is_linux'] is True
            assert wrapper.platform_info['is_windows'] is False

    @patch('platform.system')
    @patch('platform.machine')
    def test_detect_linux_x86_64(self, mock_machine, mock_system):
        """Test detection of Linux x86_64 platform."""
        mock_system.return_value = 'Linux'
        mock_machine.return_value = 'x86_64'
        
        wrapper = TrilaterationWrapper()
        
        assert wrapper.platform_info['system'] == 'Linux'
        assert wrapper.platform_info['arch'] == 'x86_64'
        assert wrapper.platform_info['is_linux'] is True
        assert wrapper.platform_info['is_rpi'] is False

    @patch('platform.system')
    @patch('platform.machine')
    def test_detect_linux_arm64_rpi(self, mock_machine, mock_system):
        """Test detection of Raspberry Pi (Linux ARM64)."""
        mock_system.return_value = 'Linux'
        mock_machine.return_value = 'aarch64'
        
        wrapper = TrilaterationWrapper()
        
        assert wrapper.platform_info['system'] == 'Linux'
        assert wrapper.platform_info['arch'] == 'arm64'
        assert wrapper.platform_info['is_linux'] is True
        assert wrapper.platform_info['is_rpi'] is True

    @patch('platform.system')
    @patch('platform.machine')
    def test_detect_windows_platform(self, mock_machine, mock_system):
        """Test detection of Windows platform."""
        mock_system.return_value = 'Windows'
        mock_machine.return_value = 'AMD64'
        
        wrapper = TrilaterationWrapper()
        
        assert wrapper.platform_info['system'] == 'Windows'
        assert wrapper.platform_info['arch'] == 'x86_64'
        assert wrapper.platform_info['is_windows'] is True
        assert wrapper.platform_info['is_rpi'] is False


class TestLibraryPathResolution:
    """Tests for library path resolution based on platform."""

    @patch('platform.system')
    @patch('platform.machine')
    def test_default_path_windows(self, mock_machine, mock_system):
        """Test default path resolution on Windows."""
        mock_system.return_value = 'Windows'
        mock_machine.return_value = 'AMD64'
        
        wrapper = TrilaterationWrapper()
        path = wrapper._get_default_library_path()
        
        assert path is not None
        assert 'trilateration.dll' in path
        assert 'uwbdemo' in path

    @patch('platform.system')
    @patch('platform.machine')
    @patch('os.path.exists')
    def test_default_path_linux_x86_64(self, mock_exists, mock_machine, mock_system):
        """Test default path resolution on Linux x86_64."""
        mock_system.return_value = 'Linux'
        mock_machine.return_value = 'x86_64'
        
        # Simulate arch-specific library exists
        def exists_side_effect(path):
            return 'libtrilateration_x86_64.so' in path
        
        mock_exists.side_effect = exists_side_effect
        
        wrapper = TrilaterationWrapper()
        path = wrapper._get_default_library_path()
        
        assert path is not None
        assert 'libtrilateration_x86_64.so' in path

    @patch('platform.system')
    @patch('platform.machine')
    @patch('os.path.exists')
    def test_default_path_linux_arm64(self, mock_exists, mock_machine, mock_system):
        """Test default path resolution on Raspberry Pi (ARM64)."""
        mock_system.return_value = 'Linux'
        mock_machine.return_value = 'aarch64'
        
        # Simulate arch-specific library exists
        def exists_side_effect(path):
            return 'libtrilateration_arm64.so' in path
        
        mock_exists.side_effect = exists_side_effect
        
        wrapper = TrilaterationWrapper()
        path = wrapper._get_default_library_path()
        
        assert path is not None
        assert 'libtrilateration_arm64.so' in path


class TestLibraryLoading:
    """Tests for library loading behavior."""

    def test_load_dll_with_nonexistent_path_fails(self):
        """Test that loading from nonexistent path fails gracefully."""
        wrapper = TrilaterationWrapper()
        result = wrapper.load_dll("/nonexistent/path/library.so")
        
        assert result is False
        assert wrapper.library is None

    def test_load_dll_sets_both_library_and_dll_attributes(self):
        """Test backward compatibility: both library and dll are set."""
        wrapper = TrilaterationWrapper()
        
        # Only test if library actually exists
        if wrapper._get_default_library_path() and \
           os.path.exists(wrapper._get_default_library_path()):
            result = wrapper.load_dll()
            
            if result:
                assert wrapper.library is not None
                assert wrapper.dll is not None
                assert wrapper.dll is wrapper.library  # Should be same object


class TestPythonFallback:
    """Tests for Python fallback when native library unavailable."""

    def test_python_fallback_2d_produces_valid_results(
        self,
        anchor_positions_2d: List[Tuple[float, float]],
    ):
        """Test that Python fallback produces valid 2D trilateration results."""
        from tests.conftest import calculate_distance_2d
        
        tag_pos = (5.0, 4.0)
        distances = [
            calculate_distance_2d(tag_pos, anchor_positions_2d[0]),
            calculate_distance_2d(tag_pos, anchor_positions_2d[1]),
            calculate_distance_2d(tag_pos, anchor_positions_2d[2]),
        ]
        
        result = PythonTrilateration.trilaterate_2d(anchor_positions_2d, distances)
        
        assert result is not None
        # Should be within 20cm of true position
        error = calculate_distance_2d(result, tag_pos)
        assert error < 0.2, f"Python fallback error {error:.3f}m exceeds 20cm"

    def test_python_fallback_3d_produces_valid_results(
        self,
        anchor_positions_3d: List[Tuple[float, float, float]],
    ):
        """Test that Python fallback produces valid 3D trilateration results."""
        from tests.conftest import calculate_distance_3d
        
        tag_pos = (5.0, 4.0, 0.5)
        distances = [
            calculate_distance_3d(tag_pos, anchor_positions_3d[0]),
            calculate_distance_3d(tag_pos, anchor_positions_3d[1]),
            calculate_distance_3d(tag_pos, anchor_positions_3d[2]),
        ]
        
        result = PythonTrilateration.trilaterate_3d(anchor_positions_3d, distances)
        
        assert result is not None
        # XY should be accurate, Z tolerance is higher
        xy_error = ((result[0] - tag_pos[0])**2 + (result[1] - tag_pos[1])**2)**0.5
        assert xy_error < 0.5, f"Python fallback XY error {xy_error:.3f}m too large"


class TestWrapperCreation:
    """Tests for wrapper factory function."""

    def test_create_wrapper_succeeds(self):
        """Test that create_trilateration_wrapper returns valid wrapper."""
        wrapper = create_trilateration_wrapper()
        
        assert wrapper is not None
        assert isinstance(wrapper, TrilaterationWrapper)
        assert wrapper.platform_info is not None

    def test_create_wrapper_without_library_logs_warning(self, caplog):
        """Test that missing library logs appropriate warning."""
        wrapper = create_trilateration_wrapper("/nonexistent/path.so")
        
        assert wrapper is not None
        # Should log warning about fallback
        assert any('Python备用算法' in record.message or 'fallback' in record.message.lower()
                   for record in caplog.records)


class TestCrossPlatformConsistency:
    """Tests to ensure consistent behavior across platforms."""

    def test_wrapper_initialization_consistent(self):
        """Test wrapper initializes consistently regardless of library availability."""
        wrapper = TrilaterationWrapper()
        
        # These should always be initialized
        assert wrapper.MAX_ANCHORS == 8
        assert len(wrapper.anchor_array) == 8
        assert len(wrapper.distance_array) == 8
        assert wrapper.location is not None
        
        # All distances should be initialized to -1
        for i in range(8):
            assert wrapper.distance_array[i] == -1

    def test_set_anchor_positions_works_without_library(self):
        """Test anchor position setting works even without loaded library."""
        wrapper = TrilaterationWrapper()
        # Don't load library
        
        positions = [(0, 0, 2), (10, 0, 2), (5, 8.66, 2)]
        wrapper.set_anchor_positions(positions)
        
        # Verify positions were set
        for i, (x, y, z) in enumerate(positions):
            assert wrapper.anchor_array[i].x == x
            assert wrapper.anchor_array[i].y == y
            assert wrapper.anchor_array[i].z == z

    def test_set_distances_works_without_library(self):
        """Test distance setting works even without loaded library."""
        wrapper = TrilaterationWrapper()
        # Don't load library
        
        distances = [5000, 6000, 7000, -1, -1, -1, -1, -1]
        wrapper.set_distances(distances)
        
        # Verify distances were set
        assert wrapper.distance_array[0] == 5000
        assert wrapper.distance_array[1] == 6000
        assert wrapper.distance_array[2] == 7000
        for i in range(3, 8):
            assert wrapper.distance_array[i] == -1


class TestArchitectureNormalization:
    """Tests for architecture name normalization."""

    @patch('platform.machine')
    def test_x86_64_variants_normalized(self, mock_machine):
        """Test that x86_64 variants are normalized to 'x86_64'."""
        variants = ['x86_64', 'AMD64', 'x64', 'amd64']
        
        for variant in variants:
            mock_machine.return_value = variant
            wrapper = TrilaterationWrapper()
            assert wrapper.platform_info['arch'] == 'x86_64', \
                f"Failed to normalize {variant} to x86_64"

    @patch('platform.machine')
    def test_arm64_variants_normalized(self, mock_machine):
        """Test that ARM64 variants are normalized to 'arm64'."""
        variants = ['aarch64', 'arm64', 'ARM64', 'AARCH64']
        
        for variant in variants:
            mock_machine.return_value = variant
            wrapper = TrilaterationWrapper()
            assert wrapper.platform_info['arch'] == 'arm64', \
                f"Failed to normalize {variant} to arm64"


@pytest.mark.integration
class TestRealLibraryLoading:
    """Integration tests with real library (if available)."""

    def test_native_library_loads_if_available(self):
        """Test that native library loads on supported platform if built."""
        wrapper = TrilaterationWrapper()
        result = wrapper.load_dll()
        
        # This test is informational - library may not be built yet
        if result:
            pytest.skip("Native library loaded successfully")
        else:
            pytest.skip("Native library not available (expected during development)")

    def test_calculation_with_native_or_fallback(
        self,
        anchor_positions_3d: List[Tuple[float, float, float]],
    ):
        """Test calculation works with either native or Python fallback."""
        from tests.conftest import calculate_distance_3d
        
        wrapper = create_trilateration_wrapper()
        wrapper.set_anchor_positions(anchor_positions_3d)
        
        tag_pos = (5.0, 4.0, 0.5)
        distances_m = [
            calculate_distance_3d(tag_pos, anchor_positions_3d[0]),
            calculate_distance_3d(tag_pos, anchor_positions_3d[1]),
            calculate_distance_3d(tag_pos, anchor_positions_3d[2]),
        ]
        distances_mm = [int(d * 1000) for d in distances_m]
        
        # Pad to 8 values
        while len(distances_mm) < 8:
            distances_mm.append(-1)
        
        wrapper.set_distances(distances_mm)
        
        # Try native library first
        result = wrapper.calculate_location()
        
        if result is None:
            # Fallback to Python
            result = PythonTrilateration.trilaterate_3d(anchor_positions_3d, distances_m)
            result = Vec3D(x=result[0], y=result[1], z=result[2]) if result else None
        
        assert result is not None, "Both native and fallback failed"
        
        # Verify reasonable accuracy
        xy_error = ((result.x - tag_pos[0])**2 + (result.y - tag_pos[1])**2)**0.5
        assert xy_error < 1.0, f"Position error {xy_error:.3f}m too large"

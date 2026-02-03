"""
Unit tests for coordinate conversion module.

Tests cover:
- GNSS to ENU conversion (gnss_to_enu)
- ENU to plane coordinate conversion (enu_to_plane)
- Plane to GNSS conversion (plane_to_gnss)
- Round-trip conversion accuracy
- Anchor coordinate management
- Virtual anchor coordinate generation

Reference geometry: WGS84 ellipsoid with Hong Kong area coordinates.
"""

import math
from typing import Dict

import pytest

from coordinate_converter import (
    CoordinateConverter,
    GNSSCoordinate,
    PlaneCoordinate,
    create_virtual_anchor_coordinates,
)


class TestGNSSCoordinate:
    """Tests for GNSSCoordinate dataclass."""

    def test_gnss_coordinate_creation(self):
        """Test GNSSCoordinate creation with all fields."""
        coord = GNSSCoordinate(
            lat=22.2900,
            lon=114.1700,
            alt=2.0,
            e=5.0,
            n=10.0,
            u=2.0,
        )

        assert coord.lat == 22.2900
        assert coord.lon == 114.1700
        assert coord.alt == 2.0
        assert coord.e == 5.0
        assert coord.n == 10.0
        assert coord.u == 2.0

    def test_gnss_coordinate_default_enu(self):
        """Test GNSSCoordinate with default ENU values."""
        coord = GNSSCoordinate(lat=22.29, lon=114.17, alt=2.0)

        assert coord.e == 0.0
        assert coord.n == 0.0
        assert coord.u == 0.0


class TestPlaneCoordinate:
    """Tests for PlaneCoordinate dataclass."""

    def test_plane_coordinate_creation(self):
        """Test PlaneCoordinate creation."""
        coord = PlaneCoordinate(x=5.0, y=10.0, z=2.0)

        assert coord.x == 5.0
        assert coord.y == 10.0
        assert coord.z == 2.0

    def test_plane_coordinate_equality(self):
        """Test PlaneCoordinate equality."""
        coord1 = PlaneCoordinate(x=5.0, y=10.0, z=2.0)
        coord2 = PlaneCoordinate(x=5.0, y=10.0, z=2.0)
        coord3 = PlaneCoordinate(x=5.0, y=10.0, z=3.0)

        assert coord1 == coord2
        assert coord1 != coord3


class TestCoordinateConverterInitialization:
    """Tests for CoordinateConverter initialization."""

    def test_converter_initial_state(self):
        """Test that converter initializes with empty state."""
        converter = CoordinateConverter()

        assert converter.origin is None
        assert len(converter.anchor_gnss) == 0
        assert len(converter.anchor_plane) == 0

    def test_set_origin(self, origin_gnss_coordinate: GNSSCoordinate):
        """Test setting origin coordinate."""
        converter = CoordinateConverter()
        converter.set_origin(origin_gnss_coordinate)

        assert converter.origin is not None
        assert converter.origin.lat == origin_gnss_coordinate.lat
        assert converter.origin.lon == origin_gnss_coordinate.lon
        assert converter.origin.alt == origin_gnss_coordinate.alt


class TestGNSSToENUConversion:
    """Tests for GNSS to ENU coordinate conversion."""

    def test_gnss_to_enu_same_point(self, origin_gnss_coordinate: GNSSCoordinate):
        """
        Test that converting origin to ENU gives (0, 0, 0).

        The origin point should have zero ENU offset from itself.
        """
        converter = CoordinateConverter()
        converter.set_origin(origin_gnss_coordinate)

        e, n, u = converter.gnss_to_enu(origin_gnss_coordinate)

        assert abs(e) < 1e-6, f"E should be ~0, got {e}"
        assert abs(n) < 1e-6, f"N should be ~0, got {n}"
        assert abs(u) < 1e-6, f"U should be ~0, got {u}"

    def test_gnss_to_enu_10m_north(self, origin_gnss_coordinate: GNSSCoordinate):
        """
        Test ENU conversion for point ~10m north.

        1 degree of latitude ≈ 111km, so 10m ≈ 0.00009 degrees.
        """
        converter = CoordinateConverter()
        converter.set_origin(origin_gnss_coordinate)

        # Point approximately 10m north
        lat_offset = 10.0 / 111000.0  # ~10m in degrees
        north_point = GNSSCoordinate(
            lat=origin_gnss_coordinate.lat + lat_offset,
            lon=origin_gnss_coordinate.lon,
            alt=origin_gnss_coordinate.alt,
        )

        e, n, u = converter.gnss_to_enu(north_point)

        assert abs(e) < 0.1, f"E should be ~0, got {e}"
        assert abs(n - 10.0) < 0.5, f"N should be ~10m, got {n}"
        assert abs(u) < 0.1, f"U should be ~0, got {u}"

    def test_gnss_to_enu_10m_east(self, origin_gnss_coordinate: GNSSCoordinate):
        """
        Test ENU conversion for point ~10m east.

        Longitude to meters depends on latitude (cosine factor).
        """
        converter = CoordinateConverter()
        converter.set_origin(origin_gnss_coordinate)

        # Point approximately 10m east
        cos_lat = math.cos(math.radians(origin_gnss_coordinate.lat))
        lon_offset = 10.0 / (111000.0 * cos_lat)

        east_point = GNSSCoordinate(
            lat=origin_gnss_coordinate.lat,
            lon=origin_gnss_coordinate.lon + lon_offset,
            alt=origin_gnss_coordinate.alt,
        )

        e, n, u = converter.gnss_to_enu(east_point)

        assert abs(e - 10.0) < 0.5, f"E should be ~10m, got {e}"
        assert abs(n) < 0.1, f"N should be ~0, got {n}"
        assert abs(u) < 0.1, f"U should be ~0, got {u}"

    def test_gnss_to_enu_altitude_change(self, origin_gnss_coordinate: GNSSCoordinate):
        """Test ENU conversion for altitude change only."""
        converter = CoordinateConverter()
        converter.set_origin(origin_gnss_coordinate)

        elevated_point = GNSSCoordinate(
            lat=origin_gnss_coordinate.lat,
            lon=origin_gnss_coordinate.lon,
            alt=origin_gnss_coordinate.alt + 5.0,  # 5m higher
        )

        e, n, u = converter.gnss_to_enu(elevated_point)

        assert abs(e) < 0.1, f"E should be ~0, got {e}"
        assert abs(n) < 0.1, f"N should be ~0, got {n}"
        assert abs(u - 5.0) < 0.1, f"U should be ~5m, got {u}"

    def test_gnss_to_enu_without_origin_raises(self):
        """Test that gnss_to_enu raises error without origin."""
        converter = CoordinateConverter()
        point = GNSSCoordinate(lat=22.29, lon=114.17, alt=2.0)

        with pytest.raises(ValueError, match="未设置原点"):
            converter.gnss_to_enu(point)


class TestENUToPlaneConversion:
    """Tests for ENU to plane coordinate conversion."""

    def test_enu_to_plane_identity(self):
        """Test that ENU to plane is identity transform (E=X, N=Y, U=Z)."""
        converter = CoordinateConverter()

        result = converter.enu_to_plane(5.0, 10.0, 2.0)

        assert result.x == 5.0, "X should equal E"
        assert result.y == 10.0, "Y should equal N"
        assert result.z == 2.0, "Z should equal U"

    def test_enu_to_plane_negative_values(self):
        """Test ENU to plane with negative values."""
        converter = CoordinateConverter()

        result = converter.enu_to_plane(-5.0, -10.0, -2.0)

        assert result.x == -5.0
        assert result.y == -10.0
        assert result.z == -2.0


class TestPlaneToGNSSConversion:
    """Tests for plane to GNSS coordinate conversion."""

    def test_plane_to_gnss_origin(self, origin_gnss_coordinate: GNSSCoordinate):
        """Test that plane origin (0,0,0) converts back to GNSS origin."""
        converter = CoordinateConverter()
        converter.set_origin(origin_gnss_coordinate)

        origin_plane = PlaneCoordinate(x=0.0, y=0.0, z=0.0)
        result = converter.plane_to_gnss(origin_plane)

        assert abs(result.lat - origin_gnss_coordinate.lat) < 1e-6
        assert abs(result.lon - origin_gnss_coordinate.lon) < 1e-6
        # Altitude adds z to origin.alt
        assert abs(result.alt - origin_gnss_coordinate.alt) < 0.1

    def test_plane_to_gnss_10m_offset(self, origin_gnss_coordinate: GNSSCoordinate):
        """Test plane to GNSS for 10m east, 10m north offset."""
        converter = CoordinateConverter()
        converter.set_origin(origin_gnss_coordinate)

        offset_plane = PlaneCoordinate(x=10.0, y=10.0, z=0.0)
        result = converter.plane_to_gnss(offset_plane)

        # Latitude should increase (north)
        assert result.lat > origin_gnss_coordinate.lat
        # Longitude should increase (east)
        assert result.lon > origin_gnss_coordinate.lon

    def test_plane_to_gnss_without_origin_raises(self):
        """Test that plane_to_gnss raises error without origin."""
        converter = CoordinateConverter()
        plane = PlaneCoordinate(x=5.0, y=10.0, z=2.0)

        with pytest.raises(ValueError, match="未设置原点"):
            converter.plane_to_gnss(plane)


class TestRoundtripConversion:
    """Tests for roundtrip coordinate conversion accuracy."""

    def test_gnss_to_plane_to_gnss_roundtrip(
        self, origin_gnss_coordinate: GNSSCoordinate
    ):
        """
        Test GNSS → ENU → Plane → GNSS roundtrip conversion.

        Starting from a GNSS point, convert to ENU, then plane,
        then back to GNSS. Result should match original.
        """
        converter = CoordinateConverter()
        converter.set_origin(origin_gnss_coordinate)

        # Create a test point offset from origin
        lat_offset = 15.0 / 111000.0  # ~15m north
        cos_lat = math.cos(math.radians(origin_gnss_coordinate.lat))
        lon_offset = 20.0 / (111000.0 * cos_lat)  # ~20m east

        original = GNSSCoordinate(
            lat=origin_gnss_coordinate.lat + lat_offset,
            lon=origin_gnss_coordinate.lon + lon_offset,
            alt=origin_gnss_coordinate.alt + 3.0,
        )

        # Convert GNSS → ENU → Plane
        e, n, u = converter.gnss_to_enu(original)
        plane = converter.enu_to_plane(e, n, u)

        # Convert Plane → GNSS
        result = converter.plane_to_gnss(plane)

        # Check roundtrip accuracy (should be < 1cm for small distances)
        lat_error_m = abs(result.lat - original.lat) * 111000.0
        lon_error_m = abs(result.lon - original.lon) * 111000.0 * cos_lat

        assert lat_error_m < 0.01, f"Latitude error {lat_error_m:.4f}m exceeds 1cm"
        assert lon_error_m < 0.01, f"Longitude error {lon_error_m:.4f}m exceeds 1cm"
        assert abs(result.alt - original.alt) < 0.01, "Altitude error exceeds 1cm"

    def test_plane_to_gnss_to_plane_roundtrip(
        self, origin_gnss_coordinate: GNSSCoordinate
    ):
        """
        Test Plane → GNSS → ENU → Plane roundtrip conversion.
        """
        converter = CoordinateConverter()
        converter.set_origin(origin_gnss_coordinate)

        original_plane = PlaneCoordinate(x=25.0, y=30.0, z=1.5)

        # Convert Plane → GNSS
        gnss = converter.plane_to_gnss(original_plane)

        # Convert GNSS → ENU → Plane
        e, n, u = converter.gnss_to_enu(gnss)
        result_plane = converter.enu_to_plane(e, n, u)

        assert abs(result_plane.x - original_plane.x) < 0.01, "X roundtrip error > 1cm"
        assert abs(result_plane.y - original_plane.y) < 0.01, "Y roundtrip error > 1cm"
        assert abs(result_plane.z - original_plane.z) < 0.01, "Z roundtrip error > 1cm"


class TestAnchorManagement:
    """Tests for anchor coordinate management."""

    def test_update_anchor_gnss(self, origin_gnss_coordinate: GNSSCoordinate):
        """Test updating a single anchor's GNSS coordinates."""
        converter = CoordinateConverter()

        # Update A0 (should become origin)
        converter.update_anchor_gnss(
            "A0",
            {
                "lat": origin_gnss_coordinate.lat,
                "lon": origin_gnss_coordinate.lon,
                "alt": origin_gnss_coordinate.alt,
                "e": 0.0,
                "n": 0.0,
                "u": 2.0,
            },
        )

        assert "A0" in converter.anchor_gnss
        assert converter.origin is not None
        assert converter.origin.lat == origin_gnss_coordinate.lat

    def test_update_all_anchors(self, virtual_gnss_config: Dict):
        """Test batch updating all anchors."""
        converter = CoordinateConverter()

        # Create virtual anchors
        virtual_anchors = create_virtual_anchor_coordinates(virtual_gnss_config)

        # Convert to expected format
        gnss_dict = {}
        for anchor_id, coord in virtual_anchors.items():
            gnss_dict[anchor_id] = {
                "lat": coord.lat,
                "lon": coord.lon,
                "alt": coord.alt,
                "e": coord.e,
                "n": coord.n,
                "u": coord.u,
            }

        converter.update_all_anchors(gnss_dict)

        assert "A0" in converter.anchor_gnss
        assert "A1" in converter.anchor_gnss
        assert "A2" in converter.anchor_gnss
        assert converter.origin is not None

    def test_anchor_plane_coordinates_computed(
        self, coordinate_converter: CoordinateConverter
    ):
        """Test that plane coordinates are computed for all anchors."""
        # The fixture already has anchors configured
        plane_coords = coordinate_converter.get_anchor_plane_coordinates()

        assert "A0" in plane_coords
        assert "A1" in plane_coords
        assert "A2" in plane_coords

        # A0 should be at origin (0, 0, z)
        assert plane_coords["A0"].x == 0.0
        assert plane_coords["A0"].y == 0.0

    def test_anchor_a0_is_origin(self, coordinate_converter: CoordinateConverter):
        """Test that A0 is correctly set as origin (0, 0)."""
        plane_coords = coordinate_converter.get_anchor_plane_coordinates()

        assert plane_coords["A0"].x == 0.0, "A0.x should be 0"
        assert plane_coords["A0"].y == 0.0, "A0.y should be 0"

    def test_anchor_a1_position(self, coordinate_converter: CoordinateConverter):
        """Test A1 position relative to A0."""
        plane_coords = coordinate_converter.get_anchor_plane_coordinates()

        # A1 should be at (10, 0, 2) based on virtual config
        assert abs(plane_coords["A1"].x - 10.0) < 0.5, "A1.x should be ~10m"
        assert abs(plane_coords["A1"].y - 0.0) < 0.5, "A1.y should be ~0m"

    def test_anchor_a2_position(self, coordinate_converter: CoordinateConverter):
        """Test A2 position relative to A0."""
        plane_coords = coordinate_converter.get_anchor_plane_coordinates()

        # A2 should be at (5, 8.66, 2) based on virtual config
        assert abs(plane_coords["A2"].x - 5.0) < 0.5, "A2.x should be ~5m"
        assert abs(plane_coords["A2"].y - 8.66) < 0.5, "A2.y should be ~8.66m"

    def test_get_anchor_plane_coordinates_returns_copy(
        self, coordinate_converter: CoordinateConverter
    ):
        """Test that get_anchor_plane_coordinates returns a copy."""
        coords1 = coordinate_converter.get_anchor_plane_coordinates()
        coords2 = coordinate_converter.get_anchor_plane_coordinates()

        # Modify one copy
        coords1["A0"] = PlaneCoordinate(x=999.0, y=999.0, z=999.0)

        # Original should be unchanged
        assert coords2["A0"].x == 0.0


class TestVirtualAnchorCoordinates:
    """Tests for virtual anchor coordinate generation."""

    def test_create_virtual_anchor_coordinates(self, virtual_gnss_config: Dict):
        """Test virtual anchor coordinate generation."""
        anchors = create_virtual_anchor_coordinates(virtual_gnss_config)

        assert "A0" in anchors
        assert "A1" in anchors
        assert "A2" in anchors

    def test_virtual_anchors_have_correct_enu(self, virtual_gnss_config: Dict):
        """Test that virtual anchors preserve ENU coordinates."""
        anchors = create_virtual_anchor_coordinates(virtual_gnss_config)

        # A0 should be at (0, 0, 2)
        assert anchors["A0"].e == 0.0
        assert anchors["A0"].n == 0.0
        assert anchors["A0"].u == 2.0

        # A1 should be at (10, 0, 2)
        assert anchors["A1"].e == 10.0
        assert anchors["A1"].n == 0.0
        assert anchors["A1"].u == 2.0

        # A2 should be at (5, 8.66, 2)
        assert anchors["A2"].e == 5.0
        assert anchors["A2"].n == 8.66
        assert anchors["A2"].u == 2.0

    def test_virtual_anchors_gnss_calculated(self, virtual_gnss_config: Dict):
        """Test that GNSS coordinates are calculated from ENU."""
        anchors = create_virtual_anchor_coordinates(virtual_gnss_config)

        base_lat = virtual_gnss_config["base_lat"]
        base_lon = virtual_gnss_config["base_lon"]

        # A0 should be at base coordinates
        assert abs(anchors["A0"].lat - base_lat) < 0.001
        assert abs(anchors["A0"].lon - base_lon) < 0.001

        # A1 should be east of base (higher longitude)
        assert anchors["A1"].lon > base_lon

        # A2 should be northeast of base
        assert anchors["A2"].lat > base_lat
        assert anchors["A2"].lon > base_lon


class TestWGS84Parameters:
    """Tests for WGS84 ellipsoid parameters."""

    def test_wgs84_constants(self):
        """Test that WGS84 constants are correct."""
        assert abs(CoordinateConverter.WGS84_A - 6378137.0) < 1.0
        assert abs(CoordinateConverter.WGS84_F - (1.0 / 298.257223563)) < 1e-12

    def test_wgs84_derived_constants(self):
        """Test derived WGS84 constants."""
        a = CoordinateConverter.WGS84_A
        f = CoordinateConverter.WGS84_F
        b = CoordinateConverter.WGS84_B
        e2 = CoordinateConverter.WGS84_E2

        # b = a * (1 - f)
        expected_b = a * (1 - f)
        assert abs(b - expected_b) < 1.0

        # e2 = 2f - f^2
        expected_e2 = 2 * f - f ** 2
        assert abs(e2 - expected_e2) < 1e-12


class TestCoordinateConverterEdgeCases:
    """Tests for edge cases in coordinate conversion."""

    def test_conversion_at_equator(self):
        """Test coordinate conversion near the equator."""
        converter = CoordinateConverter()

        equator_origin = GNSSCoordinate(lat=0.0, lon=0.0, alt=0.0)
        converter.set_origin(equator_origin)

        # 10m north at equator
        north_point = GNSSCoordinate(lat=10.0 / 111000.0, lon=0.0, alt=0.0)
        e, n, u = converter.gnss_to_enu(north_point)

        assert abs(n - 10.0) < 0.5, f"Expected ~10m north, got {n}m"

    def test_conversion_at_high_latitude(self):
        """Test coordinate conversion at high latitude (60°N)."""
        converter = CoordinateConverter()

        high_lat_origin = GNSSCoordinate(lat=60.0, lon=0.0, alt=0.0)
        converter.set_origin(high_lat_origin)

        # 10m east at 60°N (longitude degree is shorter)
        cos_lat = math.cos(math.radians(60.0))
        lon_offset = 10.0 / (111000.0 * cos_lat)
        east_point = GNSSCoordinate(lat=60.0, lon=lon_offset, alt=0.0)

        e, n, u = converter.gnss_to_enu(east_point)

        assert abs(e - 10.0) < 1.0, f"Expected ~10m east, got {e}m"

    def test_conversion_with_large_offset(
        self, origin_gnss_coordinate: GNSSCoordinate
    ):
        """
        Test conversion accuracy with larger offset (100m).

        Linear approximation should still be reasonable for 100m.
        """
        converter = CoordinateConverter()
        converter.set_origin(origin_gnss_coordinate)

        # 100m north, 100m east
        lat_offset = 100.0 / 111000.0
        cos_lat = math.cos(math.radians(origin_gnss_coordinate.lat))
        lon_offset = 100.0 / (111000.0 * cos_lat)

        far_point = GNSSCoordinate(
            lat=origin_gnss_coordinate.lat + lat_offset,
            lon=origin_gnss_coordinate.lon + lon_offset,
            alt=origin_gnss_coordinate.alt,
        )

        e, n, u = converter.gnss_to_enu(far_point)

        # At 100m, error should still be < 1m
        assert abs(e - 100.0) < 1.0, f"E error > 1m: {e}"
        assert abs(n - 100.0) < 1.0, f"N error > 1m: {n}"

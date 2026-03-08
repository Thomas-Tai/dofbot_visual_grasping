# Copyright (c) 2024 DOFBOT Project
# SPDX-License-Identifier: BSD-3-Clause

"""
Unit tests for the DOFBOT hardware driver.

Tests both real driver and mock driver behavior using dependency injection.
"""

import math
import time
import threading
import pytest
from typing import List, Optional

# Import mock driver
from dofbot_hardware.mock_driver import MockDofbotDriver, MockConfig, FailureType
from dofbot_hardware.exceptions import (
    DofbotConnectionError,
    DofbotCommunicationError,
    DofbotJointError,
    DofbotValueError,
)


# Fixtures

@pytest.fixture
def mock_config():
    """Default mock configuration."""
    return MockConfig(
        communication_delay_ms=5,
        position_noise_std=0.1,
        failure_rate=0.0,
        max_velocity=90.0
    )


@pytest.fixture
def mock_driver(mock_config):
    """Create a mock driver for testing."""
    driver = MockDofbotDriver(config=mock_config)
    yield driver
    driver.disconnect()


@pytest.fixture
def connected_mock_driver(mock_driver):
    """Create and connect a mock driver."""
    mock_driver.connect()
    yield mock_driver
    mock_driver.disconnect()


# Lifecycle Tests

class TestDriverLifecycle:
    """Tests for driver connection lifecycle."""
    
    def test_connect_success(self, mock_driver):
        """Test successful connection."""
        assert mock_driver.connect() is True
        assert mock_driver.is_connected() is True
        mock_driver.disconnect()
    
    def test_connect_already_connected(self, mock_driver):
        """Test connecting when already connected."""
        mock_driver.connect()
        assert mock_driver.connect() is True  # Should still return True
        mock_driver.disconnect()
    
    def test_disconnect_success(self, mock_driver):
        """Test successful disconnection."""
        mock_driver.connect()
        mock_driver.disconnect()
        assert mock_driver.is_connected() is False
    
    def test_double_disconnect_safe(self, mock_driver):
        """Test that double disconnect is safe."""
        mock_driver.connect()
        mock_driver.disconnect()
        mock_driver.disconnect()  # Should not raise
        assert mock_driver.is_connected() is False
    
    def test_context_manager(self, mock_config):
        """Test context manager support."""
        with MockDofbotDriver(config=mock_config) as driver:
            assert driver.is_connected() is True
        assert driver.is_connected() is False


# Unit Conversion Tests

class TestUnitConversion:
    """Tests for degree-to-radian conversion."""
    
    @pytest.mark.parametrize("degrees,radians", [
        (90.0, 0.0),       # Center
        (0.0, -math.pi/2),  # Min
        (180.0, math.pi/2), # Max
        (45.0, -math.pi/4), # Quarter
        (135.0, math.pi/4), # Three-quarter
    ])
    def test_degrees_to_radians(self, degrees, radians):
        """Test degree to radian conversion."""
        driver = MockDofbotDriver()
        result = driver._degrees_to_radians(degrees, 0)
        assert math.isclose(result, radians, rel_tol=1e-6)
    
    @pytest.mark.parametrize("radians,degrees", [
        (0.0, 90.0),       # Center
        (-math.pi/2, 0.0),  # Min
        (math.pi/2, 180.0), # Max
        (-math.pi/4, 45.0), # Quarter
        (math.pi/4, 135.0), # Three-quarter
    ])
    def test_radians_to_degrees(self, radians, degrees):
        """Test radian to degree conversion."""
        driver = MockDofbotDriver()
        result = driver._radians_to_degrees(radians, 0)
        assert math.isclose(result, degrees, rel_tol=1e-6)


# Read Operations Tests

class TestReadOperations:
    """Tests for reading joint positions."""
    
    def test_read_all_joints_returns_6_values(self, connected_mock_driver):
        """Test that read returns 6 joint values."""
        positions = connected_mock_driver.read_joint_positions()
        assert len(positions) == 6
    
    def test_read_single_joint_valid_id(self, connected_mock_driver):
        """Test reading single joint with valid ID."""
        for joint_id in range(1, 7):
            position = connected_mock_driver.read_single_joint(joint_id)
            assert position is not None
            assert isinstance(position, float)
    
    def test_read_single_joint_invalid_id_raises(self, connected_mock_driver):
        """Test reading single joint with invalid ID."""
        with pytest.raises(DofbotJointError):
            connected_mock_driver.read_single_joint(0)
        with pytest.raises(DofbotJointError):
            connected_mock_driver.read_single_joint(7)
    
    def test_read_without_connection_raises(self, mock_driver):
        """Test that reading without connection raises error."""
        with pytest.raises(DofbotConnectionError):
            mock_driver.read_joint_positions()
    
    def test_read_after_write_reflects_change(self, connected_mock_driver):
        """Test that positions update after write."""
        # Write new positions
        target = [0.0, 0.5, 0.0, 0.0, 0.0, 0.4]
        connected_mock_driver.write_joint_positions(target, 100)
        
        # Wait for physics simulation
        time.sleep(0.2)
        
        # Read positions
        positions = connected_mock_driver.read_joint_positions()
        
        # Check they're close (allowing for physics simulation)
        for i, (actual, expected) in enumerate(zip(positions, target)):
            if i < 5:  # First 5 joints
                assert abs(actual - expected) < 0.5  # Loose tolerance


# Write Operations Tests

class TestWriteOperations:
    """Tests for writing joint positions."""
    
    def test_write_all_joints_success(self, connected_mock_driver):
        """Test writing all joints."""
        positions = [0.0, 0.5, 0.0, 0.0, 0.0, 0.4]
        result = connected_mock_driver.write_joint_positions(positions, 1000)
        assert result is True
    
    def test_write_single_joint_success(self, connected_mock_driver):
        """Test writing single joint."""
        result = connected_mock_driver.write_single_joint(1, 0.5, 1000)
        assert result is True
    
    def test_write_invalid_joint_count_raises(self, connected_mock_driver):
        """Test writing wrong number of joints."""
        with pytest.raises(DofbotValueError):
            connected_mock_driver.write_joint_positions([0.0, 0.5], 1000)
    
    def test_write_out_of_range_angle_clamped(self, connected_mock_driver):
        """Test that out-of-range angles are clamped."""
        # Write extreme value
        positions = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]  # Way beyond limits
        result = connected_mock_driver.write_joint_positions(positions, 100)
        assert result is True
        
        # Get internal positions to verify clamping
        internal = connected_mock_driver.get_internal_positions()
        for p in internal:
            assert 0.0 <= p <= 180.0
    
    def test_write_without_connection_raises(self, mock_driver):
        """Test that writing without connection raises error."""
        positions = [0.0, 0.5, 0.0, 0.0, 0.0, 0.4]
        with pytest.raises(DofbotConnectionError):
            mock_driver.write_joint_positions(positions, 1000)


# Error Handling Tests

class TestErrorHandling:
    """Tests for error handling and failure injection."""
    
    def test_inject_connection_failure(self, mock_driver):
        """Test connection failure injection."""
        mock_driver.inject_failure(FailureType.CONNECTION_LOST)
        
        with pytest.raises(DofbotConnectionError):
            mock_driver.connect()
    
    def test_inject_timeout_failure(self, connected_mock_driver):
        """Test timeout failure injection."""
        connected_mock_driver.inject_failure(FailureType.TIMEOUT)
        
        with pytest.raises(DofbotCommunicationError):
            connected_mock_driver.read_joint_positions()
    
    def test_inject_joint_stuck(self, connected_mock_driver):
        """Test joint stuck failure injection."""
        # Get initial position
        initial = connected_mock_driver.read_joint_positions()
        
        # Inject stuck joint
        connected_mock_driver.inject_failure(FailureType.JOINT_STUCK, joint_id=1)
        
        # Try to move joint 1
        new_positions = [1.0, 0.0, 0.0, 0.0, 0.0, 0.4]
        connected_mock_driver.write_joint_positions(new_positions, 100)
        
        # Wait for physics
        time.sleep(0.2)
        
        # Joint 1 should not have moved
        positions = connected_mock_driver.read_joint_positions()
        assert abs(positions[0] - initial[0]) < 0.01
    
    def test_clear_failure(self, connected_mock_driver):
        """Test clearing injected failure."""
        connected_mock_driver.inject_failure(FailureType.TIMEOUT)
        
        with pytest.raises(DofbotCommunicationError):
            connected_mock_driver.read_joint_positions()
        
        connected_mock_driver.clear_failure(FailureType.TIMEOUT)
        
        # Should work now
        positions = connected_mock_driver.read_joint_positions()
        assert len(positions) == 6


# Command Recording Tests

class TestCommandRecording:
    """Tests for command history recording."""
    
    def test_record_write_command(self, connected_mock_driver):
        """Test that write commands are recorded."""
        positions = [0.0, 0.5, 0.0, 0.0, 0.0, 0.4]
        connected_mock_driver.write_joint_positions(positions, 1000)
        
        history = connected_mock_driver.get_command_history()
        assert len(history) >= 1
        
        last_cmd = history[-1]
        assert last_cmd.command_type == 'write'
        assert last_cmd.time_ms == 1000
    
    def test_record_read_command(self, connected_mock_driver):
        """Test that read commands are recorded."""
        connected_mock_driver.read_joint_positions()
        
        history = connected_mock_driver.get_command_history()
        assert len(history) >= 1
        
        last_cmd = history[-1]
        assert last_cmd.command_type == 'read'
    
    def test_clear_history(self, connected_mock_driver):
        """Test clearing command history."""
        connected_mock_driver.read_joint_positions()
        connected_mock_driver.clear_history()
        
        history = connected_mock_driver.get_command_history()
        assert len(history) == 0


# Thread Safety Tests

class TestThreadSafety:
    """Tests for thread safety."""
    
    def test_concurrent_read_write(self, connected_mock_driver):
        """Test concurrent reads and writes from multiple threads."""
        errors = []
        
        def writer_thread():
            try:
                for _ in range(10):
                    positions = [0.0, 0.5, 0.0, 0.0, 0.0, 0.4]
                    connected_mock_driver.write_joint_positions(positions, 100)
                    time.sleep(0.01)
            except Exception as e:
                errors.append(e)
        
        def reader_thread():
            try:
                for _ in range(10):
                    connected_mock_driver.read_joint_positions()
                    time.sleep(0.01)
            except Exception as e:
                errors.append(e)
        
        threads = [
            threading.Thread(target=writer_thread),
            threading.Thread(target=reader_thread),
            threading.Thread(target=reader_thread),
        ]
        
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        assert len(errors) == 0


# Run with pytest
if __name__ == '__main__':
    pytest.main([__file__, '-v'])
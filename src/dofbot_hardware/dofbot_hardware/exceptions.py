# Copyright (c) 2024 DOFBOT Project
# SPDX-License-Identifier: BSD-3-Clause

"""
Custom exceptions for the DOFBOT hardware driver.

This module defines a hierarchy of exceptions for handling various
hardware communication and connection errors.
"""


class DofbotError(Exception):
    """Base exception for all DOFBOT hardware errors."""

    def __init__(self, message: str = "DOFBOT hardware error"):
        self.message = message
        super().__init__(self.message)


class DofbotConnectionError(DofbotError):
    """Raised when connection to the DOFBOT hardware fails."""

    def __init__(self, message: str = "Failed to connect to DOFBOT hardware"):
        super().__init__(message)


class DofbotCommunicationError(DofbotError):
    """Raised when communication with the DOFBOT hardware fails."""

    def __init__(self, message: str = "Communication error with DOFBOT hardware"):
        super().__init__(message)


class DofbotTimeoutError(DofbotCommunicationError):
    """Raised when a hardware operation times out."""

    def __init__(self, message: str = "Operation timed out"):
        super().__init__(message)


class DofbotJointError(DofbotError):
    """Raised when there's an issue with a specific joint."""

    def __init__(self, joint_id: int, message: str = "Joint error"):
        self.joint_id = joint_id
        super().__init__(f"Joint {joint_id}: {message}")


class DofbotValueError(DofbotError):
    """Raised when an invalid value is provided to the hardware."""

    def __init__(self, message: str = "Invalid value provided"):
        super().__init__(message)
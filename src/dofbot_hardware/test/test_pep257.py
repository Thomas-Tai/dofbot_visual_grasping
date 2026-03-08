# Copyright (c) 2024 DOFBOT Project
# SPDX-License-Identifier: BSD-3-Clause

"""Test that the package follows PEP 257 docstring conventions."""

import pytest
from ament_pep257 import main


def test_pep257():
    """Test PEP 257 compliance."""
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
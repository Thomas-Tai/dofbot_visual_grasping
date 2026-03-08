# Copyright (c) 2024 DOFBOT Project
# SPDX-License-Identifier: BSD-3-Clause

"""Test that the package has proper copyright notices."""

import pytest
from ament_copyright import main


def test_copyright():
    """Test copyright notice compliance."""
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found copyright errors'
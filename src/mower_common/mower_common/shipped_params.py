# SPDX-License-Identifier: GPL-3.0-or-later
"""The shipped parameter yaml as the one source of truth (review 2026-11-02 H5).

Tests load ``src/<package>/config/<file>.yaml`` (the file launch passes on the mower) with
:func:`shipped_params` and compare it with the code defaults with :func:`drift`, so a
default that silently differs from the field value fails CI instead of making the unit
tests exercise a configuration the mower never runs.
"""

import math
import os

_SRC = os.path.realpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))


def shipped_yaml_path(package, filename):
    """Source tree first (tests run from the checkout), then the installed share dir."""
    path = os.path.join(_SRC, package, 'config', filename)
    if os.path.isfile(path):
        return path
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(package), 'config', filename)


def shipped_params(package, filename, node):
    """``ros__parameters`` of ``node`` in the package's shipped yaml (``{}`` if absent)."""
    import yaml
    with open(shipped_yaml_path(package, filename)) as fh:
        doc = yaml.safe_load(fh) or {}
    return dict((doc.get(node) or {}).get('ros__parameters') or {})


def _same(a, b):
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b or a == b and type(a) is type(b)
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return math.isclose(float(a), float(b), rel_tol=1e-9, abs_tol=1e-12)
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b))
    return a == b


def drift(defaults, shipped, exempt=()):
    """``{key: (code_default, yaml_value)}`` for every yaml key with a different code default.
    Keys the code does not declare and ``exempt`` keys are skipped."""
    return {k: (defaults[k], v) for k, v in shipped.items()
            if k in defaults and k not in exempt and not _same(defaults[k], v)}

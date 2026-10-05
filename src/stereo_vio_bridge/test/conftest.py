"""Make the package importable when pytest is run without PYTHONPATH."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

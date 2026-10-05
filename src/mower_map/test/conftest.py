# Make `python3 -m pytest src/mower_map/test` work without a colcon install.
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

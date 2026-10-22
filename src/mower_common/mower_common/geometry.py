# SPDX-License-Identifier: GPL-3.0-or-later
"""Robot geometry constants (owner-measured 2026-10-20): the single source for code.

base_link = the rear drive-axle centre = the pivot centre. The YAML files keep their own
literal values (ROS parameters); test/test_yaml_constants.py asserts they equal these.
"""

# The MCU splits (v, w) into wheel speeds with this track; all wheel-speed math uses it.
TRACK_SPLIT_M = 0.40
# Tyre centre-to-centre (0.22 m tyres, 6 cm tread): yaw from measured wheel speeds (sim).
TRACK_PHYSICAL_M = 0.39

# Footprint around base_link: bumper ahead, rear edge behind, +- half width (0.47 m wide
# with the wheel trim).
FOOTPRINT_FRONT_M = 0.60
FOOTPRINT_REAR_M = 0.10
FOOTPRINT_HALF_WIDTH_M = 0.235
FOOTPRINT_WIDTH_M = 2.0 * FOOTPRINT_HALF_WIDTH_M
FOOTPRINT = ((FOOTPRINT_FRONT_M, FOOTPRINT_HALF_WIDTH_M),
             (FOOTPRINT_FRONT_M, -FOOTPRINT_HALF_WIDTH_M),
             (-FOOTPRINT_REAR_M, -FOOTPRINT_HALF_WIDTH_M),
             (-FOOTPRINT_REAR_M, FOOTPRINT_HALF_WIDTH_M))

# Standard blade disc (220 mm) and the deck (blade) centre ahead of base_link.
BLADE_DISC_M = 0.22
DECK_CENTRE_X_M = 0.23

# Docking ArUco marker side (vendor marker_size_ 0.04f; measured 0.055 pending, review 2.12).
MARKER_SIZE_M = 0.04

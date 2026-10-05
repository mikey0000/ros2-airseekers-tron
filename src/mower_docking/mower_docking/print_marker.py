# Copyright 2026 The mower_docking authors
# SPDX-License-Identifier: Apache-2.0
"""Write a printable ArUco marker PNG at a given physical size.

  ros2 run mower_docking print_marker --id 0 --size 0.04 --dpi 600 -o marker.png

The black square (excluding the white quiet zone) must measure exactly
``--size`` metres once printed; check with a ruler and set ``marker_size``
to what you measure.
"""
import argparse

from mower_docking import aruco_detect


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--dict', default='DICT_4X4_50')
    ap.add_argument('--id', type=int, default=0)
    ap.add_argument('--size', type=float, default=0.04, help='black square side in metres')
    ap.add_argument('--dpi', type=int, default=600)
    ap.add_argument('-o', '--output', default='aruco_marker.png')
    a = ap.parse_args(argv)
    if not aruco_detect.HAVE_CV2:
        raise SystemExit('cv2.aruco not available')
    cv2, np = aruco_detect.cv2, aruco_detect.np
    side_px = int(round(a.size / 0.0254 * a.dpi))
    img = aruco_detect.generate_marker_image(a.dict, a.id, side_px)
    q = max(side_px // 5, 10)  # quiet zone >= 1 module
    canvas = np.full((side_px + 2 * q, side_px + 2 * q), 255, np.uint8)
    canvas[q:q + side_px, q:q + side_px] = img
    cv2.imwrite(a.output, canvas)
    print('%s: %s id %d, %.1f mm black square = %d px @ %d dpi (print at 100%% scale)' % (
        a.output, a.dict, a.id, a.size * 1000, side_px, a.dpi))


if __name__ == '__main__':
    main()

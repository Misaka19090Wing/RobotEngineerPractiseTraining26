#!/usr/bin/env python3
"""
Generate a clean trinary occupancy map for the fixed course world.

The old PGM produced from noisy PCD projection marked the corridor center as
occupied, which made Nav2 fail to plan. This map uses the same course geometry
as synthetic_lidar.py so start and corridor cells are free.

Usage:
  ros2 run autoVehicle generate_course_map.py
  python3 scripts/generate_course_map.py --out robotProj_ws/autoVehicle/maps
"""
import argparse
import os


RES = 0.02
ORIGIN_X = -0.5
ORIGIN_Y = -2.0
MAX_X = 23.2
MAX_Y = 1.7

CORR_END = 22.58
JUNC_L = 22.48
JUNC_R = 22.98
JUNC_Y = 1.60


def is_free(x, y):
    if 0.0 <= x <= CORR_END and abs(y) <= 0.15:
        return True
    if JUNC_L <= x <= JUNC_R and abs(y) <= JUNC_Y:
        return True
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--out', default=os.path.expanduser('~/pointcloud_maps'))
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    width = int((MAX_X - ORIGIN_X) / RES) + 1
    height = int((MAX_Y - ORIGIN_Y) / RES) + 1
    pixels = []

    for row in range(height):
        y = ORIGIN_Y + (height - 1 - row) * RES
        for col in range(width):
            x = ORIGIN_X + col * RES
            pixels.append(254 if is_free(x, y) else 0)

    pgm_path = os.path.join(args.out, 'course_map.pgm')
    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n{width} {height}\n255\n'.encode())
        f.write(bytes(pixels))

    yaml_path = os.path.join(args.out, 'course_map.yaml')
    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write('image: course_map.pgm\n')
        f.write('mode: trinary\n')
        f.write(f'resolution: {RES}\n')
        f.write(f'origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]\n')
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.196\n')

    print(f'Wrote {pgm_path} ({width}x{height})')
    print(f'Wrote {yaml_path}')


if __name__ == '__main__':
    main()

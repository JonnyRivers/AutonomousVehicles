import controller2d_math
import numpy as np

heading_east = np.pi
heading_south_east = -3 * np.pi / 4
heading_south = -np.pi / 2
heading_south_west = -np.pi / 4
heading_west = 0
heading_north_west = np.pi / 4
heading_north = np.pi / 2
heading_north_east = 3 * np.pi / 4

half_root_two = 1 / np.sqrt(2)
expected_direction_east = [-1, 0]
expected_direction_south_east = [-half_root_two, -half_root_two]
expected_direction_south = [0, -1]
expected_direction_south_west = [half_root_two, -half_root_two]
expected_direction_west = [1, 0]
expected_direction_north_west = [half_root_two, half_root_two]
expected_direction_north = [0, 1]
expected_direction_north_east = [-half_root_two, half_root_two]

def verify_direction(value, expected):
    if(not np.isclose(value[0], expected[0])):
        raise Exception(f"direction.x value {value[0]} not close to expected value of {expected[0]}")
    if(not np.isclose(value[1], expected[1])):
        raise Exception(f"direction.x value {value[1]} not close to expected value of {expected[1]}")

direction_east = controller2d_math.heading_to_direction(heading_east)
direction_south_east = controller2d_math.heading_to_direction(heading_south_east)
direction_south = controller2d_math.heading_to_direction(heading_south)
direction_south_west = controller2d_math.heading_to_direction(heading_south_west)
direction_west = controller2d_math.heading_to_direction(heading_west)
direction_north_west = controller2d_math.heading_to_direction(heading_north_west)
direction_north = controller2d_math.heading_to_direction(heading_north)
direction_north_east = controller2d_math.heading_to_direction(heading_north_east)

verify_direction(direction_east, expected_direction_east)
verify_direction(direction_south_east, expected_direction_south_east)
verify_direction(direction_south, expected_direction_south)
verify_direction(direction_south_west, expected_direction_south_west)
verify_direction(direction_west, expected_direction_west)
verify_direction(direction_north_west, expected_direction_north_west)
verify_direction(direction_north, expected_direction_north)
verify_direction(direction_north_east, expected_direction_north_east)

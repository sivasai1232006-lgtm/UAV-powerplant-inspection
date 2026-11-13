import math
from uav_info import coordinates

gps_to_gazebo_enu = coordinates.gps_to_gazebo_enu
local_enu_to_map_rotation = coordinates.local_enu_to_map_rotation
local_origin_to_map = coordinates.local_origin_to_map
local_ned_to_map = coordinates.local_ned_to_map
map_to_local_ned = coordinates.map_to_local_ned
rotation_matrix_to_quaternion = coordinates.rotation_matrix_to_quaternion
local_enu_yaw_in_map = coordinates.local_enu_yaw_in_map

def ned_to_enu(vector):
    north, east, down = vector
    return east, north, -down

def enu_to_ned(vector):
    east, north, up = vector
    return north, east, -up

def frd_to_flu(vector):
    forward, right, down = vector
    return forward, -right, -down

def px4_attitude_to_enu_flu(quaternion):
    sqrt_half = math.sqrt(0.5)
    ned_to_enu_quaternion = (0.0, sqrt_half, sqrt_half, 0.0)
    flu_to_frd_quaternion = (0.0, 1.0, 0.0, 0.0)
    return multiply_quaternions(multiply_quaternions(ned_to_enu_quaternion, quaternion),flu_to_frd_quaternion,)

def rotate_vector(quaternion, vector):
    vector_quaternion = (0.0, *vector)
    inverse = (quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3])
    rotated = multiply_quaternions(multiply_quaternions(quaternion, vector_quaternion),inverse,)
    return rotated[1:]

def multiply_quaternions(left, right):
    left_w, left_x, left_y, left_z = left
    right_w, right_x, right_y, right_z = right
    return (
        left_w * right_w - left_x * right_x - left_y * right_y
        - left_z * right_z,
        left_w * right_x + left_x * right_w + left_y * right_z
        - left_z * right_y,
        left_w * right_y - left_x * right_z + left_y * right_w
        + left_z * right_x,
        left_w * right_z + left_x * right_y - left_y * right_x
        + left_z * right_w,
    )

#!/usr/bin/python3

import math
import time
##from dexapod import settings
import settings

def rotation_matrix(rotation_angle_xyz, position_xyz):
    rotation_angle_x = rotation_angle_xyz[0]
    rotation_angle_y = rotation_angle_xyz[1]
    rotation_angle_z = rotation_angle_xyz[2]
    x1 = position_xyz[0]
    y1 = position_xyz[1]
    z1 = position_xyz[2]
    cosa = math.cos(math.radians(rotation_angle_z))
    sina = math.sin(math.radians(rotation_angle_z))
    cosb = math.cos(math.radians(rotation_angle_y))
    sinb = math.sin(math.radians(rotation_angle_y))
    cosc = math.cos(math.radians(rotation_angle_x))
    sinc = math.sin(math.radians(rotation_angle_x))
    # Rotation Matrix
    Axx = cosa*cosb
    Axy = cosa*sinb*sinc - sina*cosc
    Axz = cosa*sinb*cosc + sina*sinc
    Ayx = sina*cosb
    Ayy = sina*sinb*sinc + cosa*cosc
    Ayz = sina*sinb*cosc - cosa*sinc
    Azx = -sinb
    Azy = cosb*sinc
    Azz = cosb*cosc
    # Run matrix
    x2 = Axx*x1 + Axy*y1 + Axz*z1
    y2 = Ayx*x1 + Ayy*y1 + Ayz*z1
    z2 = Azx*x1 + Azy*y1 + Azz*z1
    return([x2, y2, z2])

def find_closest_point_array(point_array, test_point):
    closest_distance = 999999
    closest_point = []
    array_counter = 0
    for point in point_array:
        delta_x = point[0] - test_point[0]
        delta_y = point[1] - test_point[1]
        delta_z = point[2] - test_point[2]
        arm_distance = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
        point_distance = abs(settings.second_arm_length - arm_distance)
        if point_distance < closest_distance:
            closest_distance = point_distance
            closest_point = [point[0], point[1], point[2], array_counter]
        array_counter += 1
    return(closest_point)

def create_arm_xyz_arrays(arm_rotation_data, radial_offset, xyz_array):
    arm_xyz_data = {}
    for arm_id in arm_rotation_data:
        file_name = 'arm_' + arm_id + '.xyz'
        f = open(file_name, 'w')
        f.write('')
        f.close()
        f = open(file_name, 'a')
        arm_xyz_array = []
        for xyz_position in xyz_array:
            rotated_xyz = rotation_matrix(arm_rotation_data[arm_id], xyz_position)
            arm_xyz_array.append(rotated_xyz)
            f.write(str(rotated_xyz[0]) + ' ' + str(rotated_xyz[1]) + ' ' + str(rotated_xyz[2]) + '\n')
        f.close()
        arm_xyz_data[arm_id] = arm_xyz_array
    return arm_xyz_data

def create_circle_xyz_array(arm_start_angle, arm_end_angle, angular_resolution, arm_radius, arm_radial_offset):
    xyz_array = []
    arm_angle = float(arm_start_angle)
    while arm_angle < arm_end_angle:
        arm_bearing_x = (math.cos(math.radians(arm_angle)) * arm_radius) + arm_radial_offset
        arm_bearing_z = math.sin(math.radians(arm_angle)) * arm_radius
        xyz_array.append([arm_bearing_x, 0, arm_bearing_z])
        arm_angle += angular_resolution
    return xyz_array

def end_xyz_rotated_translated(machine_position):
    rotation_angle_xyz = [machine_position['a'], machine_position['b'], machine_position['c']]
    end_xyz_rotated = {}
    for end_bearing_id in settings.end_bearing_offsets:
        end_xyz_rotated[end_bearing_id] = rotation_matrix(rotation_angle_xyz, settings.end_bearing_offsets[end_bearing_id])
    end_xyz_translated = {}
    for end_bearing_id in end_xyz_rotated:
        end_bearing_x_translated = end_xyz_rotated[end_bearing_id][0] + machine_position['x']
        end_bearing_y_translated = end_xyz_rotated[end_bearing_id][1] + machine_position['y']
        end_bearing_z_translated = end_xyz_rotated[end_bearing_id][2] + machine_position['z']
        end_xyz_translated[end_bearing_id] = [end_bearing_x_translated, end_bearing_y_translated, end_bearing_z_translated]
    return end_xyz_translated

def get_closest_arm_positions(machine_position):
    end_bearing_xyz_array = end_xyz_rotated_translated(machine_position)
    closest_arm_positions = {}
    for arm_id in end_bearing_xyz_array:
        closest_arm_positions[arm_id] = find_closest_point_array(arm_xyz_arrays[arm_id], end_bearing_xyz_array[arm_id])
    return closest_arm_positions

def get_arm_positions(machine_position, arm_angles, arm_velocities):
    end_bearing_xyz_array = end_xyz_rotated_translated(machine_position)
    closest_arm_positions = {}
    for arm_id in end_bearing_xyz_array:
        closest_arm_positions[arm_id] = find_closest_point_array(arm_xyz_arrays[arm_id], end_bearing_xyz_array[arm_id])
    return closest_arm_positions

def fixture_to_machine_coordinates(fixture):
    fixture_rotation = [fixture['offset_a'], fixture['offset_b'], fixture['offset_c']]
    fixture_position = [fixture['x'], fixture['y'], fixture['z']]
    # Find machine xyz position from fixture xyzabc offsets
    machine_rotated = rotation_matrix(fixture_rotation, fixture_position)
    machine_translated_x = machine_rotated[0] + fixture['offset_x']
    machine_translated_y = machine_rotated[1] + fixture['offset_y']
    machine_translated_z = machine_rotated[2] + fixture['offset_z']
    # Find machine abc rotation from fixture abc rotation
    machine_rotated_a = fixture['a'] + fixture['offset_a']
    machine_rotated_b = fixture['b'] + fixture['offset_b']
    machine_rotated_c = fixture['c'] + fixture['offset_c']
    return {'x': machine_translated_x,
            'y': machine_translated_y,
            'z': machine_translated_z,
            'a': machine_rotated_a,
            'b': machine_rotated_b,
            'c': machine_rotated_c}
    

    
    
        
circle_xyz_array = create_circle_xyz_array(settings.min_arm_angle,
                                           settings.max_arm_angle,
                                           settings.angular_resolution,
                                           settings.arm_radius,
                                           settings.arm_radial_offset)

arm_xyz_arrays = create_arm_xyz_arrays(settings.arm_rotations,
                                       settings.arm_radial_offset,
                                       circle_xyz_array)
 


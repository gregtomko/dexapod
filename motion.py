#!/usr/bin/python3

import re
import math

def parse_gcode_line(line, gcode_numbers):
    line_data = re.findall('[a-z][^a-z]*', line.lower())
    for i in line_data:
        gcode_numbers[i[0]] = float(i[1:])
    return gcode_numbers

def get_fixture_position(g_nums, fixture):
    f = None
    if g_nums['x'] != None:
        fixture['x'] = g_nums['x']
    if g_nums['y'] != None:
        fixture['y'] = g_nums['y']
    if g_nums['z'] != None:
        fixture['z'] = g_nums['z']
    if g_nums['a'] != None:
        fixture['a'] = g_nums['a']
    if g_nums['b'] != None:
        fixture['b'] = g_nums['b']
    if g_nums['c'] != None:
        fixture['c'] = g_nums['c']
    if g_nums['f'] != None:
        f = g_nums['f']
    return fixture, f
    
def get_position_delta (position_1, position_2):
    position_delta = {}
    for n in position_1:
        position_delta[n] = position_1[n] - position_2[n]
    position_delta['xyz'] = math.sqrt((position_delta['x']**2) + (position_delta['y']**2) + (position_delta['z']**2))
    position_delta['abc'] = math.sqrt((position_delta['a']**2) + (position_delta['b']**2) + (position_delta['c']**2))
    return position_delta
    
def get_velocity(pd, iv, mv):
    v_data = {}
    t = pd['xyz'] / mv
    v_data['time_xyz'] = t
    print('move time = ' + str(t) + ' secomds')
    v_data['x'] = pd['x'] / t
    print('move velocity x = ' + str(v_data['x']) + 'mm/sec')
    v_data['y'] = pd['y'] / t
    print('move velocity y = ' + str(v_data['y']) + 'mm/sec')
    v_data['z'] = pd['z'] / t
    print('move velocity z = ' + str(v_data['z']) + 'mm/sec')
    v_data['delta_x'] = v_data['x'] - iv['x']
    v_data['delta_y'] = v_data['y'] - iv['y']
    v_data['delta_z'] = v_data['z'] - iv['z']
    v_data['delta_xyz'] = math.sqrt((v_data['delta_x']**2) + (v_data['delta_y']**2) + (v_data['delta_z']**2))
    print('velocity_delta_xyz = ' + str(v_data['delta_xyz']) + 'mm/sec')
    return v_data
    
def get_acceleration(v_delta, iv, accel_xyz):
    a_data = {}
    a_data['accel_time_xyz'] = v_delta['delta_xyz'] / accel_xyz
    print('acceleration time xyz = ' + str(a_data['accel_time_xyz']) + 'sec')
    a_data['x'] = v_delta['delta_x'] / a_data['accel_time_xyz']
    print('acceleration x = ' + str(a_data['x']) + 'mm/sec^2')
    a_data['y'] = v_delta['delta_y'] / a_data['accel_time_xyz']
    print('acceleration y = ' + str(a_data['y']) + 'mm/sec^2')
    a_data['z'] = v_delta['delta_z'] / a_data['accel_time_xyz']
    print('acceleration z = ' + str(a_data['z']) + 'mm/sec^2')
    a_data['distance_x'] = (iv['x'] * a_data['accel_time_xyz']) + ((a_data['x'] * a_data['accel_time_xyz']**2) / 2)
    print('acceleration distance x = ' + str(a_data['distance_x']) + 'mm')
    a_data['distance_y'] = (iv['y'] * a_data['accel_time_xyz']) + ((a_data['y'] * a_data['accel_time_xyz']**2) / 2)
    print('acceleration distance y = ' + str(a_data['distance_y']) + 'mm')
    a_data['distance_z'] = (iv['z'] * a_data['accel_time_xyz']) + ((a_data['z'] * a_data['accel_time_xyz']**2) / 2)
    print('acceleration distance z = ' + str(a_data['distance_z']) + 'mm')




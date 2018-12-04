#!/usr/bin/python3

##from dexapod import kinematics
import kinematics

import math
import time
import redis

r = redis.Redis()

print('------ Dexapod Motion Control ------')
print('Reads and interprets .ngc gcode files')
print('Converts commands into machine motions')
print('')

gcode_filename = input('enter gcode file name: ')
if gcode_filename == '':
    gcode_filename = 'gcode_file.ngc'

# Coordinate Systems
#
# Home Positions
home_position = {'x': 0.0,
                 'y': 0.0,
                 'z': 500.0,
                 'a': 0.0,
                 'b': 0.0,
                 'c': 0.0}
# Machine Coordinates
machine_position = {'x': 0.0,
                    'y': 0.0,
                    'z': 500.0,
                    'a': 0.0,
                    'b': 0.0,
                    'c': 0.0}
# Fixture 1 offsets
fixture_offsets = {'1': {'id': '1',
                         'x': 0.0,
                         'y': 0.0,
                         'z': 0.0,
                         'a': 0.0,
                         'b': 0.0,
                         'c': 0.0,
                         'offset_x': 40.0,
                         'offset_y': 40.0,
                         'offset_z': 500.0,
                         'offset_a': 0.0,
                         'offset_b': 0.0,
                         'offset_c': 0.0},
                   '2': {'id': '1',
                         'x': 0.0,
                         'y': 0.0,
                         'z': 0.0,
                         'a': 0.0,
                         'b': 0.0,
                         'c': 0.0,
                         'offset_x': 0.0,
                         'offset_y': 0.0,
                         'offset_z': 0.0,
                         'offset_a': 0.0,
                         'offset_b': 0.0,
                         'offset_c': 0.0}}

            
# Machine Accelerations and Velocities
max_arm_velocity = 360.0            # deg/sec
max_arm_acceleration = 720.0        # deg/sec^2
max_eoat_velocity_xyz = 1000.0      # mm/sec
max_eoat_acceleration_xyz = 20000.0 # mm/sec^2
max_eoat_velocity_abc = 360.0       # deg/sec
max_eoat_acceleration_abc = 720.0   # deg/sec^2

cycle_duration = 0.001              # sec

current_fixture = fixture_offsets['1']

initial_velocity_xyz = 0.0
initial_velocity_x = 0.0
initial_velocity_y = 0.0
initial_velocity_z = 0.0
initial_velocity_abc = 0.0
initial_velocity_a = 0.0
initial_velocity_b = 0.0
initial_velocity_c = 0.0
feed_rate = 0.0
g_number = None
x_number = None
y_number = None
z_number = None
a_number = None
b_number = None
c_number = None
f_number = None
initial_arm_velocities = {'a': 0.0,
                          'b': 0.0,
                          'c': 0.0,
                          'd': 0.0,
                          'e': 0.0,
                          'f': 0.0}

old_machine_position = kinematics.fixture_to_machine_coordinates(current_fixture)

gcode_file = open(gcode_filename, 'r')
for gcode_line in gcode_file.readlines():
    start_time = time.time()
    print('gcode_line = ' + gcode_line[:-1])
    i_number = None
    j_number = None
    k_number = None
    p_number = None
    q_number = None
    r_number = None
    line_data = gcode_line.split()
    for i in gcode_line.split():
        if i[0] == 'g' or i[0] == 'G':
            g_number = float(i[1:])
        elif i[0] == 'x' or i[0] == 'X':
            x_number = float(i[1:])
        elif i[0] == 'y' or i[0] == 'Y':
            y_number = float(i[1:])
        elif i[0] == 'z' or i[0] == 'Z':
            z_number = float(i[1:])
        elif i[0] == 'a' or i[0] == 'A':
            a_number = float(i[1:])
        elif i[0] == 'b' or i[0] == 'B':
            b_number = float(i[1:])
        elif i[0] == 'c' or i[0] == 'C':
            c_number = float(i[1:])
        elif i[0] == 'i' or i[0] == 'i':
            i_number = float(i[1:])
        elif i[0] == 'j' or i[0] == 'J':
            j_number = float(i[1:])
        elif i[0] == 'k' or i[0] == 'K':
            k_number = float(i[1:])
        elif i[0] == 'p' or i[0] == 'P':
            p_number = float(i[1:])
        elif i[0] == 'q' or i[0] == 'Q':
            q_number = float(i[1:])
        elif i[0] == 'r' or i[0] == 'R':
            r_number = float(i[1:])
        elif i[0] == 'f' or i[0] == 'F':
            f_number = float(i[1:])
    
    if g_number == 1:
        print('performing g01 interpolated_motion')
        
        if x_number != None:
            current_fixture['x'] = x_number
        if y_number != None:
            current_fixture['y'] = y_number
        if z_number != None:
            current_fixture['z'] = z_number
        if a_number != None:
            current_fixture['a'] = a_number
        if b_number != None:
            current_fixture['b'] = b_number
        if c_number != None:
            current_fixture['c'] = c_number
        if f_number != None:
            feed_rate = f_number
        
        if feed_rate == 0.0:
            print('ERROR: feed_rate = 0, must be a positive number')
            break  
        
        new_machine_position = kinematics.fixture_to_machine_coordinates(current_fixture)
        
        machine_delta_x = new_machine_position['x'] - old_machine_position['x']
        machine_delta_y = new_machine_position['y'] - old_machine_position['y']
        machine_delta_z = new_machine_position['z'] - old_machine_position['z']
        machine_delta_a = new_machine_position['a'] - old_machine_position['a']
        machine_delta_b = new_machine_position['b'] - old_machine_position['b']
        machine_delta_c = new_machine_position['c'] - old_machine_position['c']
        print('machine_delta_x = ' + str(machine_delta_x) + 'mm')
        print('machine_delta_y = ' + str(machine_delta_y) + 'mm')
        print('machine_delta_z = ' + str(machine_delta_z) + 'mm')
        print('machine_delta_a = ' + str(machine_delta_a) + ' deg')
        print('machine_delta_b = ' + str(machine_delta_b) + ' deg')
        print('machine_delta_c = ' + str(machine_delta_c) + ' deg')
        
        machine_delta_xyz = math.sqrt((machine_delta_x**2) + (machine_delta_y**2) + (machine_delta_z**2))
        print('machine_delta_xyz = ' + str(machine_delta_xyz) + 'mm')
        xyz_move_time = 0.0
        cycle_distance_xyz = 0.0
        if abs(machine_delta_xyz) > 0:
            xyz_move_time = machine_delta_xyz / (feed_rate / 60)
            print('xyz_move_time = ' + str(xyz_move_time) + ' secomds')
            cycle_distance_xyz = machine_delta_xyz / (xyz_move_time / cycle_duration)
            print('cycle_distance_xyz = ' + str(cycle_distance_xyz) + 'mm')
        else:
            print('No XYZ motion')
        
        
        
        
        
        
        machine_delta_abc = math.sqrt((machine_delta_a**2) + (machine_delta_b**2) + (machine_delta_c**2))
        print('machine_delta_abc = ' + str(machine_delta_abc) + ' deg')
        abc_move_time = 0.0
        cycle_distance_abc = 0.0
        if abs(machine_delta_abc) > 0:
            abc_move_time = machine_delta_abc / max_eoat_velocity_abc
            print('abc_move_time = ' + str(abc_move_time) + ' secomds')
            cycle_distance_abc = machine_delta_abc / (abc_move_time / cycle_duration)
            print('cycle_distance_abc = ' + str(cycle_distance_abc) + ' deg')
        else:
            print('No ABC motion')
        
        
        
        
        
            
        old_machine_position = new_machine_position
        print('')
    
    '''
    while abs(delta_x) > 0 and abs(delta_y) > 0 and abs(delta_z) > 0 and abs(delta_a) > 0 and abs(delta_b) > 0 and abs(delta_c) > 0:
        cycle_eoat_velocity = previous_eoat_velocity + (max_eoat_acceleration * cycle_time)
        
    '''
    
    
    
    
    
    
    
    
    
    
    
    
    
        

# Find machine coordinates from fixture coordinates
start_time = time.time()
machine_position = kinematics.fixture_to_machine_coordinates(current_fixture)
closest_arm_positions = kinematics.get_closest_arm_positions(machine_position)
elapsed_time = time.time() - start_time
print('elapsed_time: ' + str(elapsed_time))
print('machine_position = ' + str(machine_position))
print('closest_arm_positions: ' + str(closest_arm_positions))




























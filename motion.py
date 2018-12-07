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
eoat_acceleration_xyz = 1000.0      # mm/sec^2
max_eoat_velocity_abc = 360.0       # deg/sec
eoat_acceleration_abc = 720.0       # deg/sec^2
feed_rate = 0.0                     # mm/min
cycle_duration = 0.001              # sec

current_fixture = fixture_offsets['1']

previous_velocity = {'x': 0.0,
                    'y': 0.0,
                    'z': 0.0,
                    'xyz': 0.0,
                    'a': 0.0,
                    'b': 0.0,
                    'c': 0.0,
                    'abc': 0.0}

current_velocity = {}

current_gcode_numbers = {'g': None,
                         'x': None,
                         'y': None,
                         'z': None,
                         'a': None,
                         'b': None,
                         'c': None,
                         'f': None,
                         'i': None,
                         'j': None,
                         'k': None,
                         'p': None,
                         'q': None,
                         'r': None}

initial_arm_velocities = {'a': 0.0,
                          'b': 0.0,
                          'c': 0.0,
                          'd': 0.0,
                          'e': 0.0,
                          'f': 0.0}

def parse_gcode_line(line, gcode_numbers):
    line_data = line.split()
    for i in line_data:
        if i[0] == 'g' or i[0] == 'G':
            gcode_numbers['g'] = float(i[1:])
        elif i[0] == 'x' or i[0] == 'X':
            gcode_numbers['x'] = float(i[1:])
        elif i[0] == 'y' or i[0] == 'Y':
            gcode_numbers['y'] = float(i[1:])
        elif i[0] == 'z' or i[0] == 'Z':
            gcode_numbers['z'] = float(i[1:])
        elif i[0] == 'a' or i[0] == 'A':
            gcode_numbers['a'] = float(i[1:])
        elif i[0] == 'b' or i[0] == 'B':
            gcode_numbers['b'] = float(i[1:])
        elif i[0] == 'c' or i[0] == 'C':
            gcode_numbers['c'] = float(i[1:])
        elif i[0] == 'i' or i[0] == 'i':
            gcode_numbers['i'] = float(i[1:])
        elif i[0] == 'j' or i[0] == 'J':
            gcode_numbers['j'] = float(i[1:])
        elif i[0] == 'k' or i[0] == 'K':
            gcode_numbers['k'] = float(i[1:])
        elif i[0] == 'p' or i[0] == 'P':
            gcode_numbers['p'] = float(i[1:])
        elif i[0] == 'q' or i[0] == 'Q':
            gcode_numbers['q'] = float(i[1:])
        elif i[0] == 'r' or i[0] == 'R':
            gcode_numbers['r'] = float(i[1:])
        elif i[0] == 'f' or i[0] == 'F':
            gcode_numbers['f'] = float(i[1:])
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
    global cycle_duration
    v_data = {}
    t = pd['xyz'] / mv
    v_data['time_xyz'] = t
    print('move time = ' + str(t) + ' secomds')
    v_data['cycle_distance_xyz'] = pd['xyz'] / (t / cycle_duration)
    print('cycle distance = ' + str(v_data['cycle_distance_xyz']) + 'mm')
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
    
def get_acceleration(v_delta, iv):
    global eoat_acceleration_xyz
    a_data = {}
    a_data['accel_time_xyz'] = v_delta['delta_xyz'] / eoat_acceleration_xyz
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











start_time = time.time()

old_machine_position = kinematics.fixture_to_machine_coordinates(current_fixture)

gcode_file = open(gcode_filename, 'r')
gcode_lines = gcode_file.readlines()
for line_number in range(len(gcode_lines)):
    start_time = time.time()
    current_gcode_line = gcode_lines[line_number]
    print('current_gcode_line = ' + current_gcode_line[:-1])
    current_gcode_numbers = parse_gcode_line(current_gcode_line, current_gcode_numbers)
    print('current_gcode_numbers = ' + str(current_gcode_numbers))
    
    if current_gcode_numbers['g'] == 1:
        print('performing g01 interpolated_motion')      
        commanded_fixture_position, feed_rate = get_fixture_position(current_gcode_numbers, current_fixture)
        print('commanded_fixture_position = ' + str(commanded_fixture_position))
        if feed_rate == 0.0:
            print('ERROR: feed_rate = 0, must be a positive number')
            break  
        
        move_position = kinematics.fixture_to_machine_coordinates(commanded_fixture_position)
        move_position_delta = get_position_delta(move_position, old_machine_position)
        print('move_position_delta = ' + str(move_position_delta))

        move_velocity_xyz = feed_rate / 60
        print('move_velocity_xyz = ' + str(move_velocity_xyz) + 'mm/sec')
        
        move_velocity_data = {}
        move_acceleration_data = {}
        if abs(move_position_delta['xyz']) > 0:
            move_velocity_data = get_velocity(move_position_delta, previous_velocity, move_velocity_xyz)
            move_acceleration_data = get_acceleration(move_velocity_data, previous_velocity)
            current_velocity['x'] = move_velocity_data['x']
            current_velocity['y'] = move_velocity_data['y']
            current_velocity['z'] = move_velocity_data['z']
            current_velocity['xyz'] = move_velocity_xyz
            
            next_velocity_delta = {}
            next_acceleration_data = {}
            next_gcode_numbers = {}
            next_fixture = {}
            next_feed_rate = 0.0
            if len(gcode_lines) >= line_number + 2:
                next_gcode_line = gcode_lines[line_number + 1]
                print('next_gcode_line = ' + next_gcode_line[:-1])
                next_gcode_numbers = current_gcode_numbers
                next_gcode_numbers = parse_gcode_line(next_gcode_line, next_gcode_numbers)
                print('next_gcode_numbers = ' + str(next_gcode_numbers))
                if next_gcode_numbers['g'] == 1:
                    print('found another g01 interpolated_motion')
                    for f in commanded_fixture_position: 
                        next_fixture[f] = commanded_fixture_position[f]
                    next_fixture, next_feed_rate = get_fixture_position(next_gcode_numbers, next_fixture)
                    print('next_fixture = ' + str(next_fixture))
                    
                    next_machine_position = kinematics.fixture_to_machine_coordinates(next_fixture)
                    next_position_delta = get_position_delta(next_machine_position, move_position)
                    print('next_position_delta = ' + str(next_position_delta))
                    
                    next_velocity_xyz = next_feed_rate / 60
                    print('next_velocity_xyz = ' + str(next_velocity_xyz) + 'mm/sec')
                    
                    ##next_velocity_delta = {}
                    ##next_acceleration_data = {}
                    
                    print('current_velocity = ' + str(current_velocity))
                    if abs(move_position_delta['xyz']) > 0:
                        next_velocity_delta = get_velocity(next_position_delta, current_velocity, next_velocity_xyz)
                        print('next_velocity_delta = ' + str(next_velocity_delta))
                        next_acceleration_data = get_acceleration(next_velocity_delta, current_velocity)
                    
                else:
                    print('next line is not g01, so assume zero velocity')
                
            
            
                
                    
            else:
                print('no next line, so stop at current gcode position')
            
            
            # Accelerate to current gcode move velocity
            
            
            
            
            
            
            
            # Remove after new position after accel and decell is found
            ##commanded_fixture_position
        
        
        
        
        else:
            print('No XYZ motion')    
        


        
        
        for i in move_position:
            old_machine_position[i] = move_position[i]
        print('')
    else:
        print('Not a g01 move!!!!!!!!!')
    
    for i in previous_velocity:
        current_velocity[i] = previous_velocity[i]
    
    
    
    
elapsed_time = time.time() - start_time    

print('elapsed_time = ' + str(elapsed_time))






















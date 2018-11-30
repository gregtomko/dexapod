#!/usr/bin/python3

second_arm_length = 800     # mm
arm_radius = 300.0          # mm
circle_xyz_array = []
min_arm_angle = 282.5       # deg.
max_arm_angle = 462.5       # deg.
motor_pulley_teeth = 18.0
arm_pulley_teeth = 175.0
steps_per_rev = 200.0
micro_stepping = 8.0
total_steps = ((steps_per_rev * micro_stepping) * (arm_pulley_teeth / motor_pulley_teeth)) / 2
angular_resolution = 180 / total_steps
arm_radial_offset = 103.3012
max_arm_acceleration = 90.0
max_end_acceleration = 10000.0

arm_rotations = {'a':[0, 0, 240],
                 'b':[0, 0, 300],
                 'c':[0, 0, 0],
                 'd':[0, 0, 60],
                 'e':[0, 0, 120],
                 'f':[0, 0, 180]}

end_bearing_offsets = {'a':[-32.825, -128.1261, 23.5978],
                       'b':[32.825, -128.1261, 23.5978],
                       'c':[127.373, 35.6358, 23.5978],
                       'd':[94.548, 92.4903, 23.5978],
                       'e':[-94.548, 92.4903, 23.5978],
                       'f':[-127.373, 35.6358, 23.5978]}









#!/usr/bin/python3

import inputs, time

joystick_deadband = {'ABS_Z': [127, 133, 'left joystick x'],
                     'ABS_Y': [147, 153, 'left joystick y'],
                     'ABS_RZ': [147, 153, 'right joystick y'],
                     'ABS_RX': [128, 132, 'right joystick x']
                     }

key_names = {'BTN_BASE': 'left bottom key',
             'BTN_BASE2': 'right bottom key',
             'BTN_PINKIE': 'right top key',
             'BTN_TOP2': 'left top key',
             'BTN_TRIGGER': '1 key',
             'BTN_TOP': '4 key',
             'BTN_THUMB2': '3 key',
             'BTN_THUMB': '2 key',
             'BTN_BASE3': 'select key',
             'BTN_BASE4': 'start key',
             'ABS_Z': 'left joystick x',
             'ABS_Y': 'left joystick y',
             'ABS_RZ': 'right joystick y',
             'ABS_RX': 'right joystick x',
             'ABS_HAT0Y': 'up/down button',
             'ABS_HAT0X': 'lef/rightt button',
             }


while True:
    events = inputs.get_gamepad()
    for event in events:
        ##print(event.ev_type, event.code, event.state)
        event_code = event.code
        if event_code in key_names:
            event_state = int(event.state)
            if event_code in joystick_deadband:
                if event_state < joystick_deadband[event_code][0] or event_state > joystick_deadband[event_code][1]:
                        print(joystick_deadband[event_code][2] + ' = ' + str(event_state))
            else:
                print(key_names[event_code] + ' = ' + str(event_state))

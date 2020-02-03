import time

from pyPS4Controller.controller import Controller
import anki_vector
from anki_vector.util import degrees


class State:

    def __init__(self):

        self.is_x_pressed = False
        self.is_triangle_pressed = False
        self.is_circle_pressed = False
        self.is_square_pressed = False
        self.is_L1_pressed = False
        self.is_R1_pressed = False
        self.l3_acceleration_value = 0
        self.r3_acceleration_value = 0
        self.last_motor_state_update = time.time()


class MyController(Controller):
    # requires pyPS4Controller version 1.1.2+

    def __init__(self, interface, via_bluetoothctl, robot):
        super().__init__(interface, via_bluetoothctl)
        self.state = State()
        self.robot = robot
        self.max_speed = 300  # mm/sec
        self.max_joystick_input_value = 32767

    def __update_motor_state(self):

        def __get_mm_per_second(value):
            try:
                input_percentage = float(value) / float(self.max_joystick_input_value)
            except ZeroDivisionError:
                input_percentage = 0
            return input_percentage * self.max_speed

        left_mm_per_sec = __get_mm_per_second(self.state.l3_acceleration_value) * -1
        right_mm_per_sec = __get_mm_per_second(self.state.r3_acceleration_value) * -1

        seconds_sense_last_update = time.time() - self.state.last_motor_state_update
        if self.via_bluetooth:
            if right_mm_per_sec == 0 or left_mm_per_sec == 0:
                print("Priority motor state update.")
                self.state.last_motor_state_update = time.time()
                self.robot.motors.set_wheel_motors(left_mm_per_sec, right_mm_per_sec)
            elif seconds_sense_last_update > 1:
                print("Non-priority motor state update.")
                self.state.last_motor_state_update = time.time()
                self.robot.motors.set_wheel_motors(left_mm_per_sec, right_mm_per_sec)
            else:
                print("Cant update the motor state right now. Please wait: {} seconds."
                      .format(float(1-seconds_sense_last_update)))
        else:
            self.robot.motors.set_wheel_motors(left_mm_per_sec, right_mm_per_sec)

    def on_x_press(self):
        print("on_x_press")

    def on_x_release(self):
        print("on_x_release")

    def on_triangle_press(self):
        print("on_triangle_press")

    def on_triangle_release(self):
        print("on_triangle_release")

    def on_circle_press(self):
        print("on_circle_press")

    def on_circle_release(self):
        print("on_circle_release")

    def on_square_press(self):
        print("on_square_press")

    def on_square_release(self):
        print("on_square_release")

    def on_L1_press(self):
        print("on_L1_press")

    def on_L1_release(self):
        print("on_L1_release")

    def on_L2_press(self, value):
        print("on_L2_press: ", value)

    def on_L2_release(self):
        print("on_L2_release")

    def on_R1_press(self):
        print("on_R1_press")

    def on_R1_release(self):
        print("on_R1_release")

    def on_R2_press(self, value):
        print("on_R2_press: ", value)

    def on_R2_release(self):
        print("on_R2_release")

    def on_up_arrow_press(self):
        print("on_up_arrow_press")

    def on_up_down_arrow_release(self):
        print("on_up_down_arrow_release")

    def on_down_arrow_press(self):
        print("on_down_arrow_press")

    def on_left_arrow_press(self):
        print("on_left_arrow_press")

    def on_left_right_arrow_release(self):
        print("on_left_right_arrow_release")

    def on_right_arrow_press(self):
        print("on_right_arrow_press")

    def on_L3_up(self, value):
        print("on_L3_up: ", value)
        self.state.l3_acceleration_value = value
        self.__update_motor_state()

    def on_L3_down(self, value):
        print("on_L3_down: ", value)
        self.state.l3_acceleration_value = value
        self.__update_motor_state()

    def on_L3_left(self, value):
        print("on_L3_left: ", value)

    def on_L3_right(self, value):
        print("on_L3_right: ", value)

    def on_L3_release(self):
        print("on_L3_release")
        self.state.l3_acceleration_value = 0
        self.__update_motor_state()

    def on_R3_release(self):
        print("on_R3_release")
        self.state.r3_acceleration_value = 0
        self.__update_motor_state()

    def on_R3_up(self, value):
        print("on_R3_up: ", value)
        self.state.r3_acceleration_value = value
        self.__update_motor_state()

    def on_R3_down(self, value):
        print("on_R3_down: ", value)
        self.state.r3_acceleration_value = value
        self.__update_motor_state()

    def on_R3_left(self, value):
        print("on_R3_left: ", value)

    def on_R3_right(self, value):
        print("on_R3_right: ", value)


args = anki_vector.util.parse_command_args()
with anki_vector.Robot(args.serial,
                       enable_custom_object_detection=True,
                       enable_nav_map_feed=True,
                       enable_audio_feed=True,
                       default_logging=False) as robot:
    MyController(interface="/dev/input/js0", via_bluetoothctl=True, robot=robot).listen()

from threading import Thread, Event
from queue import Queue
from time import sleep

class VehicleLogic:
    def __init__(self, msg_queue):
        self.msg_queue = msg_queue
        # Initialize vehicle state
        self.speed = 0.0
        self.steering = 0.0

    def update_vehicle_logic_kbd(self, input_character):
        # Update vehicle logic based on keyboard input
        # Example implementation: 
        if input_character == 'w':
            self.speed = 1.0
        elif input_character == 's':
            self.speed = -1.0
        elif input_character == 'a':
            self.steering = -1.0
        elif input_character == 'd':
            self.steering = 1.0
        elif input_character == 'q' or input_character == 'Q':
            self.msg_queue.put("q")  # Signal to stop threads

    def get_speed_and_steering_data(self, input_character):
        # Calculate the next speed and steering data based on vehicle state
        return self.speed, self.steering

class MotorSteeringDriver:
    NEUTRAL_ZERO_POINT = 1500
    MAX_FWD_POINT = 2000
    MAX_REV_POINT = 1000
    STEERING_NOMINAL_CENTER = 1500
    STEERING_OFFSET = -60
    STEERING_PLUS_MINUS = 190

    def __init__(self, msg_queue):
        self.msg_queue = msg_queue

    def set_speed_steering(self, speed, steering):
        # Clip values
        speed = max(min(speed, 1.0), -1.0)
        steering = max(min(steering, 1.0), -1.0)
        # Calculate PWM values
        pwm_speed_pulse = int((speed + 1) * (self.MAX_FWD_POINT - self.MAX_REV_POINT) / 2 + self.NEUTRAL_ZERO_POINT)
        pwm_steering_pulse = int(steering * self.STEERING_PLUS_MINUS + self.STEERING_NOMINAL_CENTER + self.STEERING_OFFSET)
        # Apply PWM signals
        self.msg_queue.put((pwm_speed_pulse, pwm_steering_pulse))

    def stop(self):
        self.set_speed_steering(0.0, 0.0)

def keyboard_input_thread(kbd_queue, msg_queue):
    print("Keyboard input thread started")
    while True:
        input_character = input("Enter a character: ")
        if input_character == 'q' or input_character == 'Q':
            kbd_queue.put(input_character)
            msg_queue.put("Character sent to car")
            break
        else:
            kbd_queue.put(input_character)
            msg_queue.put("Character sent to car")

def status_display_thread(msg_queue):
    print("Status display thread started")
    while True:
        message = msg_queue.get()
        print(message)
        if message == "q":
            break

def motor_steering_control_thread(kbd_queue, msg_queue, run_flag):
    vehicle_logic = VehicleLogic(msg_queue)
    motor_steering_driver = MotorSteeringDriver(msg_queue)
    speed, steering = vehicle_logic.get_speed_and_steering_data('')
    motor_steering_driver.set_speed_steering(speed, steering)
    while run_flag.is_set():
        input_character = kbd_queue.get()
        vehicle_logic.update_vehicle_logic_kbd(input_character)
        speed, steering = vehicle_logic.get_speed_and_steering_data(input_character)
        motor_steering_driver.set_speed_steering(speed, steering)
        sleep(0.1)
    motor_steering_driver.stop()

def master_vehicle_process():
    run_flag = Event()
    run_flag.set()
    msg_queue = Queue()
    display_thread = Thread(target=status_display_thread, args=(msg_queue,))
    display_thread.start()
    kbd_queue = Queue()
    controller_thread = Thread(target=motor_steering_control_thread, args=(kbd_queue, msg_queue, run_flag))
    controller_thread.start()
    keyboard_thread = Thread(target=keyboard_input_thread, args=(kbd_queue, msg_queue))
    keyboard_thread.start()
    keyboard_thread.join()
    run_flag.clear()
    display_thread.join()

master_vehicle_process()
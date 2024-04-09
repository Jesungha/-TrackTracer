"""main software, driving with keyboard control
versions...
- v01: initial version, working on structure, keyboard input, terminal display, motor/servo control"""
import sys
import numpy as np
import ctypes
import queue
from threading import Thread
import time
from datetime import datetime
import curses
#import PCA9685 as SDH
import pigpio
from multiprocessing import Process, Value, Array, Event, Pipe 
from multiprocessing import Queue as MpQueue
import io 
import os

DEBUG = True
##FIX
#PCA9685_STUB = False
pigpio.exceptions = False
pi = pigpio.pi()

# define the keyboard inputs that we accept for driving... feel free to change
STOP_VEHICLE = ord(' ')
LEFT_TURN = curses.KEY_LEFT
RIGHT_TURN = curses.KEY_RIGHT
SPEED_UP = curses.KEY_UP
SPEED_DOWN = curses.KEY_DOWN
QUIT = ord('q')
NOOP = 'noop'
# and we define a max size for the keyboard queue, just as a safety...
KBD_QUEUE_MAX_SIZE = 10



def keyboard_input_thread (screen, kbd_queue, msg_queue):
    # just loop - drops out when QUIT received...
    while True:
        ch = screen.getch()
        msg_queue.put_nowait([0, 0, 'Keyboard Character In: %s at %.4f' % (str(ch), time.time())])
        if ch == STOP_VEHICLE:
            ch_text = 'Keyboard Action: STOP'
        elif ch == SPEED_UP:
            ch_text = 'Keyboard Action: SPEED_UP' 
        elif ch == SPEED_DOWN:
            ch_text = 'Keyboard Action: SPEED_DOWN'
        elif ch==LEFT_TURN:
            ch_text = 'Keyboard Action: LEFT_TURN' 
        elif ch== RIGHT_TURN:
            ch_text = 'Keyboard Action: RIGHT_TURN' 
        elif ch == QUIT:
            ch_text = 'Keyboard Action: QUIT'
        else:
            ch_text = 'Keyboard Action: UNKNOWN' 
            msg_queue.put_nowait([1, 0, ch_text])
        # and for all cases, queue the character(s) 
        try:
            kbd_queue.put_nowait(ch)
        except queue.Full:
            #incase we have limit on queue size
            pass
            if DEBUG:
                msg_queue.put_nowait ([21, 0, 'Keyboard Queue Full'])
        if ch == QUIT:
            msg_queue.put_nowait ([20, 0, 'Keyboard Action: QUIT'])
            return



# have a centralised handler for displaying the status in the terminal 
#reads messages off a common msg_queue and displays them
#runs as a separate thread
def output_display_thread (screen, msg_queue):
    while True:
        # we keep the message display going until the end terminates with an Exceptio
        try:
            x_pos, y_pos, msg = msg_queue.get(block=True, timeout=None) 
        except Exception:
            return
        screen.move(x_pos, y_pos) 
        screen.clrtoeol()
        screen.addstr(x_pos, y_pos, msg) 
        screen.refresh()



# the vehicle logic
# works out how the users keboard input commands are to be converted into signals to # send off to the motor and the steering (target range between +/- 1.0)
#writes out suitable status messages to a msg_queue
class VehicleLogic:
    """
    note... the VehicleLogicClass needs to be flexible, as in the longer term, there will likely be multiple possible inputs...
    - the keyboard
    - keyboard steering, we will treat as pulses that decay with time
    - keyboard speed, we will treat as explicit values
    future inference input, treat as explicit values
    - future noise, which is additive
    """
    # the inputs from the keyboard for steering are pulses that decay, # so they provide pulses to get the steering going in the right direction
    KBD_SPEED_INC = 0.01 # how to we increment the speed
    KBD_SPEED_DEC = 0.01
    # and how do we decrement the speed
    KBD_STEERING_PULSE = 0.2 
    KBD_STEERING_DECAY = 0.97
    # magnitude the keyboard steering pulse # how do we decay the steering pulses
    def __init__(self, msg_queue):
        # any potential messages for the terminal 
        self.msg_queue = msg_queue
        # initialize the two main kbd variables that define the vehicle state...
        self.kbd_speed = 0.0 # valid values between +/- 1.0
        self.kbd_steering_mag = 0.0 # valid values between 0.0 and 1.0
        self.kbd_steering_sign = 1 # steering has both a magnitude and a sign...
        
        #and placeholders for future interference data
        self.inf_speed = 0.0
        self.inf_steering = 0.0
        
    def update_vehicle_logic_kbd(self, kbd_input):
        if kbd_input == NOOP: 
            self._set_noop()
        elif kbd_input == STOP_VEHICLE: 
            self._set_stop()
        elif kbd_input == SPEED_UP: 
            self._set_speed_up() 
        elif kbd_input == SPEED_DOWN: 
            self._set_slow_down ()
        elif kbd_input == RIGHT_TURN: 
            self._set_veer_right()
        elif kbd_input == LEFT_TURN:
            self._set_veer_left()
    
    

    def get_next_speed_steering_data(self):
        # in the future we may include noise
        speed_noise = 0.0 # self.speed_noise.next_val()
        steering_noise = 0.0# self.steering_noise.next_val()
        # calculate the speed and steering... kbd (+noise) + inference data... 
        speed, steering = self._get_speed_steering (speed_noise, steering_noise)
        # and then decay the steering magnitude for kbd inputs we are pulsing 
        # self.kbd_steering_mag*= self.KBD_STEERING_DECAY
        # and create a vehicle status string... maybe useful for saving 
        current_timestamp = datetime.now()
        vehicle_status = '%s, %.6f,%.6f, %.6f, %.6f' % (current_timestamp.strftime('%H-%M-%S'),speed, steering, speed_noise, steering_noise)
        return speed, steering, current_timestamp, vehicle_status
    
    
    def _get_speed_steering (self, speed_noise=0.0, steering_noise=0.0):
        # calculate the speed and steering... kbd (+noise) + inference data... 
        total_speed = self._limit_range(min(self.kbd_speed, self.inf_speed)+ speed_noise)
        
        total_steering = self._limit_range(self.inf_steerin + (self.kbd_steering_mag * self.kbd_steering_sign) + steering_noise)
        return total_speed, total_steering
    
    def _limit_range(self, value):
        return np.clip(value, -1.0, 1.0)
    
    def _get_kbd_speed(self):
        return self.kbd_speed
    
    def _set_noop(self):
        pass
    
    def _set_speed_up(self):
        self.kbd_speed = self._limit_range(self.kbd_speed + self.KBD_SPEED_INC)
        self.inf_speed = self.kbd_speed
        
    def _set_slow_down(self):
        self.kbd_speed = self._limit_range(self.kbd_speed - self.KBD_SPEED_DEC)
    
    def _set_veer_right(self, pulse=KBD_STEERING_PULSE):
        #turn right
        new_steering  = self.kbd_steering_mag * self.kbd_steering_sign + pulse
        if new_steering >= 0.0:
            #positive direction
            self.kbd_steering_mag = new_steering
            self.kbd_steering_sign = 1
        else:
            #negative direction
            self.kbd_steering_mag = -new_steering
            self.kbd_steering_sign = -1
        #limit the mag of steering
        self.kbd_steering_mag = self._limit_range(self.kbd_steering_mag)
        
    def _set_veer_left(self, pulse=KBD_STEERING_PULSE):
        #turn left
        new_steering = self.kbd_steering_mag * self.kbd_steering_sign - pulse
        if new_steering >= 0.0:
            #positive direction
            self.kbd_steering_mag = new_steering
            self.kbd_steering_sign = 1
        else:
            #negative direction
            self.kbd_steering_mag = -new_steering
            self.kbd_steering_sign = -1
        #limit the mag of steering
        self.kbd_steering_mag = self._limit_range(self.kbd_steering_mag)
    
    def _set_stop(self):
        self.kbd_speed = 0.0
        self.inf_speed = 0.0
        self.kbd_steering_mag = 0.0
        self.kbd_steering_sign = 1
        self.inf_steering = 0.0
        return
    


# a wrapper for the steering and motor
# encapsulates details of how we set the motor and steering via the Servo hat 
class MotorSteeringDriver():
    """def set_servo_pulse(channel, pulse):
        pulse_length = 1000000    # 1,000,000 us per second
        pulse_length //= 60       # 60 Hz
        print('{0}us per period'.format(pulse_length))
        pulse_length //= 4096     # 12 bits of resolution
        print('{0}us per bit'.format(pulse_length))
        pulse *= 1000
        pulse //= pulse_length
        pwm.set_pwm(channel, 0, pulse)"""
    
    #### TO BE FIXED ###
    def __init__(self, msg_queue):
        # in case we need to log something to the terminal display 
        self.msg_queue = msg_queue
        # for the steering servo, channel 
        self.STEERING_SERVO_PIN= 25
        # speed controller, channel 3 
        self.ESC_SERVO_PIN = 27
        
        # Initialize the Hat
        self.PWM_FREQ_50_HZ = 50
        ##FIX
        #if not PCA9685_STUB:
            #self.pwm = SDH.PCA9685(self.SDH_I2C_ADDRESS, debug=False)
            #self.pwm.setPWMFreq(self.PWM FREQ_50_HZ)
        pi.set_mode(self.STEERING_SERVO_PIN, pigpio.OUTPUT) 
        pi.set_mode(self.ESC_SERVO_PIN, pigpio.OUTPUT)
        self.pwm = pi.set_PWM_frequency(self.STEERING_SERVO_PIN, self.PWM_FREQ_50_HZ)

        
        # define the three set-points for the ESC 
        self.NEUTRAL_ZERO_POINT = 1500
        self.MAX_FWD_POINT = 2000
        self.MAX_REV_POINT = 1000
        # and we convert these into scale factors

        self.FWD_SPEED_GAIN = (self.MAX_FWD_POINT - self.NEUTRAL_ZERO_POINT) / 1.0 
        self.REV_SPEED_GAIN = (self.NEUTRAL_ZERO_POINT - self.MAX_REV_POINT) / 1.0
        
        # and for the steering
        self._STEERING_NOMINAL_CENTER = 1500
        self._STEERING_OFFSET = -60
        self.STEERING_PLUS_MINUS = 280
        self.STEERING_CENTER = self._STEERING_NOMINAL_CENTER + self._STEERING_OFFSET # and the steering scale factors
        self.STEERING_GAIN = self.STEERING_PLUS_MINUS / 1.0
        
        # also lets define some restrictions on the speed and steering 
        self.STEERING_CLAMP = [-1.0, 1.0]
        self.SPEED_CLAMP = [-0.2, 0.2]
        
        # and lets center things...
        #FIX
        #if not PCA9685_STUB:
        
        
        #    self.pwm.setServoPulse (self.STEERING_SERVO_CHANNEL, self.STEERING_CENTER) 
        #    self.pwm.setServoPulse (self.ESC_SERVO_CHANNEL, self.NEUTRAL_ZERO_POINT)
        pi.set_servo_pulsewidth(self.STEERING_SERVO_PIN, self.STEERING_CENTER)
        pi.set_servo_pulsewidth(self.ESC_SERVO_PIN, self.NEUTRAL_ZERO_POINT)
        # and track the current speed and steering... should not be needed here 
        self.current_speed = 0.0 
        self.current_steering = 0.0
        
        
    
    # speed should be between +/- 1.0 
    # # full fwd and full reverse
    def set_speed (self, speed):
        speed = np.clip(speed, -1.0, 1.0)
        
        # and an optional restriction...
        speed= np.clip(speed, self.SPEED_CLAMP [0], self.SPEED_CLAMP [1])
        
        # and record it for luck 
        self.current_speed = speed
        
        # and we expand this now out to the PWM pulse widths
        if speed >= 0.0:
            # forward
            pwm_speed_pulse = int(speed * self. FWD_SPEED_GAIN + self.NEUTRAL_ZERO_POINT)
        else:
            # reverse
            pwm_speed_pulse = int(speed * self.REV_SPEED_GAIN + self.NEUTRAL_ZERO_POINT)
        # and now set the actual speed via the Servo driver hat
        #if not PCA9685_STUB:
        #   self.pwm.setServoPulse (self.ESC_SERVO_CHANNEL, pwm_speed_pulse)
        pi.set_servo_pulsewidth(self.ESC_SERVO_PIN, pwm_speed_pulse)
        self.msg_queue.put_nowait([6, 0, 'Motor Speed: %d' % pwm_speed_pulse])
            
    #steering should be between +/- 1.0
    def set_steering(self, steering):
        steering = np.clip(steering, -1.0, 1.0)
        # and an optional restriction...
        steering = np.clip(steering, self.STEERING_CLAMP [0], self.STEERING_CLAMP [1])
        # and record it for luck 
        self.current_steering = steering
        # and we expand this now out to the PWM pulse widths
        pwm_steering_pulse = int(steering * self.STEERING_GAIN + self.STEERING_CENTER)
        # and now set the actual steering via the Servo driver hat
        #FIX
        #if not PCA9685_STUB:
        #    self.pwm.setServoPulse (self.STEERING_SERVO_CHANNEL, pwm_steering_pulse)
        pi.set_servo_pulsewidth(self.STEERING_SERVO_PIN, pwm_steering_pulse)
        self.msg_queue.put_nowait([7, 0, 'Steering Angle: %d' % pwm_steering_pulse])
        
    def stop(self):
        self.set_speed(0.0)
        self.set_steering(0.0)
        
    def set_speed_steering(self, speed, steering):
        self.set_speed(speed)
        self.set_steering(steering)
            
            

# main speed and steering control of the vehicle
# reads user keyboard input from the kdb_queue, calculates the required motor and # steering commands, and applies to the motor and steering servo
# writes out suitable status messages to a msg_queue
# runs as a separate thread
# will continue running as defined by the run_flag
def motor_steering_control_thread (kbd_queue, msg_queue, run_flag):
    # a logical model for the vehicle...
    #here we combine all the inputs (kbd, inference), and noise, to calc the required # speed and steering actions
    vehicle_logic = VehicleLogic (msg_queue)
    # driver for the motor and steering... execute the speed and steering commands 
    motor_steering_driver = MotorSteeringDriver (msg_queue)
    # and initialize the system
    speed, steering,_,_ = vehicle_logic.get_next_speed_steering_data()
    motor_steering_driver.set_speed_steering(speed, steering)
    
    # and just collect some stats 
    max_timeout = 0.0
    min_timeout = 10.0 
    avg_timeout = 0.0 
    loop_count = 0
    # now, wait to start running... 
    run_flag.wait()
    # use a timestamp to try and get a repeatable loop period TARGET PERIOD
    last_timestamp = time.time()
    TARGET_PERIOD = 0.05


    # continually iterate whilst running
    # we loop here, say once every 0.05 seconds and update the motor speed # and steering based on the various inputs (keyboard, inference, noise) 
    while run_flag.is_set():
        
        #firstly, check out the keyboard queue non blocking
        try:
            # Queue either pops with the data at the head of the queue, or throws and retu 
            kbd_input = kbd_queue.get_nowait()
        except queue. Empty:
            kbd_input = NOOP
        # and update the vehicle logic
        vehicle_logic.update_vehicle_logic_kbd(kbd_input)
        
        # and now, hopefully at an evently spaced time period, we can grab the latest valu and push out to the servo and motor
        speed, steering, ts, vehicle_status = vehicle_logic.get_next_speed_steering_data() 
        motor_steering_driver.set_speed_steering (speed, steering)
        # now see if we need to sleep for a while in this loop 
        current_timestamp = time.time()
        time_to_sleep = max(0.0, TARGET_PERIOD - (current_timestamp -  last_timestamp))
        # if less than the TARGET PERIOD, we need to sleep
        if time_to_sleep > 0.0:
            time.sleep(time_to_sleep)
        
        last_timestamp = time.time()
        
        loop_count += 1
        max_timeout = max(max_timeout, time_to_sleep)
        min_timeout = min(min_timeout, time_to_sleep)
        avg_timeout += time_to_sleep
        
        msg_queue.put_nowait([3, 0, 'motor_speed_control_thread. sleep: (%.6f, %.6f, %.6f)' % (max_timeout, min_timeout, avg_timeout/loop_count)])
        
    motor_steering_driver.stop()

def master_vehicle_process():
    try:
        # a flag used for shutting down relevant processes and threads...
        run_flag = Event()
        run_flag.set()
        
        # setup the screen for capturing input key strokes, and displaying info to the use
        screen = curses.initscr()
        curses. oecho ()
        curses.cbreak()
        screen.keypad(True)
        # and a msg queue to display to the curses screen - shared across processes
        msg_queue =  MpQueue() # multiprocessing
        display_thread  = Thread (name='output_display_thread', target=output_display_thread , args=(screen, msg_queue))
        display_thread.start()
        # and the keyboard queue... shared between threads in a single process 
        kbd_queue = queue. Queue (KBD_QUEUE_MAX_SIZE)
        # and the motor and steering controller...
        controller_thread = Thread (name='motor_steering_control_thread', target=motor_steering_control_thread, args=(kbd_queue, msg_queue, run_flag))
        controller_thread.start()
        
        # instantiate the keyboard thread - main entry point for us to control everything 
        keyboard_thread = Thread (name='keyboard_input_thread', target=keyboard_input_thread, args=(screen, kbd_queue, msg_queue))
        keyboard_thread.start()
        
        # now we wait for the user to terminate with 'quit'
        keyboard_thread.join()
        
    finally:
        run_flag.clear()
        time.sleep(2)
        curses
        screen.keypad(False)
        curses.echo()
        curses.endwin()
        

def start_all():
    master_process = Process (name='master_vehicle_process', target=master_vehicle_process)
    master_process.start()
    master_process.join()
    
if __name__ == '__main__':
    start_all()
    
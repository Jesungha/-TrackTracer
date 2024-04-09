"""

collects user character input from the keyboard and sends it to the car
writes out suitable status messages
runs as a separate thread
quits after receiving a 'q' or 'Q' character
def keyboard_input_thread(kbd_queue, msg_queue):
    print("Keyboard input thread started")
    while True:
        input_character = block waiting to recieve keyboard input_character
         if input_charcter == "some meaningful contro,l character":
             kbd_queue.place_on_queue_non_blocking(input_character)
             msg_queue.place_on_queue_non_blocking("Character sent to car")
        if input_character == "q" or input_character == "Q":
               break


have a centralised handler for displaying the status in the terminal
reads messages from the message queue and displays them
runs as a separate thread
def status_display_thread(msg_queue):
    print("Status display thread started")
    while True:
        message = block waiting to receive a message from the message queue
        print(message)
        if message == "q":
            break


main speed and steering control of the vehicle
reads user keyboard input from the kbd_queue calculates the required motor and steering commands, and applies to motor and steering servo
writes out suitable status messages
runs as a separate thread
will continue running as defined by the run_flag
    
def motor_steering_control_thread( kbd_queue, msg_queue, run_flag):
    
    instantiate a logic model for the vehicle...vehicle state
    this maps user input characters into speed and steering commands. 
    vehicle_logic = VehicleLogic(msg_queue)

    instatiate a driver / wrapper for the motor and steering
    motor_steering_driver = MotorSteeringDriver(msg_queue)

    and initialize the motor and steering
    speed, steering = vehicle_logic.get_speed_and_steering_data('')
    motor_steering_driver.set_speed_and_steering(speed, steering)

    loop until run_flag says stop:
    while run_flag.is_set():
        input_character = block waiting to receive a character from the keyboard
        vehicle_logic.update_vehicle_logic_kbd(input_character)
        motor_steering_driver.get_next_speed_steering_Data(speed, steering)
        motor_steering_driver.set_speed_and_steering(speed, steering)
        
        sleep if needed to keep consistent loop timing
    motor_steering_driver.stop()

vehicle logic
worksout how the userkeyboard input commands are to be converted
sendoff 
Class VehicleLogic(msg_queue):
    
    #update the vehicle logic based on the keyboard input
    def update_vehicle_logic_kbd(self, input_character):
        #based on the input, update the speed state
        #based on the vehicle state, update the steering
    
    #calculate the next speed and steering data
    def get_speed_and_steering_data(self, input_character):
        #based on the vehicle state, calculate the speed and steering data
        return speed, steering

#driver/ wrapper for the steering and motor
#takes in the interneal speed and steering data and converts into pwm values need to drive the ESC and steering servo
#uses the calibration values set previously when configuring ESC and servo

class MotorSteeringDriver(msg_queue):


    # define the three set-points for the ESC
    NEUTRAL ZERO POINT := 1500
    MAX FWD POINT := 2000
    MAX REV POINT := 1000
    # and for the steering
    STEERING NOMINAL CENTER := 1500
    STEERING OFFSET=-60
    STEERING PLUS MINUS := 190
    STEERING CENTER :=
    STEERING NOMINAL CENTER + STEERING_OFFSET

    def set_speed_steering ( speed, steering ):
        # clip values
        speed := clip speed to valid [-1.0 or 0.0, +1.0] range
        steering := clip steering to valid [-1.0, +1.0] range
        # calculate the suitable / scaled PWM values based on calibrated values pwm_speed_pulse := map speed to the [MAX REV POINT, MAX FWD_POINT] range
        pwm_steering_pulse := map steering to the [STEERING CENTER +/- STEERING PLUS_MINUS] range
        
        apply the pwm pulse signals to the ESC and steering servo

    def stop():
        set_speed_steering(0.0, 0.0)
    
def master_vehicle_process():
    run_flag := initialize flag for controlling threads, processes
    # start up the output display, and the msg_queue for receiving the messages
    msg_queue = new message queue
    display_thread := new thread to run output_display_thread ( msg_queue )
    display_thread.start()

    # the queue for transporting keyboard commands
    kbd_queue := new keyboard command queue

    # and the motor and steering controller...
    controller_thread = new thread to run motor_steering_control_thread( kbd_queue, msg_queue, run_flag)
    controller thread.start()

    # start up the keyboard input thread
    keyboard_thread = new thread to run keyboard_input_thread ( kbd_queue,msg_queue)
    keyboard_thread.start()

    #everything should be running now until the user enters 'quit' wait for the user to terminate with 'quit'
    run_flag := reset the flag to shutdown threads
    sleep for a bit to let things shut down gracefully















"""
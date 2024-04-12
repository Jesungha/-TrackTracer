import threading
import time
import picamera

class SharedImageMemBuff:
    def __init__(self):
        self.image_data = None
        self.timestamp = None

class CustomImageProcessor:
    def __init__(self, shared_save_mem, shared_save_mem_flag, msg_queue):
        self.shared_save_mem = shared_save_mem
        self.shared_save_mem_flag = shared_save_mem_flag
        self.msg_queue = msg_queue

    def write(self, new_image_frame):
        if self.shared_save_mem_flag:
            self.shared_save_mem.image_data = new_image_frame
            self.shared_save_mem_flag = False  # Set flag to False after writing
            self.msg_queue.put('Helpful statistics on images processed')

def save_car_control_data_thread(ctrl_queue, run_flag):
    control_file = open_timestamped_control_file()
    while run_flag:
        control_data = ctrl_queue.get()  # Blocking call
        if control_data:
            control_file.write(control_data + '\n')
    control_file.close()

def open_timestamped_control_file():
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    return open(f"control_data_{timestamp}.txt", "a")

def camera_capture_process(shared_save_mem, shared_save_mem_flag, msg_queue, run_flag):
    with picamera.PiCamera() as camera:
        custom_image_processor = CustomImageProcessor(shared_save_mem, shared_save_mem_flag, msg_queue)
        camera.start_recording(custom_image_processor, format='rgb')
        while run_flag:
            time.sleep(1)  # Adjust as needed
        camera.stop_recording()

def save_image_process(shared_save_mem, shared_save_mem_flag, run_flag):
    while run_flag:
        if shared_save_mem_flag:
            # Write shared_save_mem to disk
            shared_save_mem_flag = False  # Reset flag after writing

import os
import csv
import pandas as pd
import rospy
from std_msgs.msg import Int32
import time
import datetime

# This path needs to be adjusted according to where you want the folders to be generated
base_directory = '/home/rosnoetic/Documents/3dv_logs'

class DataLogger:

    def __init__(self, logging_period=10):
        self.logging_period = logging_period
        self.occluded = []
        self.start_time = None

        self.init_time = datetime.datetime.now()
        self.path = os.path.join(base_directory, self.init_time.strftime('%Y-%m-%d_%H-%M-%S'))

        os.makedirs(self.path)
        print(f"Created directory for storing data: {self.path}")

        rospy.init_node('data_logger', anonymous=True)
        rospy.Subscriber('log/occluded', Int32, self.occluded_callback)

        print("Subscribed to 'log/occluded' topics")


    def occluded_callback(self, data):
        timestamp = rospy.get_time()  # get the current time
        self.occluded.append((timestamp, data.data))

    def start_logging(self):
        self.start_time = time.time()
        print("Started logging data")

        while not rospy.is_shutdown():
            if time.time() - self.start_time >= self.logging_period:
                self.save_to_csv()
                self.start_time = time.time()

    def save_to_csv(self):
        time_stamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        occluded_df = pd.DataFrame(self.occluded, columns=['timestamp', 'occluded'])

        occluded_path = os.path.join(self.path, f'occluded_{time_stamp}.csv')

        occluded_df.to_csv(occluded_path, index=False)

        print(f"Saved occluded data to: {occluded_path}")

        self.occluded = []

if __name__ == '__main__':
    print("Initializing DataLogger node...")
    data_logger = DataLogger(15)  # adjust this value for the logging period in seconds
    data_logger.start_logging()

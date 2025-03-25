#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This node logs the data from the gui.
It collects the data into numpy arrays then saves everything into a matlab structure
Subscribed: simulation/output/log
Message type: Log.msg
Save path: ~/Documents/crossing_gui_ros1/
TODOs in case of change:
    - add array to initialization
    - add append to callback
    - add array to matlab structure in log()
"""


# ============ IMPORTS: ============
# Import modules:
import sys
import time
import os
import math
import numpy as np
import scipy.io
from pathlib import Path    # for the save path

import matplotlib.pyplot as plt
import matplotlib


matplotlib.rcParams['mathtext.fontset'] = 'stix'
matplotlib.rcParams['font.family'] = 'STIXGeneral'

plt.title(r'ABC123 vs $\mathrm{ABC123}^{123}$')

# Import ROS2:
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor # for declaring parameters

from crossing_gui_ros2.simulation_node_functions.create_parameters import DeclareSimulationParameters

# Import ROS2 message definitions:
from crossing_gui_ros2_interfaces.msg import SimulationOutput

# Import ROS2 service definitions:
from std_srvs.srv import Empty

class LogNode(Node):
    def __init__(self):
        """Initialize and allocate memory for the logging."""
        super().__init__('log_node')


        parameter_declarator = DeclareSimulationParameters(self.declare_parameter, self.get_parameter)
        parameter_declarator.declare_parameters_for_log_node()
        # currently there are no parameters.

        # Create timer for periodic tasks:
        timer_period = 1/1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Subscribers:
        self.subscriber_simulation_output = self.create_subscription(SimulationOutput, 'simulation/output', self.callback_simulation_output, 10)

        # Publishers:
        # ---

        # Services:
        # Service to start the logging:
        self.service_startlog = self.create_service(Empty, 'startlog', self.callback_startlog)
        self.log_started = False

        # Service for saving the numpy arrays into a matlab file on disk:
        self.service_savelogfiles = self.create_service(Empty, 'savelogfiles', self.callback_savelogfiles)

        # Memory allocation:
        # self.t_stamp            = np.array([0.0])

        self.pedestrian_x = np.array([])
        self.pedestrian_y = np.array([])
        self.pedestrian_x_speed = np.array([])
        self.pedestrian_y_speed = np.array([])
        self.pedestrian_x_acceleration = np.array([])
        self.pedestrian_y_acceleration = np.array([])
        self.pedestrian_displaymessage = []

        self.pedestrian_intention = np.array([])
        self.pedestrian_intention_decay = np.array([])
        self.pedestrian_y_speed_smooth_decay = np.array([])

        self.vehicle_x = np.array([])
        self.vehicle_y = np.array([])
        self.vehicle_x_speed = np.array([])
        self.vehicle_y_speed = np.array([])
        self.vehicle_x_acceleration = np.array([])
        self.vehicle_y_acceleration = np.array([])
        self.vehicle_displaymessage = []

        self.time_data_arrived = np.array([])




        # Other variables:
        self.node_start_time = time.time()
        self.last_callback_time = 0.0
        self.was_there_any_callback_at_all = False

        self.i = 0

    def timer_callback(self):

        self.i += 1




    def callback_startlog(self, request: Empty, response: Empty) -> Empty:

        # Ok just create a flag, which tells this node to start to save the incoming data locally.

        if self.log_started == True:
            # If logging is already started, stop it:
            self.log_started = False
            self.get_logger().info('Logging stopped.')

            self.plot_and_save() # you have to change the code otherwise it wont work correctly
            self.reset_data_arrays()
        else:
            # If logging is not yet started, start it:
            self.log_started = True
            self.get_logger().info('Logging started.')
        return response
    
    def plot_and_save(self):
        # Plot the data
        
        # Trim the first 1 sec of the data:
        self.trim_beginning_of_data(1)
        
        # Set global font size:
        plt.rcParams.update({'font.size': 14})
        
        
        # steps = np.arange(len(self.pedestrian_x_speed))  # Create an array for time (steps) # delet later

        timesteps = self.time_data_arrived - self.time_data_arrived[0]

        fig, ax1 = plt.subplots()

        # Plot vehicle x speed and pedestrian y speed on the left y-axis
        ax1.plot(timesteps, self.vehicle_x_speed, label=r'$v_V(t)$',zorder=2, color='red', linewidth=2, linestyle='-')
        ax1.plot(timesteps, self.pedestrian_y_speed, label=r'$w_P(t)$',zorder=2, color='#a2b1d4', linewidth=2, linestyle='-')
        ax1.plot(timesteps, self.pedestrian_y_speed_smooth_decay, label=r'$w^*_P(t)$',zorder=2, color='#4664aa', linewidth=2, linestyle=':')
        ax1.set_xlabel(r'Zeit in $s$')
        ax1.set_ylabel(r'Geschwindigkeit in $m/s$')

        ax1.set_yticks(np.arange(0, max(max(self.vehicle_x_speed), max(self.pedestrian_y_speed)) + 1, 1))

        # ax1.grid(True, which='both',linestyle='-', linewidth=0.5, color='gray', zorder=0)
        ax1.set_axisbelow(True)

        # ax1.set_ylim(0, max(1, max(self.pedestrian_intention)))

        # Create a twin Axes for pedestrian_intention
        ax2 = ax1.twinx()
        ax2.plot(timesteps[::], self.pedestrian_intention[::], label=r'$I(t)$',zorder=2, color='#b2dfd9', linewidth=2, linestyle='-')  # Plot every 10th point
        ax2.plot(timesteps[::], self.pedestrian_intention_decay[::], label=r'$I^*(t)$',zorder=2, color='#009682', linewidth=2, linestyle=':')  # Plot every 10th point
        ax2.set_ylabel(r'Intention')
        ax2.set_ylim(0, 2)
        # ax2.set_yticks(np.arange(0, 2.1, 0.1))
        ax2.set_yticks([0.5, 1.0])
        # ax2.grid(True, which='both',linestyle='--', linewidth=0.5, color='gray', zorder=0)
        ax2.set_axisbelow(True)


        # Find indices where vehicle_x is between ...
        indices = np.where((self.vehicle_x >= 40 - 4.5/2) & (self.vehicle_x <= 40 + 4.5/2))[0]

        if len(indices) != 0:
            # Get the smallest and largest index
            min_index = np.min(indices)
            max_index = np.max(indices)

            # Fill the background for time steps where vehicle_x is between ...
            ax1.fill_between(timesteps[min_index:max_index], 0, max(self.vehicle_x_speed), color='red', alpha=0.3)

        # Find indices where pedestrian_y is between 20 and 30
        indices = np.where((self.pedestrian_y >= 10 - 1.5) & (self.pedestrian_y <= 10 + 1.5))[0]

        if len(indices) != 0:
            # Get the smallest and largest index
            min_index = np.min(indices)
            max_index = np.max(indices)

            # Fill the background for time steps where vehicle_x is between 20 and 30
            ax1.fill_between(timesteps[min_index:max_index], 0, max(self.vehicle_x_speed), color='#808080', alpha=0.2)

        # Combine legends for both axes
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        # legend = ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left', bbox_to_anchor=(1, 1),facecolor='white') # this was for presentation
        legend = ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right', facecolor='white')
        legend.get_frame().set_edgecolor("black")
        legend.get_frame().set_alpha(1)

        plt.title(r'')

        # Define the directory to save the plot
        save_dir = str(Path.home()) + "/ros2_ws/src/bogdan-master-thesis/documentation/plots/"
        os.makedirs(save_dir, exist_ok=True)

        # Define the file name for the plot
        file_name = save_dir + "plot_" + str(int(time.time())) + ".pdf"  # Use PDF format for vector graphics

        # Save the plot as a vector graphics file (PDF)
        plt.savefig(file_name, format='pdf', bbox_inches='tight') # pdf or svg

        # Close the plot to release resources
        plt.close()

        # Log the path where the plot is saved
        self.get_logger().info(f'Plot saved at: {file_name}')

    def reset_data_arrays(self):
        # Reset data arrays to initial values
        self.pedestrian_x = np.array([])
        self.pedestrian_y = np.array([])
        self.pedestrian_x_speed = np.array([])
        self.pedestrian_y_speed = np.array([])
        self.pedestrian_x_acceleration = np.array([])
        self.pedestrian_y_acceleration = np.array([])
        self.pedestrian_displaymessage = []

        self.pedestrian_intention = np.array([])
        self.pedestrian_intention_decay = np.array([])
        self.pedestrian_y_speed_smooth_decay = np.array([])

        self.vehicle_x = np.array([])
        self.vehicle_y = np.array([])
        self.vehicle_x_speed = np.array([])
        self.vehicle_y_speed = np.array([])
        self.vehicle_x_acceleration = np.array([])
        self.vehicle_y_acceleration = np.array([])
        self.vehicle_displaymessage = []

        self.time_data_arrived = np.array([])
        
        
    def trim_beginning_of_data(self, seconds_to_trim=1.0):
        # Find the time that is 1 second after the first timestamp
        time_to_trim = self.time_data_arrived[0] + seconds_to_trim

        # Find the index of the first timestamp that is greater than time_to_trim
        trim_index = np.argmax(self.time_data_arrived > time_to_trim)
        
        # Trim all arrays using the found index
        self.pedestrian_x = self.pedestrian_x[trim_index:]
        self.pedestrian_y = self.pedestrian_y[trim_index:]
        self.pedestrian_x_speed = self.pedestrian_x_speed[trim_index:]
        self.pedestrian_y_speed = self.pedestrian_y_speed[trim_index:]
        self.pedestrian_x_acceleration = self.pedestrian_x_acceleration[trim_index:]
        self.pedestrian_y_acceleration = self.pedestrian_y_acceleration[trim_index:]
        self.pedestrian_displaymessage = self.pedestrian_displaymessage[trim_index:]

        self.pedestrian_intention = self.pedestrian_intention[trim_index:]
        self.pedestrian_intention_decay = self.pedestrian_intention_decay[trim_index:]
        self.pedestrian_y_speed_smooth_decay = self.pedestrian_y_speed_smooth_decay[trim_index:]

        self.vehicle_x = self.vehicle_x[trim_index:]
        self.vehicle_y = self.vehicle_y[trim_index:]
        self.vehicle_x_speed = self.vehicle_x_speed[trim_index:]
        self.vehicle_y_speed = self.vehicle_y_speed[trim_index:]
        self.vehicle_x_acceleration = self.vehicle_x_acceleration[trim_index:]
        self.vehicle_y_acceleration = self.vehicle_y_acceleration[trim_index:]
        self.vehicle_displaymessage = self.vehicle_displaymessage[trim_index:]

        self.time_data_arrived = self.time_data_arrived[trim_index:]

    def callback_simulation_output(self, data):
        
        # We only want to start the logging if the startlog service was called:
        if self.log_started == True:
            
            # self.t_stamp            = np.append(self.t_stamp,            data.t_stamp)
            self.pedestrian_x       = np.append(self.pedestrian_x,       data.pedestrian_x)
            self.pedestrian_y       = np.append(self.pedestrian_y,       data.pedestrian_y)
            self.pedestrian_x_speed = np.append(self.pedestrian_x_speed, data.pedestrian_x_speed)
            self.pedestrian_y_speed = np.append(self.pedestrian_y_speed, - data.pedestrian_y_speed)
            self.pedestrian_x_acceleration = np.append(self.pedestrian_x_acceleration, data.pedestrian_x_acceleration)
            self.pedestrian_y_acceleration = np.append(self.pedestrian_y_acceleration, data.pedestrian_y_acceleration)
            self.pedestrian_displaymessage.append(data.pedestrian_displaymessage)
            self.pedestrian_intention = np.append(self.pedestrian_intention, data.pedestrian_intention)
            self.pedestrian_intention_decay = np.append(self.pedestrian_intention_decay, data.pedestrian_intention_decay)
            self.pedestrian_y_speed_smooth_decay = np.append(self.pedestrian_y_speed_smooth_decay, data.pedestrian_y_speed_smooth_decay)

            self.vehicle_x          = np.append(self.vehicle_x,          data.vehicle_x)
            self.vehicle_y          = np.append(self.vehicle_y,          data.vehicle_y)
            self.vehicle_x_speed    = np.append(self.vehicle_x_speed,    data.vehicle_x_speed)
            self.vehicle_y_speed    = np.append(self.vehicle_y_speed,    data.vehicle_y_speed)
            self.vehicle_x_acceleration = np.append(self.vehicle_x_acceleration, data.vehicle_x_acceleration)
            self.vehicle_y_acceleration = np.append(self.vehicle_y_acceleration, data.vehicle_y_acceleration)
            self.vehicle_displaymessage.append(data.vehicle_displaymessage)

            self.time_data_arrived = np.append(self.time_data_arrived, time.time())

    def callback_savelogfiles(self, request, response):
        """Service to ask this node to save the data to disk.

        Args:
            request (Empty):
            response (Empty):
        """

        # Call the saving function:
        self.log()

        # After saving is ready, shutdown the node:
        if self.getlogger_enable_log:
                self.get_logger().info(f'Shutdown')

        self.destroy_node()
        rclpy.shutdown()
        sys.exit()

    def log(self):
        """
        Save the data into a matlab file after node is shutdown.
        Data will be only saved if in gui log == True hence data is present in array
        """
        if len(self.t_stamp) > 1:
            if self.getlogger_enable_log:
                self.get_logger().info(f'Saving files ...')
            dir2save = str(Path.home()) + "/Documents/crossing_gui_ros2/"
            Path(dir2save).mkdir(parents=True, exist_ok=True)

            date = str(time.asctime(time.localtime()))
            date = date[-4:] + " " + date[4:-5]
            date = date.replace(' ', '_')
            file_name = dir2save + "log_" + date + ".mat"
            file_name = file_name.replace(':', '-')

            mat_data = {"Timestamp"         : self.t_stamp,
                        "pedestrian_x"      : self.pedestrian_x,
                        "pedestrian_y"      : self.pedestrian_y,
                        "vehicle_x"         : self.vehicle_x,
                        "vehicle_y"         : self.vehicle_y,
                        "distance"          : self.distance,
                        "pedestrian_x_speed": self.pedestrian_x_speed,
                        "pedestrian_y_speed": self.pedestrian_y_speed,
                        "vehicle_x_speed"   : self.vehicle_x_speed,
                        "vehicle_y_speed"   : self.vehicle_y_speed
                        }
            mat_data_struct = {"mat_data": mat_data}

            scipy.io.savemat(file_name, mat_data_struct)

            if self.getlogger_enable_log:
                self.get_logger().info(f'Data saved into "/Documents/crossing_gui_ros2/" !')

def main(args=None):
    rclpy.init(args=args)

    log_node = LogNode()

    rclpy.spin(log_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)


    # Node will be destroyed upon saving request aswell!
    log_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
    Node for controlling the pedestrian object.
    This node publishes the speed information for the pedestrian object in the GUI.
"""

"""
TODOS:

"""

# ================= Imports: =================
# Python:
import sys
import pathlib
# from turtle import position
# from matplotlib.pyplot import draw
import numpy as np

import time

from threading import Event

# Pygame:
import os
os.environ['SDL_AUDIODRIVER'] = 'dsp' # to avoid alsa errors under WSL
import pygame
from crossing_gui_ros2.simulation_gui_node import RED

# ROS2:
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor # for declaring parameters

from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from rclpy.callback_groups import ReentrantCallbackGroup

from crossing_gui_ros2.simulation_node_functions.create_parameters import DeclareSimulationParameters
from crossing_gui_ros2.simulation_node_functions.vehicle_class_definition import Vehicle
from crossing_gui_ros2.simulation_node_functions.pedestrian_class_definition import Pedestrian

from std_srvs.srv import Empty

# ROS2 message and service definitions:
from crossing_gui_ros2_interfaces.msg import PedestrianSpeedInputMsg, SimulationOutput

# ================= Constants: =================
# Size of the speedchange on button push:
DELTASPEED = 0.2 # m/s

# Colors:
BLACK         = (0,0,0)
WHITE         = (255, 255, 255)
BRIGHTBLUE    = (0,50,255)
DARKTURQUOISE = (3,54,73)
GREEN         = (0,204,0)
RED           = (240,28,28)

# Text:
FONTNAME            = 'freesansbold.ttf'
TEXTCOLOR           = BLACK
BASICFONTSIZE       = 18
TEXTBACKGROUNDCOLOR = WHITE

# Period of publishing the message:
TIMER_PERIOD = 1/15 # seconds (~ 15 fps)

# Asset folder setup:
PKGNAME = "crossing_gui_ros2"
PKGDIR = pathlib.Path(__file__).parent.parent.parent
FONTSDIR = os.path.join(PKGDIR, PKGNAME, 'fonts')
IMGSFOLDER = os.path.join(PKGDIR, PKGNAME, 'imgs')

class Button:
    def __init__(self, screen, x, y, width, height, color, text, text_color):
        self.screen = screen
        self.rect = pygame.Rect(x, y, width, height)
        self.color = color
        self.text = text
        self.text_color = text_color

        self.draw()

    def draw(self):
        pygame.draw.rect(self.screen, self.color, self.rect)
        font = pygame.font.Font(None, 36)
        text = font.render(self.text, True, self.text_color)
        text_rect = text.get_rect(center=self.rect.center)
        self.screen.blit(text, text_rect)

    def is_button_clicked(self, current_mouse_pos):
        return self.rect.collidepoint(current_mouse_pos)

class PedestrianControlNode(Node):
    """
    The Node for publishing the pedestrian control message.

    Args:

    Attributes:
    """

    def __init__(self):
        super().__init__('pedestrian_control_node')

        # ========== Initialize ROS2 things ==========

        # Declare node paramerers:
        parameter_declarator = DeclareSimulationParameters(self.declare_parameter, self.get_parameter)
        parameter_declarator.declare_parameters_for_pedestrian_control_node()

        # Create timer for publishing:
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        # Create publishers:
        self.publisher_pedestrian_speed_input = self.create_publisher(PedestrianSpeedInputMsg, 'simulation/input/pedestrian_speed', 10)

        # Create subscribers:
        self.callback_group1 = ReentrantCallbackGroup()
        self.subscriber_simulation_output = self.create_subscription(SimulationOutput, 'simulation/output', self.callback_simulation_output, 10, callback_group=self.callback_group1)

        # Service client:
        self.callback_group = ReentrantCallbackGroup()
        self.reset_vehicle_client = self.create_client(Empty, '/reset_vehicle', callback_group=self.callback_group)
        # Wait for the service to be available
        while not self.reset_vehicle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.callback_group2 = ReentrantCallbackGroup()
        self.start_log_client = self.create_client(Empty, '/startlog', callback_group=self.callback_group2)
        # Wait for the service to be available
        while not self.start_log_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # Create Pedestrian object to store pedestrian related information:
        self.pedestrian = Pedestrian(initial_x_position=self.get_parameter('pedestrian_initial_x').value,
                                     initial_y_position=self.get_parameter('pedestrian_initial_y').value,
                                     pedestrian_crossing_x = self.get_parameter('pedestrian_crossing_x').value,
                                     road_y = self.get_parameter('road_y').value)
        self.vehicle = Vehicle()

        # Create pedestrian input message
        self.pedestrian_speed_input_message = PedestrianSpeedInputMsg()

        # ========== Initialize PYGAME things: ==========
        pygame.init()
        # os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (100,0) 
        # set window position, but if used, it allways appears on primary display
        self.display_surf = pygame.display.set_mode(size=(600, 400),
                                                    flags=pygame.SHOWN
                                                    # depth=0, # not recommended to pass this argument
                                                    # display=2, # set on which monitor the window should appear
                                                    # vsync=0 # no idea
                                                    )
        pygame.display.set_caption('Pedestrian keyboard control')
        pygame.display.set_icon(pygame.image.load(os.path.join(IMGSFOLDER, "pedestrian_keyboard_control_icon.png")))
        self.basicfont = pygame.font.Font(os.path.join(FONTSDIR, FONTNAME), BASICFONTSIZE)

        # Set background color:
        self.display_surf.fill(WHITE)
        # Draw some informations on the window:
        TEXT_SURF, TEXT_RECT = self.maketext('UP: increment speed',TEXTCOLOR, TEXTBACKGROUNDCOLOR, 10, 10)
        self.display_surf.blit(TEXT_SURF, TEXT_RECT)
        TEXT_SURF, TEXT_RECT = self.maketext('DOWN: decrement speed',TEXTCOLOR, TEXTBACKGROUNDCOLOR, 10, TEXT_RECT.bottom)
        self.display_surf.blit(TEXT_SURF, TEXT_RECT)
        TEXT_SURF, TEXT_RECT = self.maketext('1: 1.0 [m/s]',TEXTCOLOR, TEXTBACKGROUNDCOLOR, 10, TEXT_RECT.bottom)
        self.display_surf.blit(TEXT_SURF, TEXT_RECT)
        TEXT_SURF, TEXT_RECT = self.maketext('2: 1.5 [m/s]',TEXTCOLOR, TEXTBACKGROUNDCOLOR, 10, TEXT_RECT.bottom)
        self.display_surf.blit(TEXT_SURF, TEXT_RECT)
        TEXT_SURF, TEXT_RECT = self.maketext('3: 2.0 [m/s]',TEXTCOLOR, TEXTBACKGROUNDCOLOR, 10, TEXT_RECT.bottom)
        self.display_surf.blit(TEXT_SURF, TEXT_RECT)
        TEXT_SURF, TEXT_RECT = self.maketext('SPACE: 0 [m/s]',TEXTCOLOR, TEXTBACKGROUNDCOLOR, 10, TEXT_RECT.bottom)
        self.display_surf.blit(TEXT_SURF, TEXT_RECT)
        TEXT_SURF, TEXT_RECT = self.maketext('q: 0.0 intention value',TEXTCOLOR, TEXTBACKGROUNDCOLOR, 10, TEXT_RECT.bottom)
        self.display_surf.blit(TEXT_SURF, TEXT_RECT)
        TEXT_SURF, TEXT_RECT = self.maketext('w: 0.5 intention value',TEXTCOLOR, TEXTBACKGROUNDCOLOR, 10, TEXT_RECT.bottom)
        self.display_surf.blit(TEXT_SURF, TEXT_RECT)
        TEXT_SURF, TEXT_RECT = self.maketext('e: 1.0 intention value',TEXTCOLOR, TEXTBACKGROUNDCOLOR, 10, TEXT_RECT.bottom)
        self.display_surf.blit(TEXT_SURF, TEXT_RECT)

        # Create the buttons:
        self.reset_button = Button(self.display_surf, 500 - 200, 10, 80, 40, RED, "RESET", WHITE)

        self.pedmotion_1_button = Button(self.display_surf, 500 - 200, 55, 80, 40, BRIGHTBLUE, "1", WHITE)
        self.pedmotion_2_button = Button(self.display_surf, 500 - 200, 100, 80, 40, BRIGHTBLUE, "2", WHITE)
        self.pedmotion_3_button = Button(self.display_surf, 500 - 200, 145, 80, 40, BRIGHTBLUE, "3", WHITE)
        self.pedmotion_4_button = Button(self.display_surf, 500 - 200, 190, 80, 40, BRIGHTBLUE, "4", WHITE)
        self.pedmotion_5_button = Button(self.display_surf, 500 - 200, 235, 80, 40, BRIGHTBLUE, "5", WHITE)
        self.pedmotion_6_button = Button(self.display_surf, 500 - 200, 280, 80, 40, BRIGHTBLUE, "6", WHITE)

        self.start_log = Button(self.display_surf, 500 - 100, 10, 140, 40, GREEN, "LOG", WHITE)

        self.szenario_in_progress = False
        self.szenario_id = 0

        # ========== Initialize OTHER things: ==========
        # Size of the speedchange on button push:
        self.deltaspeed = DELTASPEED

        self.service_response = None
        self.running_flag = True
        self.pedestrian_y = 0.0

        self.future = None

        self.log_is_running = False

        self.pedmotion_6_state = "not stopped"

        self.szenario_start_time = time.time()
        self.szenario_last_one_second = self.szenario_start_time

        pygame.display.flip()
        # END OF INIT

    def reset_button_callback(self):
        # We call the reset method of the pedestrian and vehicle objects!

        # self.service_done_event.clear()
        event=Event()

        def done_callback(future):
            nonlocal event
            event.set()

        request = Empty.Request()
        self.future = self.reset_vehicle_client.call_async(request)
        self.future.add_done_callback(done_callback)

        event.wait()

        self.pedestrian.reset_to_initial_state()

        self.szenario_in_progress = False
        self.szenario_id = 0

        self.pedmotion_6_state = "not stopped"

    def pedmotion_1_button_callback(self):
        # This is an average walking speed of a young male pedestrian, who is relatively sure 
        # that he wants to cross the road.

        # There are papers which describe the walking speed of pedestrians in different situations.
        self.pedestrian.y_speed = - 1.51
        self.pedestrian.intention = 0.7

    def pedmotion_2_button_callback(self):
        # Until in negotiation area, go normal with the intention that we want to cross the road.
        # In the negotiation area, slow down to wait goal with the intention not to cross.
        # Upon reaching the negotiation area, change mind directly and cross the road.

        elapsed_time = time.time() - self.szenario_start_time
        
        if elapsed_time < 1.0:
            self.pedestrian.y_speed = - 1.51
            self.pedestrian.intention = 0.9
        elif 1.0 <= elapsed_time < 3.0:
            self.pedestrian.y_speed = self.pedestrian.y_speed + 0.04
            self.pedestrian.intention = 0.1
        elif 3.0 <= elapsed_time < 4.0:
            self.pedestrian.y_speed = - 1.51
            self.pedestrian.intention = 0.9
        else:
            self.pedestrian.y_speed = - 1.51
            self.pedestrian.intention = 0.1


    def pedmotion_3_button_callback(self):
        # test case for noise

        # Generate a random walk signal both for velocity and for intention.
        mean = 0.5 # mean value of intention random walk
        std_dev = 0.5 # in 2 sigma 95% of the values are included
        max_jump = 0.4 # maximum jump in one step

        rand_num = np.random.normal(0, std_dev)
        next_value = self.pedestrian.intention + rand_num
        next_value = min(max(next_value, mean - max_jump), mean + max_jump)
        next_value = max(0.0, min(next_value, 1.0))


        self.pedestrian.intention = next_value

        mean = -1 # mean value of intention random walk
        std_dev = 1 # in 2 sigma 95% of the values are included
        max_jump = 0.4 # maximum jump in one step
        
        rand_num = np.random.normal(0, std_dev)
        next_value =  self.pedestrian.y_speed + rand_num
        next_value = min(max(next_value, mean - max_jump), mean + max_jump)
        next_value = max(-2, min(next_value, 0.0))

        self.pedestrian.y_speed = next_value



    def pedmotion_4_button_callback(self):
        # We approach the road withouth the intention to cross it.
        # In the negotiation area we go fast with the intention to cross the road.
        # On the pavement we stop if the vehicle is moving.
        # If the vehicle is not moving, we cross the road.

        # This is the perfect test case to see the effect of the decaying intention value.
        # You can see that when the pedestrian is waiting, the prediction is going down,
        # but still the vehicle waits enough not to hit the pedestrian who just changed his mind.

        # Basically when you want to cross the road but you fear being hit by the car.

        if self.pedestrian.y_distance_to_road >= 5.0:
            self.pedestrian.y_speed = - 1.51
            self.pedestrian.intention = 0.0
        if 1.5 + 1.5 < self.pedestrian.y_distance_to_road < 1.5 + 5.0:
            self.pedestrian.y_speed = - 1.7
            self.pedestrian.intention = 1.0
        if 0.0 < self.pedestrian.y_distance_to_road <= 1.5 + 0.75:
            self.pedestrian.intention = 1.0
            if self.vehicle.x_speed > 1.0:
                self.pedestrian.y_speed = 0.0
            else:
                self.pedestrian.y_speed = - 1.51
        if self.pedestrian.y_distance_to_road <= 1.5:
            self.pedestrian.y_speed = - 1.51
            self.pedestrian.intention = 1.0

    def pedmotion_5_button_callback(self):
        # Slow pedestrian approaching the road with low intention.
        # Than crosses the road with even less intention.

        elapsed_time = time.time() - self.szenario_start_time
        
        if elapsed_time < 2.5:
            self.pedestrian.y_speed = - 1.51
            self.pedestrian.intention = 0.67
        elif 2.5 <= elapsed_time < 4.5:
            self.pedestrian.y_speed = - 0.1
            self.pedestrian.intention = 0.67
        else:
            self.pedestrian.y_speed = - 1.51
            self.pedestrian.intention = 0.67
        

    def pedmotion_6_button_callback(self):
        # Lets create a situation where the pedestrian is coming, the vehicle stops.
        # In the moment the vehicle moves the pedestrian moves too a little bit.
        # Hopefully the vehicle stopps again, now the pedestrian really crosses the road.

        elapsed_time = time.time() - self.szenario_start_time

        if elapsed_time < 3.0:
            self.pedestrian.y_speed = - 1.51
            self.pedestrian.intention = 1.0
        elif 3.0 <= elapsed_time < 5.5:
            self.pedestrian.y_speed = 0.0
            self.pedestrian.intention = 1.0
        elif 5.5 <= elapsed_time < 6.0:
            self.pedestrian.y_speed = - 1.51
            self.pedestrian.intention = 1.0
        elif 6.0 <= elapsed_time < 9:
            self.pedestrian.y_speed = 0.0
            self.pedestrian.intention = 1.0
        else:
            self.pedestrian.y_speed = - 1.51
            self.pedestrian.intention = 1.0

    def start_log_callback(self):
        # We call the reset method of the pedestrian and vehicle objects!
        # Save button position:
        if self.log_is_running:
            self.log_is_running = False
        else:
            self.log_is_running = True

        event=Event()
        def done_callback(future):
            nonlocal event
            event.set()

        request = Empty.Request()
        self.future = self.start_log_client.call_async(request)
        self.future.add_done_callback(done_callback)

        event.wait()

    def on_event(self):
        """Event handling.

        Args:
            ped_control_msg (PedestrianControlMessage): class for message object
                ped_control_msg.msg (Pedestriancontrol): Custom message for pedestrian control
                    ped_control_msg.msg.xspeed (double): Speed in x direction
                    ped_control_msg.msg.yspeed (double): Speed in y direction
                    ped_control_msg.msg.displaymessage (string): Info message
            deltaspeed (double): speedchange upon button press
        """

        deltaspeed = self.deltaspeed

        # Handling mouse events:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running_flag = False
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if self.reset_button.is_button_clicked(event.pos):
                    self.reset_button_callback()
                elif self.start_log.is_button_clicked(event.pos):
                    self.start_log_callback()
                else:
                    for i, button in enumerate([self.pedmotion_1_button,
                                                self.pedmotion_2_button,
                                                self.pedmotion_3_button,
                                                self.pedmotion_4_button,
                                                self.pedmotion_5_button,
                                                self.pedmotion_6_button]):
                        if button.is_button_clicked(event.pos):
                            self.reset_button_callback()
                            self.start_log_callback()
                            self.szenario_in_progress = True
                            self.szenario_id = i + 1
                            self.szenario_start_time = time.time()

        # Load information about all keys:
        keys = pygame.key.get_pressed()

        # Pressed key events:
        if keys[pygame.K_UP] and not keys[pygame.K_DOWN]:
            self.pedestrian.y_speed -= deltaspeed
        elif keys[pygame.K_DOWN] and not keys[pygame.K_UP]:
            self.pedestrian.y_speed += deltaspeed
        elif keys[pygame.K_ESCAPE]:
            self.running_flag = False
        elif keys[pygame.K_1]:
            self.pedestrian.y_speed = -1.0
        elif keys[pygame.K_2]:
            self.pedestrian.y_speed = -1.5
        elif keys[pygame.K_3]:
            self.pedestrian.y_speed = -2.0
        elif keys[pygame.K_SPACE]:
            self.pedestrian.x_speed = 0.0
            self.pedestrian.y_speed = 0.0
        elif keys[pygame.K_q]:
            self.pedestrian.intention = 0.1
        elif keys[pygame.K_w]:
            self.pedestrian.intention = 0.5
        elif keys[pygame.K_e]:
            self.pedestrian.intention = 1.0

        return None

    def maketext(self, text, color, bgcolor, top, left):
        # Create the Surface and Rect objects for some text:
        textsurf = self.basicfont.render(text, True, color, bgcolor)
        textrect = textsurf.get_rect()
        textrect.topleft = (top, left)
        return (textsurf, textrect)

    def callback_simulation_output(self, msg: SimulationOutput) -> None:
        """Callback function for the 'simulation/output' subscriber.
        """

        self.pedestrian.x_position = msg.pedestrian_x
        self.pedestrian.y_position = msg.pedestrian_y

        self.vehicle.x_position = msg.vehicle_x
        self.vehicle.y_position = msg.vehicle_y
        self.vehicle.x_speed = msg.vehicle_x_speed
        self.vehicle.y_speed = msg.vehicle_y_speed

    def publish_pedestrian_input(self) -> None:

        self.pedestrian_speed_input_message.pedestrian_x_speed = 0.0
        self.pedestrian_speed_input_message.pedestrian_y_speed = self.pedestrian.y_speed
        self.pedestrian_speed_input_message.pedestrian_intention      = self.pedestrian.intention
        self.pedestrian_speed_input_message.pedestrian_displaymessage = "Intention: "

        # Publish the message into 'Move_Pedestrian_with_arrows' topic:
        self.publisher_pedestrian_speed_input.publish(self.pedestrian_speed_input_message)

    def timer_callback(self):
        """
        """

        if self.running_flag == True:
            # UNTIL THE NODE IS RUNNING:

            if self.szenario_in_progress:
                if self.szenario_id == 1:
                    self.pedmotion_1_button_callback()
                if self.szenario_id == 2:
                    self.pedmotion_2_button_callback()
                if self.szenario_id == 3:
                    self.pedmotion_3_button_callback()
                if self.szenario_id == 4:
                    self.pedmotion_4_button_callback()
                if self.szenario_id == 5:
                    self.pedmotion_5_button_callback()
                if self.szenario_id == 6:
                    self.pedmotion_6_button_callback()
                    
                # Check if szenario is finished:
                road_top_border = 10 - 1.5 - 1.0 - 1.0 # road - roadwidth - pavement - extra
                vehicle_leave_szenario = 40 + 3 + 5 # crossing + vehicle + extra
                if self.pedestrian.y_position < road_top_border and self.vehicle.x_position > vehicle_leave_szenario:
                    if self.szenario_last_one_second == self.szenario_start_time:
                        self.szenario_last_one_second = time.time()
                    
                    if time.time() - self.szenario_last_one_second > 1.0:
                        self.szenario_in_progress = False
                        self.szenario_id = 0
                        self.start_log_callback()
                        
            
            #self.get_logger().info("pedestrian y position: " + str(self.pedestrian.y_position) + " vehicle x position: " + str(self.vehicle.x_position))



            self.on_event()

            self.publish_pedestrian_input()


            # since i only modify the log button, lets update it only:
            self.start_log.text = "START LOG" if not self.log_is_running else "STOP LOG"
            self.start_log.draw()

            # thread = Thread(target = self.pygameflipdisplay)
            # thread.start()

            pygame.display.update()


            # END OF UNTIL THE NODE IS RUNNING
        else:
            # Stop the node if request is present:
            pygame.quit()
            sys.exit()

    # def pygameflipdisplay(self):
    #     pygame.display.flip()

def main(args=None):
    rclpy.init(args=args)

    pedestrian_control_node = PedestrianControlNode()

    executor = MultiThreadedExecutor()
    rclpy.spin(pedestrian_control_node, executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pedestrian_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
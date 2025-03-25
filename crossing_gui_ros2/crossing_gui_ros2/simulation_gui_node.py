#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# If you use launch files, you have to build the package again!
# Command: colcon build --packages-select crossing_gui_ros2 --symlink-install

# To check for missing dependencies, run following command in the root of your workspace:
# rosdep install -i --from-path src --rosdistro foxy -y

"""TODOS:

"""

# ============ SETUP: ============

# Do you want to generate matlab log-files?
LOG = False

# ============ IMPORTS: ============
# Import modules:
import sys
import time

import os
os.environ['SDL_AUDIODRIVER'] = 'dsp'
import pygame

import math

# Import the file "create_parameters.py" which contains the "DeclareSimulationParameters" class:
# (Build the package first, otherwise it won't work, I have no idea why.)
from crossing_gui_ros2.simulation_node_functions import create_parameters

# Import ROS2:
import rclpy
from rclpy.node import Node

# Import ROS2 message definitions:
from crossing_gui_ros2_interfaces.msg import SimulationOutput, SystemStatesMsg
from crossing_gui_ros2_interfaces.msg import Log


# Import ROS2 service definitions:
from std_srvs.srv import Empty
from sensor_msgs.msg import Joy
from crossing_gui_ros2_interfaces.srv import GetGuiParam
 
# ============ CONSTANTS: ============
# Color constants:
#            R    G    B
GRAY     = (100, 100, 100)
DARKGREY = (50, 50, 50)
NAVYBLUE = ( 60,  60, 100)
WHITE    = (255, 255, 255)
RED      = (255,   0,   0)
GREEN    = (  0, 255,   0)
DARKGREEN= (  0, 100,   0)
KITGREEN = (  0, 150, 130)
BLUE     = (  0,   0, 255)
YELLOW   = (255, 255,   0)
ORANGE   = (255, 128,   0)
PURPLE   = (255,   0, 255)
CYAN     = (  0, 255, 255)
BLACK    = (  0,   0,   0)
BACKGROUNDCOLOR = KITGREEN

# Speed of the simulation:
FPS = 100

# Asset folder setup:
SCRIPTFOLDER = os.path.dirname(os.path.realpath(__file__))
IMGSFOLDER = os.path.join(SCRIPTFOLDER, 'imgs')
FONTSFOLDER = os.path.join(SCRIPTFOLDER, 'fonts')

# Text constants:
FONTNAME = 'freesansbold.ttf'
FONTNAME = 'AzeretMono-Medium.ttf'
TEXTCOLOR = BLACK
BASICFONTSIZE = 20
pygame.font.init()
FONT = pygame.font.Font(os.path.join(FONTSFOLDER, FONTNAME), BASICFONTSIZE)
TEXTBACKGROUNDCOLOR = None # (Background is set to transparent)

# ============ SPRITE OBJECTS: ============
class Object(pygame.sprite.Sprite):
    """Class for moving objects on the screen.
    Currently there are 2 objects, the vehicle and the pedestrian.
    """

    def __init__(self, image_name, image_width = 100, pixel_per_meter_input = 20):
        """Load the image, scale it and position it according to the coordinates.

        Args:
            image_name (_type_): Filename of an image for the object.
            image_width (int, optional): The width of the image in pixel. Defaults to 100.
        """
        self.pixel_per_meter = pixel_per_meter_input
        pygame.sprite.Sprite.__init__(self)
        self.image = pygame.image.load(os.path.join(IMGSFOLDER, image_name)).convert()
        self.image = self.scaling(self.image, image_width)
        self.image.set_colorkey(BLACK)
        self.rect = self.image.get_rect()
        self.xacceleration = 0.0
        self.yacceleration = 0.0
        self.xspeed = 0.0
        self.yspeed = 0.0
        self.x = 5.0
        self.y = 5.0
        self.displaymessage = "-"
        self.intention = 0.5
        self.rect.center = (self.x * self.pixel_per_meter, self.y * self.pixel_per_meter) # in pixel
        self.onborder = False

    def getx(self):             return self.x
    def gety(self):             return self.y
    def getdisplaymessage(self):return self.displaymessage
    def getxspeed(self):        return self.xspeed
    def getyspeed(self):        return self.yspeed
    def getxacceleration(self): return self.xacceleration
    def getyacceleration(self): return self.yacceleration
    def getonborder(self):      return self.onborder
    def getintention(self):     return self.intention

    def setx(self, x):
        self.x = x; self.rect.center = (self.x* self.pixel_per_meter, self.y* self.pixel_per_meter)
    def sety(self, y):
        self.y = y; self.rect.center = (self.x* self.pixel_per_meter, self.y* self.pixel_per_meter)
    def setdisplaymessage(self, displaymessage):  self.displaymessage = displaymessage
    def setxspeed(self, speed): self.xspeed = speed
    def setyspeed(self, speed): self.yspeed = speed
    def setxaccleration(self, acceleration): self.xacceleration = acceleration
    def setyacceleration(self, acceleration): self.yacceleration = acceleration
    def setonborder(self, onborder): self.onborder = onborder
    def setintention(self, intention): self.intention = intention

    # Scale the loaded image (mainly for car because it was too big on screen):
    def scaling(self,image, target_image_width):
        rect = image.get_rect()
        scaling_factor = target_image_width / rect.width
        original_width  = rect.width
        original_height = rect.height
        scaled_width  = original_width  * scaling_factor
        scaled_height = original_height * scaling_factor
        scaled_image = pygame.transform.scale(image, (scaled_width, scaled_height))
        return scaled_image

    # Draw text next to the object:
    def draw_displaymessage_and_intention(self, surface):
        textsurf = FONT.render(self.displaymessage +  str(self.intention), True, TEXTCOLOR, TEXTBACKGROUNDCOLOR)
        textrect = textsurf.get_rect()
        textrect.topleft = (self.rect.bottomright)
        surface.blit(textsurf, textrect)

    def draw_displaymessage(self, surface):
        textsurf = FONT.render("Display: " + self.displaymessage, True, TEXTCOLOR, TEXTBACKGROUNDCOLOR)
        textrect = textsurf.get_rect()
        textrect.topleft = (self.rect.bottomright)
        surface.blit(textsurf, textrect)

    def draw_intention(self, surface):
        textsurf = FONT.render("Intention: " + str(self.intention), True, TEXTCOLOR, TEXTBACKGROUNDCOLOR)
        textrect = textsurf.get_rect()
        textrect.topleft = (self.rect.bottomright)
        surface.blit(textsurf, textrect)

# ============ MAIN PROGRAM CLASS: ============

class SimulationGuiNode(Node):
    """Main program class.

    Args:
        Node (_type_): Native ROS2 object
    """

    def __init__(self):
        super().__init__('simulation_gui_node')
        self.running = True

        # Create parameters:
        parameter_declarator = create_parameters.DeclareSimulationParameters(self.declare_parameter, self.get_parameter)
        parameter_declarator.declare_parameters_for_simulation_gui_node()
        

        # Get parameters from ROS2 parameter server:
        self.getlogger_enable_gui = self.get_parameter('getlogger_enable_gui').value

        # Initialize fps:
        timer_period = 1/FPS # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize window:
        self.pixel_per_meter = self.get_parameter('pixel_per_meter').value
        self.window_width = self.meter2pixel(self.get_parameter('window_width').value)
        self.window_height = self.meter2pixel(self.get_parameter('window_height').value)
        self.size = ( self.window_width, self.window_height)


        # Initialize pygame:
        pygame.init()
        self.display_surface = pygame.display.set_mode(self.size)
        pygame.display.set_caption('Pedestrian Crossing Simulation')
        pygame.display.set_icon(pygame.image.load(os.path.join(IMGSFOLDER, "gui_icon.png")))

        # Subscribers:
        self.is_joy_control = False
        self.subscriber_simulation_output = self.create_subscription(SimulationOutput, 'simulation/output', self.callback_simulation_output, 10)
        self.subscriber_vehicle_state_prediction = self.create_subscription(SystemStatesMsg, 'negotiation_node/out/vehicle_state_prediction', self.callback_vehicle_state_prediction, 10)

        # Publishers:
        self.publisher_log = self.create_publisher(Log, 'simulation/output/log', 10)

        # Service clients:
        self.service_server_getguiparam = self.create_service(srv_type=GetGuiParam, srv_name="/gui/get_gui_param", callback=self.service_callback_getguiparam)

        # Sprite group: (otherwise sprites will be deleted)
        self.all_objects = pygame.sprite.Group()

        # Pedestrian setup:
        self.pedestrian = Object("walkingperson.png", self.meter2pixel(self.get_parameter("pedestrian_size").value), self.pixel_per_meter)
        self.pedestrian.setx(self.get_parameter('pedestrian_initial_x').value) # [m]
        self.pedestrian.sety(self.get_parameter('pedestrian_initial_y').value) # [m]
        self.pedestrian.setdisplaymessage('stopping')
        self.pedestrian.setxspeed(0.0) # [m/s]
        self.pedestrian.setyspeed(0.0) # [m/s]
        self.all_objects.add(self.pedestrian)
        self.pedestrian.setonborder(False)

        # Vehicle setup:
        self.vehicle = Object('vehicle.png', self.meter2pixel(self.get_parameter("vehicle_length").value), self.pixel_per_meter)
        self.vehicle.setx(self.get_parameter('vehicle_initial_x').value) # [m]
        self.vehicle.sety(self.get_parameter('vehicle_initial_y').value) # [m]
        self.vehicle.setdisplaymessage('driving')
        self.vehicle.setxspeed(self.get_parameter("vehicle_initial_x_speed").value) # [m/s]
        self.vehicle.setyspeed(self.get_parameter("vehicle_initial_y_speed").value) # [m/s]
        self.all_objects.add(self.vehicle)

        # Road rectangle setup:
        self.road_color = DARKGREY
        self.road_lane_width = 3.0
        # Create Rect object and set virtual attributes:
        self.road_rect = pygame.Rect(1,1,1,1) # dummy values
        self.road_rect.height = self.meter2pixel(self.road_lane_width)
        self.road_rect.width = self.window_width
        self.road_rect.centery = self.meter2pixel(self.get_parameter('road_y').value)
        self.road_rect.left = 0

        self.system_predicted_states = []

        self.system_predicted_states_drawn = []

    def meter2pixel(self, meter):
        return meter*self.pixel_per_meter

    def pixel2meter(self, pixel):
        return pixel/self.pixel_per_meter

    def callback_simulation_output(self, msg: SimulationOutput) -> None:
        """
        Save incoming messages from the simulation node.
        """
        self.pedestrian.setx(msg.pedestrian_x)
        self.pedestrian.sety(msg.pedestrian_y)
        self.pedestrian.setxspeed(msg.pedestrian_x_speed)
        self.pedestrian.setyspeed(msg.pedestrian_y_speed)
        self.pedestrian.setdisplaymessage(msg.pedestrian_displaymessage)
        self.pedestrian.setintention(msg.pedestrian_intention)

        self.vehicle.setx(msg.vehicle_x)
        self.vehicle.sety(msg.vehicle_y)
        self.vehicle.setxspeed(msg.vehicle_x_speed)
        self.vehicle.setyspeed(msg.vehicle_y_speed)
        self.vehicle.setxaccleration(msg.vehicle_x_acceleration)
        self.vehicle.setyacceleration(msg.vehicle_y_acceleration)
        self.vehicle.setdisplaymessage(msg.vehicle_displaymessage)
        
    def callback_vehicle_state_prediction(self, msg: SystemStatesMsg) -> None:

        self.system_predicted_states.append(msg)

        if len(self.system_predicted_states) > 5:
            self.system_predicted_states = self.system_predicted_states[-5:]

    def service_callback_getguiparam(self, request, response):
        """
        Callback function for the 'getguiparam' service.

        Retrieves the value of a specified parameter and returns it in the response.

        Definition:
        string param_name
        ---
        float64 param_value

        Args:
            request: The service request containing the parameter name.
            response: The service response to be filled with the parameter value.

        Returns:
            The response object with the parameter value set.
        """
        param_name = request.param_name
        param_value = self.get_parameter(param_name).value
        response.param_value = param_value
        return response

    def timer_callback(self):
            """
            Callback function that is called periodically by a timer.
            It handles pygame events, updates the display, and checks for program termination.

            Args:
                None

            Returns:
                None
            """

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYUP:
                    if event.key == pygame.K_ESCAPE:
                        self.running = False

            self.draw_everything() # later I should separate the static and the dynamically changing things.

            pygame.display.flip()

            if self.running == False:
                self.get_logger().info("Shutdown")
                pygame.quit()
                sys.exit()


    def draw_everything(self):

        def draw_background():
            self.display_surface.fill(BACKGROUNDCOLOR)

        def draw_road():
            pygame.draw.rect(self.display_surface, self.road_color, self.road_rect)

        def draw_pawement():
            pawement_color = GRAY
            pawement_rect = pygame.Rect(1,1,1,1)
            pawement_rect.width = self.road_rect.width
            pawement_rect.height = self.meter2pixel(1.5)
            pawement_rect.centery = self.road_rect.bottom + pawement_rect.height / 2
            pygame.draw.rect(self.display_surface, pawement_color, pawement_rect)

            pawement_rect.centery = self.road_rect.top - pawement_rect.height / 2
            pygame.draw.rect(self.display_surface, pawement_color, pawement_rect)

        def draw_grid():
            # Draw 10 meter grid:
            # Draw grid lines
            grid_size = self.meter2pixel(10.0)  # Size of each grid cell in meters
            grid_color = (200, 200, 200)  # Color of the grid lines

            # Calculate the number of grid cells in each direction
            num_cells_x = int(self.window_width / grid_size)
            num_cells_y = int(self.window_height / grid_size)

            # Calculate the coordinates of the pedestrian crossing location
            crossing_x = int(self.meter2pixel(self.get_parameter('pedestrian_crossing_x').value))
            crossing_y = int(self.meter2pixel(self.get_parameter('road_y').value))

            # Draw vertical grid lines
            # Draw line at pedestrian_crossing_x position
            pygame.draw.line(self.display_surface, grid_color, (crossing_x, 0), (crossing_x, self.window_height))

            # Draw lines every 10 meters to the left of pedestrian_crossing_x
            for i in range(1, int(crossing_x / grid_size) + 1):
                x = crossing_x - i * grid_size
                pygame.draw.line(self.display_surface, grid_color, (x, 0), (x, self.window_height))

            # Draw lines every 10 meters to the right of pedestrian_crossing_x
            for i in range(1, int((self.window_width - crossing_x) / grid_size) + 1):
                x = crossing_x + i * grid_size
                pygame.draw.line(self.display_surface, grid_color, (x, 0), (x, self.window_height))

            # Draw horizontal grid lines
            # Draw line at road_y position
            pygame.draw.line(self.display_surface, grid_color, (0, crossing_y), (self.window_width, crossing_y))

            # Draw lines every 10 meters above road_y
            for i in range(1, int(crossing_y / grid_size) + 1):
                y = crossing_y - i * grid_size
                pygame.draw.line(self.display_surface, grid_color, (0, y), (self.window_width, y))

            # Draw lines every 10 meters below road_y
            for i in range(1, int((self.window_height - crossing_y) / grid_size) + 1):
                y = crossing_y + i * grid_size
                pygame.draw.line(self.display_surface, grid_color, (0, y), (self.window_width, y))

        def draw_circle():
            crossing_radius = 5
            crossing_x = int(self.meter2pixel(self.get_parameter('pedestrian_crossing_x').value))
            crossing_y = int(self.meter2pixel(self.get_parameter('road_y').value))
            pygame.draw.circle(self.display_surface, (255, 0, 0), (crossing_x, crossing_y), crossing_radius)

        def draw_stateinfo():
            # Draw vehicle information on the bottom of the screen:
            switcher = {
                0: "Vehicle information:",
                1: f"{'a_x':<4} = " + str( round(self.vehicle.getxacceleration(),2) ) + " [m/s^2]",
                2: f"{'v_x':<4} = " + str( round(self.vehicle.getxspeed(),2) ) + " [m/s]",
                3: f"{'x'  :<4} = " + str( round(self.vehicle.getx(),2) ) + " [m]",
                4: f"{'y'  :<4} = " + str( round(self.vehicle.gety(),2) ) + " [m]",
                5: "displaymessage = \"" + self.vehicle.getdisplaymessage() + "\""
            }
            row_count = len(switcher)
            for i in range(row_count):
                textsurf = FONT.render(switcher.get(i,"---"), True, TEXTCOLOR, None)
                textposition = textsurf.get_rect()
                textheight = textposition.height
                textposition.topleft = (10, self.window_height - 10 - row_count*textheight + i*textheight)
                self.display_surface.blit(textsurf, textposition)

        def draw_system_state_prediction():
            """
            Draw the predicted states of the vehicle and pedestrian.
            """

            prev_x_vehicle = None
            prev_y_vehicle = None
            prev_x_pedestrian = None
            prev_y_pedestrian = None


            if len(self.system_predicted_states) > 0:
            

            

                for system_state in self.system_predicted_states[-1].system_states:
                    x_vehicle = system_state.x_veh
                    y_vehicle = system_state.y_veh
                    x_pedestrian = system_state.x_ped
                    y_pedestrian = system_state.y_ped

                    # Convert back from ped cros coord sys to gui coord sys
                    x_vehicle = self.meter2pixel(x_vehicle + self.get_parameter('pedestrian_crossing_x').value)
                    y_vehicle = self.meter2pixel(self.get_parameter('road_y').value - y_vehicle)
                    x_pedestrian = self.meter2pixel(x_pedestrian + self.get_parameter('pedestrian_crossing_x').value)
                    y_pedestrian = self.meter2pixel(self.get_parameter('road_y').value - y_pedestrian)

                    x_size = 5

                    # Draw 'x' at the vehicle position
                    pygame.draw.line(self.display_surface, RED, (x_vehicle - x_size, y_vehicle - x_size), (x_vehicle + x_size, y_vehicle + x_size), 2)
                    pygame.draw.line(self.display_surface, RED, (x_vehicle - x_size, y_vehicle + x_size), (x_vehicle + x_size, y_vehicle - x_size), 2)

                    # Draw 'x' at the pedestrian position
                    pygame.draw.line(self.display_surface, BLACK, (x_pedestrian - x_size, y_pedestrian - x_size), (x_pedestrian + x_size, y_pedestrian + x_size), 2)
                    pygame.draw.line(self.display_surface, BLACK, (x_pedestrian - x_size, y_pedestrian + x_size), (x_pedestrian + x_size, y_pedestrian - x_size), 2)

                    # Connect the x-es with lines
                    if prev_x_vehicle is not None and prev_y_vehicle is not None:
                        pygame.draw.line(self.display_surface, RED, (prev_x_vehicle, prev_y_vehicle), (x_vehicle, y_vehicle), 2)
                    if prev_x_pedestrian is not None and prev_y_pedestrian is not None:
                        pygame.draw.line(self.display_surface, BLACK, (prev_x_pedestrian, prev_y_pedestrian), (x_pedestrian, y_pedestrian), 2)

                    prev_x_vehicle = x_vehicle
                    prev_y_vehicle = y_vehicle
                    prev_x_pedestrian = x_pedestrian
                    prev_y_pedestrian = y_pedestrian

                self.system_predicted_states_drawn.append(self.system_predicted_states[-1])
                if len(self.system_predicted_states_drawn) > 50:
                    self.system_predicted_states_drawn = self.system_predicted_states_drawn[-50:]

                    if  all(state == self.system_predicted_states_drawn[-1] for state in self.system_predicted_states_drawn):
                        self.system_predicted_states = []

            else:
                pass

        def draw_vertical_distance(): # currently not used
            pygame.draw.line(self.display_surface, BLACK, (self.meter2pixel(self.pedestrian.getx()), self.meter2pixel(self.pedestrian.gety())),
                                (self.meter2pixel(self.pedestrian.getx()),self.meter2pixel(self.vehicle.gety())), 3)
            vdist = (self.pedestrian.gety() - self.vehicle.gety())
            vdist = abs(vdist)
            textsurf = FONT.render("{:.2f} [m]".format(vdist), True, TEXTCOLOR, WHITE)
            textrect = textsurf.get_rect()
            textrect.midleft = (self.meter2pixel(self.pedestrian.getx()),
                                self.meter2pixel(self.vehicle.gety() + (self.pedestrian.gety() - self.vehicle.gety())/2))
            self.display_surface.blit(textsurf, textrect)

        def draw_horizontal_distance(): # currently not used
            pygame.draw.line(self.display_surface, BLACK, (self.meter2pixel(self.vehicle.getx()), self.meter2pixel(self.vehicle.gety())),
                                (self.meter2pixel(self.pedestrian.getx()),self.meter2pixel(self.vehicle.gety())), 3)
            hdist = (self.vehicle.getx() - self.pedestrian.getx())
            hdist = abs(hdist)
            textsurf = FONT.render("{:.2f} [m]".format(hdist), True, TEXTCOLOR, WHITE)
            textrect = textsurf.get_rect()
            textrect.midbottom = (self.meter2pixel(self.vehicle.getx() + (self.pedestrian.getx() - self.vehicle.getx())/2),
                                self.meter2pixel(self.vehicle.gety()))
            self.display_surface.blit(textsurf, textrect)

        def draw_absolute_distance(): # currently not used
            pygame.draw.line(self.display_surface, BLACK, (self.meter2pixel(self.pedestrian.getx()), self.meter2pixel(self.pedestrian.gety())),
                            (self.meter2pixel(self.vehicle.getx()),self.meter2pixel(self.vehicle.gety())), 3)
            adist = math.sqrt((self.pedestrian.getx() - self.vehicle.getx())**2 +
                                (self.pedestrian.gety() - self.vehicle.gety())**2)
            textsurf = FONT.render("{:.2f} [m]".format(adist), True, TEXTCOLOR, WHITE)
            textrect = textsurf.get_rect()
            textrect.midtop = ((self.meter2pixel(self.pedestrian.getx()) + self.meter2pixel(self.vehicle.getx()))/2,
                            (self.meter2pixel(self.pedestrian.gety()) + self.meter2pixel(self.vehicle.gety()))/2)
            self.display_surface.blit(textsurf, textrect)

        def draw_personalspaces():
            pygame.draw.circle(self.display_surface, BLACK, (self.meter2pixel(self.pedestrian.getx()), self.meter2pixel(self.pedestrian.gety())), self.meter2pixel(self.get_parameter("pedestrian_personal_space_radius").value), 1)
            # pygame.draw.circle(self.display_surface, BLACK, (self.meter2pixel(self.vehicle.getx()), self.meter2pixel(self.vehicle.gety())), self.meter2pixel(self.get_parameter("vehicle_collision_radius").value), 1)
            pygame.draw.circle(self.display_surface, RED, (self.meter2pixel(self.vehicle.getx()), self.meter2pixel(self.vehicle.gety())), self.meter2pixel(1.5), 1)
            pygame.draw.circle(self.display_surface, RED, (self.meter2pixel(self.vehicle.getx()-1.5), self.meter2pixel(self.vehicle.gety())), self.meter2pixel(1.5), 1)
            pygame.draw.circle(self.display_surface, RED, (self.meter2pixel(self.vehicle.getx()+1.5), self.meter2pixel(self.vehicle.gety())), self.meter2pixel(1.5), 1)



        def draw_intention_and_displaymessage():
            self.pedestrian.draw_intention(self.display_surface)
            self.vehicle.draw_displaymessage(self.display_surface)

        def draw_vehicleacceleration():
            self.all_objects.draw(self.display_surface)

        def draw_globalcoordsys():
            # Horizontal arrow
            pygame.draw.line(self.display_surface, BLACK, (10, 10), (50, 10), 2)
            pygame.draw.polygon(self.display_surface, BLACK, [(50, 10), (40, 5), (40, 15)])
            # Add 'x' text
            textsurf = FONT.render("x_gui", True, BLACK)
            textrect = textsurf.get_rect()
            textrect.midleft = (55, 10)
            self.display_surface.blit(textsurf, textrect)

            # Vertical arrow
            pygame.draw.line(self.display_surface, BLACK, (10, 10), (10, 50), 2)
            pygame.draw.polygon(self.display_surface, BLACK, [(10, 50), (5, 40), (15, 40)])
            # Add 'y' text
            textsurf = FONT.render("y_gui", True, BLACK)
            textrect = textsurf.get_rect()
            textrect.midtop = (35, 55)
            self.display_surface.blit(textsurf, textrect)

        def draw_pedestriancrossing_coordsys():
            # Horizontal arrow
            pygame.draw.line(self.display_surface, BLACK, (self.meter2pixel(self.get_parameter("pedestrian_crossing_x").value), self.meter2pixel(self.get_parameter("road_y").value)), (self.meter2pixel(self.get_parameter("pedestrian_crossing_x").value) + 40, self.meter2pixel(self.get_parameter("road_y").value)), 2)
            pygame.draw.polygon(self.display_surface, BLACK, [(self.meter2pixel(self.get_parameter("pedestrian_crossing_x").value) + 40, self.meter2pixel(self.get_parameter("road_y").value)), (self.meter2pixel(self.get_parameter("pedestrian_crossing_x").value) + 30, self.meter2pixel(self.get_parameter("road_y").value) - 5), (self.meter2pixel(self.get_parameter("pedestrian_crossing_x").value) + 30, self.meter2pixel(self.get_parameter("road_y").value) + 5)])
            # Add 'x' text
            textsurf = FONT.render("x_pedcross", True, BLACK)
            textrect = textsurf.get_rect()
            textrect.midleft = (self.meter2pixel(self.get_parameter("pedestrian_crossing_x").value) + 45, self.meter2pixel(self.get_parameter("road_y").value))
            self.display_surface.blit(textsurf, textrect)

            # Vertical arrow
            pygame.draw.line(self.display_surface, BLACK, (self.meter2pixel(self.get_parameter("pedestrian_crossing_x").value), self.meter2pixel(self.get_parameter("road_y").value)), (self.meter2pixel(self.get_parameter("pedestrian_crossing_x").value), self.meter2pixel(self.get_parameter("road_y").value) - 40), 2)
            pygame.draw.polygon(self.display_surface, BLACK, [(self.meter2pixel(self.get_parameter("pedestrian_crossing_x").value), self.meter2pixel(self.get_parameter("road_y").value) - 40), (self.meter2pixel(self.get_parameter("pedestrian_crossing_x").value) - 5, self.meter2pixel(self.get_parameter("road_y").value) - 30), (self.meter2pixel(self.get_parameter("pedestrian_crossing_x").value) + 5, self.meter2pixel(self.get_parameter("road_y").value) - 30)])
            # Add 'y' text
            textsurf = FONT.render("y_pedcross", True, BLACK)
            textrect = textsurf.get_rect()
            textrect.midbottom = (self.meter2pixel(self.get_parameter("pedestrian_crossing_x").value), self.meter2pixel(self.get_parameter("road_y").value) - 50)
            self.display_surface.blit(textsurf, textrect)

        # vehicle_in_negotiation_area = -30.0 < self.vehicle.state[0] < 0.0 # 30 meters before the pedestrian crossing.
        # pedestrian_in_negotiation_area = - 6.5 < self.pedestrian.state[2] < 1.5 # 5 meters below road until road top.

        def draw_vehicle_negotiation_area():
            # Calculate the coordinates of the rectangle
            left = self.meter2pixel(self.get_parameter("pedestrian_crossing_x").value) - self.meter2pixel(30)
            right = self.meter2pixel(self.get_parameter("pedestrian_crossing_x").value)
            top = self.meter2pixel(self.get_parameter("road_y").value - self.road_lane_width/2)
            bottom = self.meter2pixel(self.get_parameter("road_y").value + self.road_lane_width/2)

            # Check if the vehicle position is within the rectangle
            vehicle_x = self.meter2pixel(self.vehicle.getx())
            vehicle_y = self.meter2pixel(self.vehicle.gety())
            if left <= vehicle_x <= right and top <= vehicle_y <= bottom:
                # Draw the rectangle with a different color
                color = GREEN
            else:
                # Draw the rectangle with the original color
                color = RED
            
            # Draw the rectangle
            pygame.draw.rect(self.display_surface, color, (left, top, right - left, bottom - top), 1) # (rect: left, top, width, height)
        def draw_pedestrian_negotiation_area():
            # Calculate the coordinates of the rectangle
            left = self.meter2pixel(self.get_parameter("pedestrian_crossing_x").value - 2)
            right = self.meter2pixel(self.get_parameter("pedestrian_crossing_x").value + 2)
            top = self.meter2pixel(self.get_parameter("road_y").value - self.road_lane_width/2 )
            bottom = self.meter2pixel(self.get_parameter("road_y").value + self.road_lane_width/2 + 5)
            # Check if the pedestrian position is within the rectangle
            pedestrian_x = self.meter2pixel(self.pedestrian.getx())
            pedestrian_y = self.meter2pixel(self.pedestrian.gety())
            if left <= pedestrian_x <= right and top <= pedestrian_y <= bottom:
                # Draw the rectangle
                color = GREEN
            else: 
                color = RED

            pygame.draw.rect(self.display_surface, color, (left, top, right - left, bottom - top), 1) # (rect: left, top, width, height)

        def draw_pedestrian_position_goals():
            # Draw the pedestrian's position goals
            pygame.draw.circle(self.display_surface, BLACK, (self.meter2pixel(self.get_parameter("pedestrian_crossing_x").value), self.meter2pixel(self.get_parameter("road_y").value - 1.5 - 0.75)), self.meter2pixel(0.75), 1)
            pygame.draw.circle(self.display_surface, BLACK, (self.meter2pixel(self.get_parameter("pedestrian_crossing_x").value), self.meter2pixel(self.get_parameter("road_y").value + 1.5 + 0.75)), self.meter2pixel(0.75), 1)

        # The order is important!
        draw_background()
        draw_road()
        draw_pawement()
        draw_grid()
        draw_circle()
        draw_stateinfo()
        draw_personalspaces()
        draw_intention_and_displaymessage()
        draw_vehicleacceleration()
        draw_globalcoordsys()
        draw_pedestriancrossing_coordsys()
        draw_vehicle_negotiation_area()
        draw_pedestrian_negotiation_area()
        draw_pedestrian_position_goals()
        draw_system_state_prediction()

def main(args=None):
    rclpy.init(args=args)

    simulation_gui_node = SimulationGuiNode()

    rclpy.spin(simulation_gui_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simulation_gui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
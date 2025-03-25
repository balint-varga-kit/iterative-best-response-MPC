#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This function contains all parameter descriptors and declarations for the application.

Workflow:
    1.: Import this file
    2.: Use the member functions to declare the parameters in the nodes init function.
    3.: Use the self.get_param("name").value function to use the parameters in the node.
    4.: To use custom values instead of the default values, initialize the parameters in the .yaml config file.

Some help:

Parameter types:
    # These types correspond to the value that is set in the ParameterValue message.
    # Default value, which implies this is not a valid parameter.
    uint8 PARAMETER_NOT_SET=0
    uint8 PARAMETER_BOOL=1
    uint8 PARAMETER_INTEGER=2
    uint8 PARAMETER_DOUBLE=3
    uint8 PARAMETER_STRING=4
    uint8 PARAMETER_BYTE_ARRAY=5
    uint8 PARAMETER_BOOL_ARRAY=6
    uint8 PARAMETER_INTEGER_ARRAY=7
    uint8 PARAMETER_DOUBLE_ARRAY=8
    uint8 PARAMETER_STRING_ARRAY=9

- dynamic_typing=False means that the type of the parameter cannot change in runtime.
- In the declare_parameter function, the value is the default value of the parameter.
- Boolean parameters does not have range attribute in the descriptor.

- The type, set in the descriptor of the parameter is not checked. If you set the parameter to a double, 
but you assign it a type as integer, than still the parameter will be declared as a double.
"""

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import FloatingPointRange

from rclpy.exceptions import ParameterNotDeclaredException

class DeclareSimulationParameters():
    """Create the parameters of the simulation/gui node.
    First the parameter descriptors are created, after that, the parameters are declared.
    """


    def __init__(self, declare_parameter_function, get_parameter_function):
        self.declare_parameter_function = declare_parameter_function
        self.get_parameter_function = get_parameter_function

    def declare_window_width(self):
        window_width_range              = IntegerRange()
        window_width_range.from_value   = 1
        window_width_range.to_value     = 10000
        window_width_range.step         = 1
        descriptor_window_width = ParameterDescriptor(
            name                    = 'window_width',
            type                    = 3,
            description             = 'Simulation window width in pixel.',
            additional_constraints  = 'None',
            read_only               = True,
            dynamic_typing          = False,
            integer_range           = (window_width_range,)
        )
        self.declare_parameter_function(name        = 'window_width',
                                        value       = 100.0,
                                        descriptor  = descriptor_window_width)

    def declare_window_height(self):
        window_height_range             = IntegerRange()
        window_height_range.from_value  = 1
        window_height_range.to_value    = 10000
        window_height_range.step        = 1
        descriptor_window_height = ParameterDescriptor(
            name                    = 'window_height',
            type                    = 3,
            description             = 'Simulation window height in pixel.',
            additional_constraints  = 'None',
            read_only               = True,
            dynamic_typing          = False,
            integer_range           = (window_height_range,)
        )
        self.declare_parameter_function(name        = 'window_height',
                                        value       = 30.0,
                                        descriptor  = descriptor_window_height)

    def declare_pixel_per_meter(self):
        pixel_per_meter_range               = IntegerRange()
        pixel_per_meter_range.from_value    = 1
        pixel_per_meter_range.to_value      = 1000
        pixel_per_meter_range.step          = 1
        descriptor_pixel_per_meter = ParameterDescriptor(
            name                    = 'pixel_per_meter',
            type                    = 3,
            description             = 'This amount of pixel on the screen is representing 1 meter in real world.',
            additional_constraints  = 'None',
            read_only               = True,
            dynamic_typing          = False,
            integer_range           = (pixel_per_meter_range,)
        )
        self.declare_parameter_function(name    = 'pixel_per_meter',
                                        value        = 30.0,
                                        descriptor   = descriptor_pixel_per_meter)

    def declare_getlogger_enable_gui(self):
        descriptor_getlogger_enable_gui = ParameterDescriptor(
            name                    = 'getlogger_enable_gui',
            type                    = 1,
            description             = 'Enables the get_logger() function to post infos in the terminal.',
            additional_constraints  = 'boolean: True or False',
            read_only               = True,
            dynamic_typing          = False,
            )
        self.declare_parameter_function(name = 'getlogger_enable_gui',
                                        value        = True,
                                        descriptor   = descriptor_getlogger_enable_gui)

    def declare_getlogger_enable_vehcontrol(self):
        descriptor_getlogger_enable_vehcontrol = ParameterDescriptor(
            name                    = 'getlogger_enable_vehcontrol',
            type                    = 1,
            description             = 'Enables the get_logger() function to post infos in the terminal.',
            additional_constraints  = 'boolean: True or False',
            read_only               = True,
            dynamic_typing          = False,
            )
        self.declare_parameter_function(name = 'getlogger_enable_vehcontrol',
                                        value        = True,
                                        descriptor   = descriptor_getlogger_enable_vehcontrol)

    def declare_pedestrian_crossing_x(self):
        try:
            window_width    = self.get_parameter_function('window_width').value
            pixel_per_meter = self.get_parameter_function('pixel_per_meter').value
        except ParameterNotDeclaredException as e:
            descriptor_pedestrian_crossing_x = ParameterDescriptor(
            name                    = 'pedestrian_crossing_x',
            type                    = 3,
            description             = 'Initial distance of the vehicle from the pedestrian crossing.',
            additional_constraints  = 'If too large, it could couse problems with the size of the window.',
            read_only               = True,
            dynamic_typing          = False,
            )
        else:
            pedestrian_crossing_x_range              = FloatingPointRange()
            pedestrian_crossing_x_range.from_value   = 0.0
            pedestrian_crossing_x_range.to_value     = window_width
            pedestrian_crossing_x_range.step         = 0.1
            descriptor_pedestrian_crossing_x = ParameterDescriptor(
                name                    = 'pedestrian_crossing_x',
                type                    = 3,
                description             = 'Initial distance of the vehicle from the pedestrian crossing.',
                additional_constraints  = 'If too large, it could couse problems with the size of the window.',
                read_only               = True,
                dynamic_typing          = False,
                floating_point_range    = (pedestrian_crossing_x_range,)
                )
        finally:
            self.declare_parameter_function(name        = 'pedestrian_crossing_x',
                                            value       = 80.0,
                                            descriptor  = descriptor_pedestrian_crossing_x)

    def declare_vehicle_collision_radius(self):
        vehicle_collision_radius_range              = FloatingPointRange()
        vehicle_collision_radius_range.from_value   = 0.0
        vehicle_collision_radius_range.to_value     = 5.0
        vehicle_collision_radius_range.step         = 0.001
        descriptor_vehicle_collision_radius = ParameterDescriptor(
            name                    = 'vehicle_collision_radius',
            type                    = 3,
            description             ='The radius of the area which represents the vehicle in the simulation.',
            additional_constraints  = '-',
            read_only               = True,
            dynamic_typing          = False,
            floating_point_range    = (vehicle_collision_radius_range,)
            )
        self.declare_parameter_function(name = 'vehicle_collision_radius',
                                value = 3.0,
                                descriptor = descriptor_vehicle_collision_radius)

    def declare_pedestrian_personal_space_radius(self):
        collision_radius_range              = FloatingPointRange()
        collision_radius_range.from_value   = 0.0
        collision_radius_range.to_value     = 15.0
        collision_radius_range.step         = 0.001
        descriptor_pedestrian_personal_space_radius = ParameterDescriptor(
            name                    = 'pedestrian_personal_space_radius',
            type                    = 3,
            description             ='The vehicle should keep this distance to the pedestrian.',
            additional_constraints  = '-',
            read_only               = True,
            dynamic_typing          = False,
            floating_point_range    = (collision_radius_range,)
            )
        self.declare_parameter_function(name = 'pedestrian_personal_space_radius',
                                value = 15.0,
                                descriptor = descriptor_pedestrian_personal_space_radius)

    def declare_vehicle_initial_x(self):
        try:
            window_width    = self.get_parameter_function('window_width').value
            pixel_per_meter = self.get_parameter_function('pixel_per_meter').value
        except ParameterNotDeclaredException as e:
            descriptor_vehicle_initial_x    = ParameterDescriptor(
                name                        = 'vehicle_initial_x',
                type                        = 3,
                description                 = 'The initial x coordinate of the vehicle in meter',
                additional_constraints      = 'Should be less than the pedestrian x position.',
                read_only                   = True,
                dynamic_typing              = False,
                )
        else:
            vehicle_initial_x_range             = FloatingPointRange()
            vehicle_initial_x_range.from_value  = 0.0
            vehicle_initial_x_range.to_value    = window_width
            vehicle_initial_x_range.step        = 0.01
            descriptor_vehicle_initial_x = ParameterDescriptor(
                name                    = 'vehicle_initial_x',
                type                    = 3,
                description             = 'The initial x coordinate of the vehicle in meter',
                additional_constraints  = 'Should be less than the pedestrian x position.',
                read_only               = True,
                dynamic_typing          = False,
                floating_point_range    = (vehicle_initial_x_range,)
                )
        finally:
            self.declare_parameter_function(name        = 'vehicle_initial_x',
                                            value       = 1.0,
                                            descriptor  = descriptor_vehicle_initial_x)

    def declare_vehicle_initial_y(self):
        try:
            window_height   = self.get_parameter_function('window_height').value
            pixel_per_meter = self.get_parameter_function('pixel_per_meter').value
        except ParameterNotDeclaredException as e:
            descriptor_vehicle_initial_y    = ParameterDescriptor(
                name                        = 'vehicle_initial_y',
                type                        = 3,
                description                 = 'The initial y coordinate of the vehicle in meter',
                additional_constraints      = 'Should be less than the window height.',
                read_only                   = True,
                dynamic_typing              = False,
                )
        else:
            vehicle_initial_y_range             = FloatingPointRange()
            vehicle_initial_y_range.from_value  = 0.0
            vehicle_initial_y_range.to_value    = window_height
            vehicle_initial_y_range.step        = 0.01
            descriptor_vehicle_initial_y = ParameterDescriptor(
                name                    = 'vehicle_initial_y',
                type                    = 3,
                description             = 'The initial y coordinate of the vehicle in meter',
                additional_constraints  = 'Should be less than the window height.',
                read_only               = True,
                dynamic_typing          = False,
                floating_point_range    = (vehicle_initial_y_range,)
                )
        finally:
            self.declare_parameter_function(name        = 'vehicle_initial_y',
                                            value       = 1.0,
                                            descriptor  = descriptor_vehicle_initial_y)

    def declare_vehicle_initial_x_speed(self):
        vehicle_initial_x_speed_range              = FloatingPointRange()
        vehicle_initial_x_speed_range.from_value   = 0.0
        vehicle_initial_x_speed_range.to_value     = 60.0 # [m/s] (= 216 km/h)
        vehicle_initial_x_speed_range.step         = 0.01
        descriptor_vehicle_initial_x_speed = ParameterDescriptor(
            name                    = 'vehicle_initial_x_speed',
            type                    = 3,
            description             = 'The initial x speed of the vehicle in meter/second',
            additional_constraints  = '-',
            read_only               = True,
            dynamic_typing          = False,
            floating_point_range    = (vehicle_initial_x_speed_range,)
            )
        self.declare_parameter_function(name        = 'vehicle_initial_x_speed',
                                        value       = 8.0, # [m/s] (= 28.8 km/h)
                                        descriptor  = descriptor_vehicle_initial_x_speed)

    def declare_vehicle_initial_y_speed(self):
        vehicle_initial_y_speed_range              = FloatingPointRange()
        vehicle_initial_y_speed_range.from_value   = 0.0
        vehicle_initial_y_speed_range.to_value     = 60.0 # [m/s] (= 216 km/h)
        vehicle_initial_y_speed_range.step         = 0.01
        descriptor_vehicle_initial_y_speed = ParameterDescriptor(
            name                    = 'vehicle_initial_y_speed',
            type                    = 3,
            description             = 'The initial y speed of the vehicle in meter/second',
            additional_constraints  = '-',
            read_only               = True,
            dynamic_typing          = False,
            floating_point_range    = (vehicle_initial_y_speed_range,)
            )
        self.declare_parameter_function(name        = 'vehicle_initial_y_speed',
                                        value       = 0.0, # [m/s] (= 28.8 km/h)
                                        descriptor  = descriptor_vehicle_initial_y_speed)

    def declare_vehicle_length(self):
        vehicle_length_range            = FloatingPointRange()
        vehicle_length_range.from_value = 0.0
        vehicle_length_range.to_value   = 50.0 # [m]
        vehicle_length_range.step       = 0.01
        descriptor_vehicle_length = ParameterDescriptor(
            name                    = 'vehicle_length',
            type                    = 3,
            description             = 'The length of the vehicle in meter.',
            additional_constraints  = '-',
            read_only               = True,
            dynamic_typing          = False,
            floating_point_range    = (vehicle_length_range,)
            )
        self.declare_parameter_function(name        = 'vehicle_length',
                                        value       = 5.0, # [m]
                                        descriptor  = descriptor_vehicle_length)
        
    def declare_pedestrian_size(self):
        pedestrian_size_range            = FloatingPointRange()
        pedestrian_size_range.from_value = 0.0
        pedestrian_size_range.to_value   = 10.0 # [m]
        pedestrian_size_range.step       = 0.01
        descriptor_pedestrian_sizeh = ParameterDescriptor(
            name                    = 'pedestrian_size',
            type                    = 3,
            description             = 'The width of the pedestrian in top view in meter.',
            additional_constraints  = '-',
            read_only               = True,
            dynamic_typing          = False,
            floating_point_range    = (pedestrian_size_range,)
            )
        self.declare_parameter_function(name        = 'pedestrian_size',
                                        value       = 5.0, # [m]
                                        descriptor  = descriptor_pedestrian_sizeh)


    def declare_pedestrian_initial_x(self):
        pedestrian_initial_x_range             = FloatingPointRange()
        pedestrian_initial_x_range.from_value  = 0.0
        pedestrian_initial_x_range.to_value    = 1000.0
        pedestrian_initial_x_range.step        = 0.001
        descriptor_pedestrian_initial_x = ParameterDescriptor(
            name                    = 'pedestrian_initial_x',
            type                    = 3,
            description             = 'Starting x position of the pedestrian.',
            additional_constraints  = 'None',
            read_only               = True,
            dynamic_typing          = False,
            floating_point_range    = (pedestrian_initial_x_range,)
        )
        self.declare_parameter_function(name        = 'pedestrian_initial_x',
                                        value       = 35.0,
                                        descriptor  = descriptor_pedestrian_initial_x)

    def declare_pedestrian_initial_y(self):
        pedestrian_initial_y_range             = FloatingPointRange()
        pedestrian_initial_y_range.from_value  = 0.0
        pedestrian_initial_y_range.to_value    = 1000.0
        pedestrian_initial_y_range.step        = 0.001
        descriptor_pedestrian_initial_y = ParameterDescriptor(
            name                    = 'pedestrian_initial_y',
            type                    = 3,
            description             = 'Starting y position of the pedestrian.',
            additional_constraints  = 'None',
            read_only               = True,
            dynamic_typing          = False,
            floating_point_range    = (pedestrian_initial_y_range,)
        )
        self.declare_parameter_function(name        = 'pedestrian_initial_y',
                                        value       = 35.0,
                                        descriptor  = descriptor_pedestrian_initial_y)

    def declare_road_y(self):
        try:
            window_height   = self.get_parameter_function('window_height').value
            pixel_per_meter = self.get_parameter_function('pixel_per_meter').value
        except ParameterNotDeclaredException as e:
            descriptor_road_y = ParameterDescriptor(
            name                    = 'road_y',
            type                    = 3,
            description             = 'The Y position of the road centerline in meter.',
            additional_constraints  = 'None',
            read_only               = True,
            dynamic_typing          = False,
        )
        else:
            road_y_range            = FloatingPointRange()
            road_y_range.from_value = 0.0
            road_y_range.to_value   = window_height
            road_y_range.step       = 0.001
            descriptor_road_y = ParameterDescriptor(
                name                    = 'road_y',
                type                    = 3,
                description             = 'The Y position of the road centerline in meter.',
                additional_constraints  = 'None',
                read_only               = True,
                dynamic_typing          = False,
                floating_point_range    = (road_y_range,)
            )
        finally:
            self.declare_parameter_function(name        = 'road_y',
                                            value       = 2.0,
                                            descriptor  = descriptor_road_y)
            
    def declare_getlogger_enable_pedcontrol(self):
        descriptor_getlogger_enable_pedcontrol = ParameterDescriptor(
            name                    = 'getlogger_enable_pedcontrol',
            type                    = 1,
            description             = 'Enables the get_logger() function to post infos in the terminal.',
            additional_constraints  = 'boolean: True or False',
            read_only               = True,
            dynamic_typing          = False,
            )
        self.declare_parameter_function(name = 'getlogger_enable_pedcontrol',
                                        value        = True,
                                        descriptor   = descriptor_getlogger_enable_pedcontrol)


    def declare_parameters_for_simulation_gui_node(self):
        """ 
            The parameters of this node, copied from YAML file:

            window_width: 1800 # [pixel]
            window_height: 720 # [pixel]
            pixel_per_meter: 20 # [pixel]
            getlogger_enable_gui: False # False: no logging on console
            pedestrian_crossing_x: 60.0 # [m]
            collision_radius: 3.0 # [m]
            vehicle_initial_x: 10.0 # [m]
            ped_initial_pos_y: 30.0 # [m]
            road_y: 15.0 # [m]
            vehicle_length: 5.0 # [m]
            vehicle_initial_x_speed: 0.0 # [m/s]
        """

        self.declare_window_width()
        self.declare_window_height()
        self.declare_pixel_per_meter()

        self.declare_getlogger_enable_gui()

        self.declare_pedestrian_crossing_x()
        self.declare_road_y()


        self.declare_vehicle_initial_x()
        self.declare_vehicle_initial_y()
        self.declare_vehicle_initial_x_speed()
        self.declare_vehicle_initial_y_speed()
        self.declare_vehicle_length()
        self.declare_vehicle_collision_radius()

        self.declare_pedestrian_initial_x()
        self.declare_pedestrian_initial_y()
        self.declare_pedestrian_personal_space_radius()
        self.declare_pedestrian_size()


    def declare_parameters_for_simulation_node(self):

        self.declare_vehicle_initial_x()
        self.declare_vehicle_initial_y()
        self.declare_vehicle_initial_x_speed()
        self.declare_vehicle_initial_y_speed()
        self.declare_vehicle_length() # only declared, not used yet

        self.declare_pedestrian_initial_x()
        self.declare_pedestrian_initial_y()

        self.declare_pedestrian_crossing_x() # only declared, not used yet

        self.declare_road_y() # only declared, not used yet

    def declare_parameters_for_vehicle_control_node(self):

        self.declare_vehicle_initial_x()
        self.declare_vehicle_initial_y()
        self.declare_vehicle_initial_x_speed()
        self.declare_vehicle_initial_y_speed()
        self.declare_vehicle_collision_radius()

        self.declare_pedestrian_crossing_x()
        self.declare_road_y()
        self.declare_getlogger_enable_vehcontrol()

        self.declare_pedestrian_personal_space_radius()

    def declare_parameters_for_pedestrian_control_node(self):
        self.declare_pedestrian_initial_x()
        self.declare_pedestrian_initial_y()
        self.declare_getlogger_enable_pedcontrol()

        self.declare_pedestrian_crossing_x()
        self.declare_road_y()

    def declare_parameters_for_log_node(self):
        pass


if __name__ == "__main__":
    print("You have run this script directly. This file is meant to be imported!")

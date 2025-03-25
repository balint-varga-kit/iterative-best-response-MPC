

import numpy as np
class Pedestrian:
    """A class representing a pedestrian in a simulation.

    Attributes:
        initial_x_position (float): The initial x position of the pedestrian.
        initial_y_position (float): The initial y position of the pedestrian.
        initial_x_speed (float): The initial x speed of the pedestrian.
        initial_y_speed (float): The initial y speed of the pedestrian.
        initial_x_acceleration (float): The initial x acceleration of the pedestrian.
        initial_y_acceleration (float): The initial y acceleration of the pedestrian.
        initial_intention (float): The initial intention of the pedestrian.
        initial_displaymessage (str): The initial display message of the pedestrian.
    """

    def __init__(self, 
                initial_x_position     = 0.0, 
                initial_x_speed        = 0.0, 
                initial_x_acceleration = 0.0, 
                initial_y_position     = -15.0, 
                initial_y_speed        = 0.0, 
                initial_y_acceleration = 0.0, 
                initial_intention      = 0.5,
                initial_displaymessage = "pedestrian current state default",
                TIMER_PERIOD = 0.1):
        # Initial state: (immutable)
        self._initial_x_position        = initial_x_position
        self._initial_x_speed           = initial_x_speed
        self._initial_x_acceleration    = initial_x_acceleration
        self._initial_y_position        = initial_y_position
        self._initial_y_speed           = initial_y_speed
        self._initial_y_acceleration    = initial_y_acceleration
        self._initial_intention         = initial_intention
        self._initial_displaymessage    = initial_displaymessage

        self._initial_x_speed_smooth_decay = initial_x_speed
        self._initial_y_speed_smooth_decay = initial_y_speed

        self.TIMER_PERIOD = TIMER_PERIOD

        # I want an intention value, which starts dropping if the pedestrian is not moving.
        # If the pedestrian is moving, it should jump back to the normal intention, which is provided as input.
        self.intention_decay = initial_intention

        # Current state: (mutable)
        self.x_position     = initial_x_position
        self.x_speed        = initial_x_speed
        self.x_acceleration = initial_x_acceleration
        self.y_position     = initial_y_position
        self.y_speed        = initial_y_speed
        self.y_acceleration = initial_y_acceleration
        self.intention      = initial_intention
        self.displaymessage = initial_displaymessage

        self.x_speed_smooth_decay = initial_x_speed
        self.y_speed_smooth_decay = initial_y_speed

        # Target state: (mutable)
        self.x_target               = initial_x_position
        self.x_speed_target         = initial_x_speed
        self.x_acceleration_target  = initial_x_acceleration
        self.y_target               = initial_y_position
        self.y_speed_target         = initial_y_speed
        self.y_acceleration_target  = initial_y_acceleration

    def update_pedestrian_state(self, pedestrian_new_state):
        """
        Update the whole pedestrian state

        Args:
            pedestrian_new_state (_type_): _description_
        """

        self.x_position     = pedestrian_new_state[0]
        self.x_speed        = pedestrian_new_state[1]
        self.y_position     = pedestrian_new_state[2]
        self.y_speed        = pedestrian_new_state[3]
        self.intention      = pedestrian_new_state[4]
        self.displaymessage = pedestrian_new_state[5]

    @property
    def state(self):
        """
        Only the state which is being used for the optimization.

        Returns:
            state (numpy.array): position and velocity of the pedestrian
        """
        state = np.array([self.x_position,
                            self.x_speed,
                            self.y_position,
                            self.y_speed,])
        return state
    
    @state.setter
    def state(self, state):
        self.x_position = state[0]
        self.x_speed    = state[1]
        self.y_position = state[2]
        self.y_speed    = state[3]

    @property
    def target_state(self):
        """
        Get the desired position and velocity of the pedestrian.

        Returns:
            target_state (numpy.array): Desired position and velocity of the pedestrian
        """
        target_state = np.array([self.x_target,
                                self.x_speed_target,
                                self.y_target,
                                self.y_speed_target])
        return target_state
    
    @target_state.setter
    def target_state(self, target_state):
        """
        Set the desired positon and velocity of the pedestrian.

        Args:
            target_state (numpy.array): Desired position and velocity
        """
        self.x_target       = target_state[0]
        self.x_speed_target = target_state[1]
        self.y_target       = target_state[2]
        self.y_speed_target = target_state[3]


    # def update_pedestrian_speed_smooth_decay(self):

    #     # Update the smooth decay of the pedestrian's speed:

    #     slope = 1 / ((self.intention + 0.1) * self.TIMER_PERIOD*60)
    #     y_speed = self.y_speed
    #     y_speed_smooth_decay = self.y_speed_smooth_decay

    #     # Upwards jumping, downwards smooth decay:
    #     if y_speed > y_speed_smooth_decay:
    #         y_speed_smooth_decay = y_speed
    #     elif y_speed < y_speed_smooth_decay:
    #         y_speed_smooth_decay = max(y_speed, y_speed_smooth_decay - slope)

    #     self.y_speed_smooth_decay = y_speed_smooth_decay


    @property
    def initial_x_position(self):
        return self._initial_x_position

    @property
    def initial_y_position(self):
        return self._initial_y_position

    @property
    def initial_x_speed(self):
        return self._initial_x_speed

    @property
    def initial_y_speed(self):
        return self._initial_y_speed

    @property
    def initial_x_acceleration(self):
        return self._initial_x_acceleration

    @property
    def initial_y_acceleration(self):
        return self._initial_y_acceleration
    
    @property
    def initial_intention(self):
        return self._initial_intention

    @property
    def initial_displaymessage(self):
        return self._initial_displaymessage

    def reset_to_initial_state(self):
        """Reset the pedestrian to its initial state.

        This method resets the pedestrian's position, speed, acceleration, and display message
        to their initial values.
        """
        self.x_position     = self._initial_x_position
        self.y_position     = self._initial_y_position
        self.x_speed        = self._initial_x_speed
        self.y_speed        = self._initial_y_speed
        self.x_acceleration = self._initial_x_acceleration
        self.y_acceleration = self._initial_y_acceleration
        self.intention      = self._initial_intention
        self.displaymessage = self._initial_displaymessage

    def __repr__(self):
            """Return a string representation of the pedestrian object.

            Returns:
                str: A string representation of the pedestrian object.
            """
            return (
                f"Pedestrian("
                f"initial_x_position={self._initial_x_position}, "
                f"initial_y_position={self._initial_y_position}, "
                f"initial_x_speed={self._initial_x_speed}, "
                f"initial_y_speed={self._initial_y_speed}, "
                f"initial_x_acceleration={self._initial_x_acceleration}, "
                f"initial_y_acceleration={self._initial_y_acceleration}, "
                f"initial_intention={self._initial_intention}, "
                f"initial_displaymessage='{self._initial_displaymessage}'"
                f")"
            )

    def __str__(self):
        """Return a string representation of the pedestrian object.

        Returns:
            str: A string representation of the pedestrian object.
        """
        return (
            f"Pedestrian("
            f"x_position={self.x_position}, "
            f"y_position={self.y_position}, "
            f"x_speed={self.x_speed}, "
            f"y_speed={self.y_speed}, "
            f"x_acceleration={self.x_acceleration}, "
            f"y_acceleration={self.y_acceleration}, "
            f"intention={self.intention}, "
            f"displaymessage='{self.displaymessage}'"
            f")"
        )

    def __eq__(self, other):
        """Check if the current pedestrian object is equal to another object.

        Args:
            other (object): The object to compare with.

        Returns:
            bool: True if the current states are equal, False otherwise.
        """
        if not isinstance(other, Pedestrian):
            return False
        return (
            self.x_position == other.x_position
            and self.y_position == other.y_position
            and self.x_speed == other.x_speed
            and self.y_speed == other.y_speed
            and self.x_acceleration == other.x_acceleration
            and self.y_acceleration == other.y_acceleration
            and self.intention == other.intention
            and self.displaymessage == other.displaymessage
        )

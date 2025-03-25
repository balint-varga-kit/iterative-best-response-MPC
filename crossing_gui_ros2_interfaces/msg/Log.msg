# Message definition.
# Created for the communication from the GUI to the logger.

# Time of publishing the message in seconds since epoch:
float64 t_stamp

# Position of the pedestrian in meter:
float64 pedestrian_x
float64 pedestrian_y

# Position of the vehicle in meter:
float64 vehicle_x
float64 vehicle_y

# Absolute distance between pedestrian and vehicle:
float64 distance

# Velocity of the pedestrian in meter/second:
float64 pedestrian_x_speed
float64 pedestrian_y_speed

# Velocity of the vehicle in meter/second:
float64 vehicle_x_speed
float64 vehicle_y_speed
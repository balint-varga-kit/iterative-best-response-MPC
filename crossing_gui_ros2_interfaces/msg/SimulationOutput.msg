# Message definition.
# Simulation node output message.
# Sent back to the controllers, to the decision node and to the GUI for visualization.

# === PEDESTRIAN INFOS ===
# x-position of the pedestrian in meter:
float64 pedestrian_x

# y-position of the pedestrian in meter:PedestrianInputMsg
float64 pedestrian_y

# x-speed of the pedestrian in meter/seconds:
float64 pedestrian_x_speed

# y-speed of the pedestrian in meter/seconds:
float64 pedestrian_y_speed

# x-acceleration of the pedestrian in meter/seconds^2:
float64 pedestrian_x_acceleration

# y-acceleration of the pedestrian in meter/seconds^2:
float64 pedestrian_y_acceleration

# The displayed message on the pedestrian:
string pedestrian_displaymessage

# Pedestrian intention:
float64 pedestrian_intention

# Pedestrian intention decay:
float64 pedestrian_intention_decay

# Pedestrian y speed smooth decay:
float64 pedestrian_y_speed_smooth_decay

# === VEHICLE INFOS ===
# x-position of the vehicle in meter:
float64 vehicle_x

# y-position of the vehicle in meter:
float64 vehicle_y

# x-speed of the vehicle in meter/seconds:
float64 vehicle_x_speed

# y-speed of the vehicle in meter/seconds:
float64 vehicle_y_speed

# x-acceleration of the vehicle in meter/seconds^2:
float64 vehicle_x_acceleration

# y-acceleration of the vehicle in meter/seconds^2:
float64 vehicle_y_acceleration

# The displayed message on the vehicle:
string vehicle_displaymessage
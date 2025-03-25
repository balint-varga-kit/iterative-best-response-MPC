




class SimpleCruiseController:
    def __init__(self):
        self.Kp = 0.5 # 
        self.min_input = -6.0 # m/s^2
        self.max_input = 1.4 # m/s^2
        self.vehicle_display_message = "Driving"

    def calculate_input(self, target_velocity, current_velocity):
        error = target_velocity - current_velocity
        vehicle_input = self.Kp * error
        vehicle_input = max(self.min_input, min(vehicle_input, self.max_input))
        
        vehicle_display_message = self.vehicle_display_message
        return vehicle_input, vehicle_display_message
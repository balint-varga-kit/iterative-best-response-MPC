is_ROS_node = 1
# if is_ROS_node:
#     from .vanilla_mpc import vanilla_mpc
# else:
#     from vanilla_mpc import vanilla_mpc



class RuleBasedController:
    def __init__(self):

        self.low_ped_speed_threshold_discounting_para = 0.47 # CAHNGE???

        # For distinguishing different cases
        self.ped_low_yspeed_para = 0.64
        self.ped_high_yspeed_para = 1.51
        self.ped_low_intention_para = 0.25
        self.ped_high_intention_para = 0.68


        # Parameters for calculating the acceleration
        self.veh_dec_gain_para = 9.91
        self.veh_max_dec_para = 4.7  # maximal deceleration
        self.veh_max_speed_para = 7.8

        self.veh_normal_acc = 1.91

        # If the vehicle is out of safe stop distance, it could keep a safe speed and doesn't need to decelerate too much
        self.veh_safe_stop_distance_para = 4.78
        self.veh_safe_speed_para = 1.18

        # If the pedestrian is out of distance from road threshold, the vehicle with high speed could go through in spite of high intention
        self.veh_high_speed_para = 6.46

        self.discounted_ped_intention_var = 1.0
        

        self.ped_veh_distance_var = 0.0
        self.ped_distance_from_road_var = 0.0

        self.safe_distance_y = 3.0
        self.ped_distance_from_road_threshold_para = self.safe_distance_y
        
        
    def veh_crossing(self, ped, veh):
        acc_start = (veh.x_speed_target-veh.x_speed) * 1 
        
        return acc_start if acc_start < self.veh_normal_acc else self.veh_normal_acc
    
        
    

    def veh_stopping(self, ped, veh):
        
        is_ped_far_from_veh = self.ped_veh_distance_var >= self.veh_safe_stop_distance_para

        if is_ped_far_from_veh and veh.x_speed <= self.veh_safe_speed_para:
            return 0.0
        else:
            return max(- self.veh_max_dec_para,- self.veh_dec_gain_para * veh.x_speed / (1 + self.ped_veh_distance_var))



    def can_veh_safe_cross(self, ped, veh, logger):
        
        
        x_where_discount_starts = 30.0
        
        
        is_pedestrian_close_to_road = abs(self.ped_distance_from_road_var) <= self.ped_distance_from_road_threshold_para
        # logger().info("is_pedestrian_close_to_road value: {}".format(is_pedestrian_close_to_road)) GOOD
        
        # TODO: remove high speed param
        # it is unnecessary!!!
        
        # Explanation:
        # if vehicle is closer than human AND 
        # vehicle faster than threshold AND 
        # NOT pedestrian close to road
        # OR
        if (self.ped_veh_distance_var < self.ped_distance_from_road_var and veh.x_speed >= self.veh_high_speed_para \
                and not is_pedestrian_close_to_road) \
                or abs(veh.x_position - ped.x_position)>x_where_discount_starts:  
            return True
        return False



    def eval(self, ped, veh, logger):
        
        
        # self.x_position     = initial_x_position
        # self.x_speed        = initial_x_speed
        # self.x_acceleration = initial_x_acceleration
        # self.y_position     = initial_y_position
        # self.y_speed        = initial_y_speed
        # self.y_acceleration = initial_y_acceleration
        # self.intention      = initial_intention
        # self.displaymessage = initial_displaymessage
        
        road_width = 3.0
        

        is_ped_outside_road = abs(veh.y_position - ped.y_position) > road_width/2 # half-WIDTH of the road, TODO: WIVW checking 

        is_veh_passed = veh.x_position > ped.x_position
        print("is_veh_passed:", is_veh_passed)
        is_ped_passed = False
        if -1*(ped.y_position - veh.y_position) < 0.5 and is_ped_outside_road:
            is_ped_passed = True
        print("is_PED_passed:", is_ped_passed)
        # self.intention_discounting(ped, veh)
        self.ped_veh_distance_var = abs(ped.x_position - veh.x_position)
        logger().info("value: {}".format(self.ped_veh_distance_var))
        self.ped_distance_from_road_var = - (ped.y_position - veh.y_position)

        is_pedestrian_close_to_road = abs(self.ped_distance_from_road_var) <= self.ped_distance_from_road_threshold_para
        print("is ped close to road:", is_pedestrian_close_to_road)
        print("can veh safe cross:", self.can_veh_safe_cross(ped, veh, logger))
        print("Ped speed: ", ped.y_speed)
        print("Ped Intention value: ", ped.intention)
        
        
        
        veh_acc_cmd_ms2 = 0.0
        
        
        if is_veh_passed or is_ped_passed:
            veh_acc_cmd_ms2 = self.veh_crossing(ped, veh)
            logger().info("0")
        elif not self.can_veh_safe_cross(ped, veh, logger):
            print("is ped outside road:", is_ped_outside_road)
            if not is_ped_outside_road:
                veh_acc_cmd_ms2 = self.veh_stopping(ped, veh)
                print("1")
                logger().info("1")
                
            elif is_pedestrian_close_to_road and abs(ped.y_speed) > 0.1:
                veh_acc_cmd_ms2 = self.veh_stopping(ped, veh)
                print("2")
                logger().info("2")
                
            elif abs(ped.y_speed) > self.ped_high_yspeed_para or \
                    ped.intention > self.ped_high_intention_para:
                veh_acc_cmd_ms2 = self.veh_stopping(ped, veh)
                print("3")
                logger().info("3")
                
            elif self.ped_low_yspeed_para <= abs(ped.y_speed) <= self.ped_high_yspeed_para and \
                    self.ped_low_intention_para <= ped.intention <= self.ped_high_intention_para:
                veh_acc_cmd_ms2 = self.veh_stopping(ped, veh)
                print("4")
                logger().info("4")
                
            else:
                veh_acc_cmd_ms2 = self.veh_crossing(ped, veh)
                print("5")
                logger().info("5")
        else:
            veh_acc_cmd_ms2 = self.veh_crossing(ped, veh)
            logger().info("99")

        pub_text = "I'm stopping" if veh_acc_cmd_ms2 <= 0.0 else "I'm driving"


        
        return veh_acc_cmd_ms2, pub_text

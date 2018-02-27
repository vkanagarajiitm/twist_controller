
import rospy                                    # will need access to parameters and system time
from pid import PID                             # PID controller provided
from yaw_controller import YawController        # Yaw controller provided
from lowpass import LowPassFilter

# original Udacity constants
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        rospy.loginfo('TwistController: Start init')
        self.sampling_rate = kwargs["sampling_rate"]
        self.decel_limit = kwargs["decel_limit"]
        self.accel_limit = kwargs["accel_limit"]
        # brake_deadband is the interval in which the brake would be ignored
        # the car would just be allowed to slow by itself/coast to a slower speed
        self.brake_deadband = kwargs["brake_deadband"]
        self.vehicle_mass = kwargs["vehicle_mass"]
        self.fuel_capacity = kwargs["fuel_capacity"]
        self.wheel_radius = kwargs["wheel_radius"]
        # bunch of parameters to use for the Yaw (steering) controller
        self.wheel_base = kwargs["wheel_base"]
        self.steer_ratio = kwargs["steer_ratio"]
        self.max_lat_accel = kwargs["max_lat_accel"]
        self.max_steer_angle = kwargs["max_steer_angle"]


        self.delta_t = 1/self.sampling_rate
        self.brake_torque_const = (self.vehicle_mass + self.fuel_capacity \
            * GAS_DENSITY) * self.wheel_radius

        
        self.low_pass_filter = LowPassFilter(0.2, self.delta_t)



        # Initialise speed PID, with tuning parameters
        # Will use this PID for the speed control
        self.velocity_pid = PID(0.9,0.0005, 0.07,
                                  self.decel_limit, self.accel_limit)        


        # Initialise Yaw controller - this gives steering values using
        # vehicle attributes/bicycle model
        # Need to have some minimum speed before steering is applied
        self.yaw_controller = YawController(wheel_base=self.wheel_base,
                                            steer_ratio=self.steer_ratio,
                                            min_speed=5.0,
                                            max_lat_accel=self.max_lat_accel,
                                            max_steer_angle=self.max_steer_angle)

        rospy.loginfo('TwistController: Complete init')
        rospy.loginfo('TwistController: Steer ratio = ' + str(self.steer_ratio))

    def control(self, target_linear,target_angular,
                current_linear):

        throttle, brake, steering = 0.0, 0.0, 0.0
        
        error_velocity = target_linear - current_linear
        
        # use velocity controller compute desired accelaration
        acc_desired = self.velocity_pid.step(error_velocity, self.delta_t)
        
        if acc_desired > 0.0:
            throttle= self.low_pass_filter.filt(acc_desired)
            brake = 0.0
        else:
            throttle = 0.0
            brake = 0.0            
            if abs(acc_desired) > self.brake_deadband:
                # don't bother braking unless over the deadband level
                # make sure we do not brake to hard
                if abs(acc_desired) > abs(self.decel_limit):
                    brake = abs(self.decel_limit) * self.brake_torque_const
                else:
                    brake = abs(acc_desired) * self.brake_torque_const


        # steering - yaw controller takes desired linear, desired angular, current linear as params
        #steering = target_angular * self.steer_ratio
        steering = self.yaw_controller.get_steering(target_linear,
                                                    target_angular,
                                                    current_linear)
        
        if abs(steering) <> 0.0:
            #rospy.loginfo('TwistController: Steering = ' + str(steering))
            rospy.loginfo('Veer: Steering = ' + str(steering) + ', required = ' + str(target_angular))

        return throttle, brake, steering

    def reset(self):
        self.velocity_pid.reset()

#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty 
import numpy as np
import time
import os
from initial_params import get_initial_params

# --- Constants ---
SIM_DURATION = 5.0          # Total duration to publish torques (seconds)
TORQUE_APPLY_DURATION = 1.0 # Duration for which user-defined torques are applied (seconds)
LOOP_RATE_HZ = 100          # Frequency of publishing loop (Hz)

class TorquePublisher:
    def __init__(self):
        rospy.init_node('torque_commander_publisher', anonymous=True)

        self.effort_pub = rospy.Publisher('/ur5/simple_effort_controller/command', Float64MultiArray, queue_size=10)
        
        # Gazebo Physics Services (only unpause is needed, as launch starts it paused)
        # rospy.wait_for_service('/gazebo/pause_physics') # Removed as pause is handled by launch
        rospy.wait_for_service('/gazebo/unpause_physics')
        # self.pause_physics_srv = rospy.ServiceProxy('/gazebo/pause_physics', Empty) # Removed
        self.unpause_physics_srv = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

        self.rate = rospy.Rate(LOOP_RATE_HZ)
        rospy.loginfo("Torque Publisher Initialized. Ready to send commands.")

    def run_publisher(self, user_defined_torques):
        """
        Runs the loop to publish torques to Gazebo.
        """
        rospy.loginfo("Setting up Gazebo environment and publishing torques...")
        
        # --- REMOVIDO: Bloco de pause de física explícito ---
        # try:
        #     self.pause_physics_srv()
        #     rospy.loginfo("Gazebo physics paused.")
        # except rospy.ServiceException as e:
        #     rospy.logwarn(f"Failed to pause Gazebo physics: {e}. Assuming already paused or not necessary.")
        
        # Give Gazebo a moment to settle if it was just launched and paused
        rospy.sleep(0.5) 
        
        # 2. Unpause Gazebo physics
        try:
            self.unpause_physics_srv()
            rospy.loginfo("Gazebo physics unpaused. Starting simulation and publishing torques.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to unpause Gazebo physics: {e}. Cannot publish torques.")
            return False

        # --- Torque Profile Function ---
        def get_torques_for_time(t):
            if t < TORQUE_APPLY_DURATION:
                return user_defined_torques 
            else:
                return np.zeros(6, dtype=np.float64) 

        # --- Main Publishing Loop ---
        start_time = rospy.Time.now().to_sec()
        current_time_from_start = 0.0

        rospy.loginfo("Starting torque publishing loop...")
        while not rospy.is_shutdown() and current_time_from_start < SIM_DURATION:
            current_time_from_start = rospy.Time.now().to_sec() - start_time
            
            tau_to_publish = get_torques_for_time(current_time_from_start)
            
            effort_msg = Float64MultiArray()
            effort_msg.data = tau_to_publish.tolist() 
            self.effort_pub.publish(effort_msg)

            self.rate.sleep()
        
        # --- Stop Robot after simulation duration ---
        rospy.loginfo(f"Simulation duration ({SIM_DURATION}s) reached. Sending final zero torque.")
        final_stop_torques = Float64MultiArray()
        final_stop_torques.data = np.zeros(6, dtype=np.float64).tolist()
        self.effort_pub.publish(final_stop_torques) 

        rospy.loginfo("Torque publishing loop concluded.")
        return True

if __name__ == '__main__':
    try:
        # initial_params_dict = get_initial_params() 
        # user_defined_torques = np.array(initial_params_dict['tau_apply'], dtype=np.float64) 
        user_defined_torques = np.array([[-100, 0, 0, 0, 0, 0],
                                         [100, 0, 0.0, 0, 0, 0], 
                                         [0, 0, -30, 0, 0, 0], 
                                         [0, 0, 30, 0, 0, 0], 
                                         [10.0, -20.0, -5.0, 1.0, 0.5, 0.1],
                                         [100.0, 100.0, 100.0, 0.8, -0.3, 0.0], 
                                         [50.0, -80.0, 30.0, 1.0, -0.5, 0.8]], dtype=np.float64)

        for i, torque in enumerate(user_defined_torques):
            publisher_node = TorquePublisher()
            rospy.loginfo(f"\n Valores de torque publicado para as juntas:\n{torque} Nm")
            publisher_node.run_publisher(torque)
            
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt received. Shutting down publisher.")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {e}")
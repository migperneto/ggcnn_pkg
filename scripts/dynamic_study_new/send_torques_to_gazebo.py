#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty 
import numpy as np
import time
import os
from initial_params import get_initial_params

# This script is udes to send torques to the UR5 robot in gazebo

# --- Constants ---
SIM_DURATION = 1.0 
TORQUE_APPLY_DURATION = 1.0 
LOOP_RATE_HZ = 100 

class GazeboTorqueCommander:
    def __init__(self):
        rospy.init_node('gazebo_torque_commander', anonymous=True)

        self.joint_names = [ 
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self.joint_name_to_index = {name: i for i, name in enumerate(self.joint_names)}
        
        self.effort_pub = rospy.Publisher('/ur5/simple_effort_controller/command', Float64MultiArray, queue_size=10)
        self.joint_state_sub = rospy.Subscriber('/ur5/joint_states', JointState, self.joint_state_callback)

        rospy.wait_for_service('/gazebo/pause_physics')
        rospy.wait_for_service('/gazebo/unpause_physics')
        self.pause_physics_srv = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics_srv = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

        self.q_gazebo = np.zeros(6)      
        self.q_dot_gazebo = np.zeros(6)  
        self.last_joint_state_msg = None 

        self.time_history = []
        self.q_history = []
        self.q_dot_history = []
        self.applied_torque_history = [] 

        self.rate = rospy.Rate(LOOP_RATE_HZ)
        rospy.loginfo("Gazebo Torque Commander Initialized. Waiting for Gazebo...")

    def joint_state_callback(self, msg):
        # --- CORREÇÃO AQUI: Inicializar temp_q e temp_dq antes do loop ---
        # Garantir que temp_q e temp_dq sempre existam, mesmo que não haja dados no msg.position/velocity
        temp_q = np.zeros(6)
        temp_dq = np.zeros(6) # <<<< Esta era a linha que faltava inicializar

        has_positions = len(msg.position) > 0
        has_velocities = len(msg.velocity) > 0
        
        for msg_idx, name_in_msg in enumerate(msg.name):
            if name_in_msg in self.joint_name_to_index: 
                ur5_idx = self.joint_name_to_index[name_in_msg]
                
                if has_positions and msg_idx < len(msg.position):
                    temp_q[ur5_idx] = msg.position[msg_idx]
                
                # --- CORREÇÃO AQUI: Garantir que temp_dq é atribuído mesmo sem dados ---
                if has_velocities and msg_idx < len(msg.velocity):
                    temp_dq[ur5_idx] = msg.velocity[msg_idx]
                else:
                    # Este else agora é redundante se temp_dq já é np.zeros(6), mas mantém a lógica
                    rospy.logwarn_throttle(5, f"No velocity data for joint '{name_in_msg}' at index {msg_idx} in /ur5/joint_states. Defaulting to 0.0.")
                    temp_dq[ur5_idx] = 0.0 
            
        self.q_gazebo = temp_q
        self.q_dot_gazebo = temp_dq # AQUI a temp_dq estava undefined antes.
        self.last_joint_state_msg = msg

    def setup_gazebo_and_run(self, initial_torques_func, torque_limits):
        rospy.loginfo("Setting up Gazebo environment...")
        
        try:
            self.pause_physics_srv()
            rospy.loginfo("Gazebo physics paused.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to pause Gazebo physics: {e}. Is Gazebo running?")
            return False

        rospy.loginfo("Waiting for /ur5/joint_states to settle...")
        try:
            rospy.wait_for_message('/ur5/joint_states', JointState, timeout=10)
            rospy.loginfo("Initial joint states received successfully.")
        except rospy.ROSException:
            rospy.logwarn("Timeout waiting for /ur5/joint_states. Initial state might be off. Proceeding anyway.")

        try:
            self.unpause_physics_srv()
            rospy.loginfo("Gazebo physics unpaused. Starting simulation.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to unpause Gazebo physics: {e}")
            return False

        start_sim_time = self.last_joint_state_msg.header.stamp.to_sec() if self.last_joint_state_msg else rospy.Time.now().to_sec()
        current_sim_time = 0.0

        rospy.loginfo("Starting torque application loop...")
        while not rospy.is_shutdown() and current_sim_time < SIM_DURATION:
            if self.last_joint_state_msg:
                current_time_from_start = self.last_joint_state_msg.header.stamp.to_sec() - start_sim_time
            else:
                current_time_from_start = rospy.Time.now().to_sec() - start_sim_time
                rospy.logwarn_throttle(1, "No joint_state message received yet. Time will be based on ROS wall clock.")

            tau_to_apply = initial_torques_func(current_time_from_start)
            
            effort_msg = Float64MultiArray()
            effort_msg.data = tau_to_apply.tolist() 
            self.effort_pub.publish(effort_msg)

            self.time_history.append(current_time_from_start)
            self.q_history.append(self.q_gazebo.copy())
            self.q_dot_history.append(self.q_dot_gazebo.copy())
            self.applied_torque_history.append(tau_to_apply.copy())

            current_sim_time = current_time_from_start 
            self.rate.sleep()
        
        rospy.loginfo(f"Simulation duration ({SIM_DURATION}s) reached. Stopping robot.")
        final_stop_torques = Float64MultiArray()
        final_stop_torques.data = np.zeros(6, dtype=np.float64).tolist()
        self.effort_pub.publish(final_stop_torques) 

        rospy.loginfo("Torque application loop concluded.")
        return True

    def save_results(self):
        if not self.time_history: 
            rospy.logwarn("No Gazebo simulation data collected. Skipping save.")
            return

        save_directory = os.path.expanduser('~/miguel_ws/src/trajectory_pkg/scripts/dynamic_study_new')
        filename = 'gazebo_sim_results.npz'
        
        full_save_path = os.path.join(save_directory, filename)

        os.makedirs(save_directory, exist_ok=True)

        try:
            np.savez(full_save_path, 
                     times=np.array(self.time_history), 
                     q=np.array(self.q_history), 
                     q_dot=np.array(self.q_dot_history),
                     tau_applied=np.array(self.applied_torque_history)) 
            rospy.loginfo(f"Gazebo simulation results saved to '{full_save_path}'.")
        except Exception as e:
            rospy.logerr(f"Failed to save simulation results to '{full_save_path}': {e}")

if __name__ == '__main__':
    try:
        torque_limits = np.array([150.0, 150.0, 150.0, 28.0, 28.0, 28.0], dtype=np.float64) 
        
        initial_params_dict = get_initial_params() 
        user_defined_torques = np.array(initial_params_dict['tau_apply'], dtype=np.float64) 
        
        rospy.loginfo(f"\nUser-defined torques to apply for {TORQUE_APPLY_DURATION}s:\n{user_defined_torques} Nm")

        def get_torques_for_time(t):
            if t < TORQUE_APPLY_DURATION:
                return user_defined_torques 
            else:
                return np.zeros(6, dtype=np.float64) 

        commander = GazeboTorqueCommander()
        
        if commander.setup_gazebo_and_run(get_torques_for_time, torque_limits):
            commander.save_results()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt received. Shutting down commander.")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {e}")
  
  
  
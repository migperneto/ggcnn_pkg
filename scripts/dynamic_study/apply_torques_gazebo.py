#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64 
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState
import numpy as np
import time
from std_srvs.srv import Empty 

# Constants (should match ur5_dynamic_study.py)
SIM_DURATION = 5.0 

class GazeboTorqueApplier:
    def __init__(self, random_torques_to_apply, sim_duration):
        rospy.init_node('gazebo_torque_applier', anonymous=True)
        
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self.joint_name_to_index = {name: i for i, name in enumerate(self.joint_names)}
        
        # --- MODIFICADO: UM ÚNICO PUBLISHER PARA Float64MultiArray ---
        # O tópico será /ur5/<nome_do_controlador_de_grupo>/command
        # De acordo com seu plugin /ur5 e o nome do controlador ur5_group_effort_controller
        self.effort_pub = rospy.Publisher('/ur5/group_effort_controller/command', Float64MultiArray, queue_size=10)
        
        # O tópico do JointState permanece o mesmo (confirmado como /ur5/joint_states)
        self.joint_state_sub = rospy.Subscriber('/ur5/joint_states', JointState, self.joint_state_callback)
        
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        rospy.wait_for_service('/gazebo/pause_physics')
        rospy.wait_for_service('/gazebo/unpause_physics')
        self.pause_physics_srv = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics_srv = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

        self.random_torques = random_torques_to_apply
        self.sim_duration = sim_duration
        self.rate = rospy.Rate(100) # Control loop rate for publishing torques

        self.q_gazebo = np.zeros(6)
        self.q_dot_gazebo = np.zeros(6)
        self.last_joint_state_msg = None 

        self.time_history_gazebo = []
        self.q_history_gazebo = []
        self.q_dot_history_gazebo = []
        
        rospy.loginfo("Gazebo Torque Applier Initialized.")

    def joint_state_callback(self, msg):
        temp_q = np.zeros(6)
        temp_q_dot = np.zeros(6)
        
        has_positions = len(msg.position) > 0
        has_velocities = len(msg.velocity) > 0
        
        for msg_idx, name_in_msg in enumerate(msg.name):
            if name_in_msg in self.joint_name_to_index:
                ur5_idx = self.joint_name_to_index[name_in_msg]
                
                if has_positions and msg_idx < len(msg.position):
                    temp_q[ur5_idx] = msg.position[msg_idx]
                
                if has_velocities and msg_idx < len(msg.velocity):
                    temp_q_dot[ur5_idx] = msg.velocity[msg_idx]
                else:
                    rospy.logwarn_throttle(5, f"No velocity data for joint '{name_in_msg}' at index {msg_idx} in message. Defaulting to 0.0.")
                    temp_q_dot[ur5_idx] = 0.0 
            
        self.q_gazebo = temp_q
        self.q_dot_gazebo = temp_q_dot
        self.last_joint_state_msg = msg 


    def set_ur5_initial_pose_gazebo(self, q_initial_radians):
        rospy.loginfo("Attempting to set UR5 initial pose in Gazebo...")
        try:
            self.pause_physics_srv()
            rospy.loginfo("Gazebo physics paused.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to pause Gazebo physics: {e}")
            return False

        rospy.loginfo("Waiting for /ur5/joint_states to settle...")
        try:
            rospy.wait_for_message('/ur5/joint_states', JointState, timeout=5)
            rospy.loginfo("Joint states received.")
        except rospy.ROSException:
            rospy.logwarn("Timeout waiting for /ur5/joint_states. Proceeding, but initial state might be off.")

        try:
            self.unpause_physics_srv()
            rospy.loginfo("Gazebo physics unpaused.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to unpause Gazebo physics: {e}")
            return False
            
        return True

    def run(self, applied_torques_func, initial_q_for_comparison):
        if not self.set_ur5_initial_pose_gazebo(initial_q_for_comparison):
            rospy.logerr("Failed to set initial pose. Exiting.")
            return None, None, None
        
        rospy.sleep(0.5) 
        
        start_sim_time = self.last_joint_state_msg.header.stamp.to_sec() if self.last_joint_state_msg else rospy.Time.now().to_sec()
        
        rospy.loginfo("Starting torque application in Gazebo...")
        while not rospy.is_shutdown():
            if self.last_joint_state_msg:
                current_time = self.last_joint_state_msg.header.stamp.to_sec() - start_sim_time
            else:
                current_time = rospy.Time.now().to_sec() - start_sim_time
                rospy.logwarn_throttle(1, "No joint_state message received yet. Time will be based on ROS wall clock.")

            if current_time >= self.sim_duration: 
                rospy.loginfo("Simulation duration reached. Stopping torque application.")
                # --- MODIFICADO: PUBLICAR Float64MultiArray DE ZEROS ---
                effort_msg = Float64MultiArray()
                effort_msg.data = np.zeros(6).tolist() # Enviar array de zeros
                self.effort_pub.publish(effort_msg)
                break

            tau_to_apply = applied_torques_func(current_time)
            
            # --- MODIFICADO: PUBLICAR Float64MultiArray DE TORQUES ---
            effort_msg = Float64MultiArray()
            effort_msg.data = tau_to_apply.tolist() # Converter numpy array para lista Python
            self.effort_pub.publish(effort_msg)
            # --- FIM DA MODIFICAÇÃO ---

            if self.last_joint_state_msg:
                self.time_history_gazebo.append(current_time)
                self.q_history_gazebo.append(self.q_gazebo.copy())
                self.q_dot_history_gazebo.append(self.q_dot_gazebo.copy())
            
            self.rate.sleep()
        
        rospy.loginfo("Torque application in Gazebo concluded.")
        return (np.array(self.time_history_gazebo), 
                np.array(self.q_history_gazebo), 
                np.array(self.q_dot_history_gazebo))

if __name__ == '__main__':
    try:
        torque_limits = np.array([100, 100, 80, 20, 20, 10]) # Nm
        random_torques = np.random.uniform(-torque_limits, torque_limits)
        
        def get_applied_torques_gazebo(t):
            if t <= 0.5:
                return random_torques
            else:
                return np.zeros(6)

        initial_q_for_gazebo_comparison = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 
        
        applier = GazeboTorqueApplier(random_torques, SIM_DURATION)
        
        (gazebo_times, gazebo_q, gazebo_q_dot) = applier.run(get_applied_torques_gazebo, initial_q_for_gazebo_comparison)
        
        if gazebo_times is not None and len(gazebo_times) > 0: 
            np.savez('gazebo_sim_results.npz', times=gazebo_times, q=gazebo_q, q_dot=gazebo_q_dot)
            rospy.loginfo("Gazebo simulation results saved to 'gazebo_sim_results.npz'.")
        else:
            rospy.logwarn("No Gazebo simulation data collected. Check Gazebo and ROS setup.")

    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3

import rospy 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

current_joint_angle = None
joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", 
                                  "elbow_joint", "wrist_1_joint", 
                                  "wrist_2_joint", "wrist_3_joint"]

# def trajectory_callback(msg):
#     global current_joint_angle
#     rospy.loginfo('Recebendo a nova trajetoria!!!')
#     current_joint_angle = msg.position


def ur5_trajectory():
    global current_joint_angle, joint_names
    rospy.init_node('ur5_commander', anonymous=True)
    pub = rospy.Publisher('/ur5/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)

    # # Subscrição ao tópico de estados das juntas
    # rospy.Subscriber('/ur5/joint_states', JointState, trajectory_callback)
    # rospy.loginfo("Subscriber para o UR5 iniciado. Aguardando dados...")

    rospy.sleep(1)

    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    
    positions = [[0.0, -1.57, 0.0, 0.0, 0.0, -1.57],   
                 [0.0, -1.56, 1.14, 0.05, 0.0, -2.77],
                 [0.0, -1.57, 0.0, 0.0, 0.0, -1.57] 
                 ]
    
    for i, pos in enumerate(positions):
        point = JointTrajectoryPoint()
        point.positions = pos 
        point.time_from_start = rospy.Duration(3 * (i + 1))   # (i*0.1)   (3 * (i + 1))
        trajectory.points.append(point)

    pub.publish(trajectory)
    rospy.loginfo(f'Mensagem completa com {i+1} posições enviada com sucesso!!!')

    # # Controla a taxa de logs (uma vez a cada 2 segundos)
    # rate = rospy.Rate(2.0)  # 2.0 Hz = 1 mensagem a cada 0.5 segundos

    # while not rospy.is_shutdown():
    #     # Verifica se há dados disponíveis
    #     if current_joint_angle is not None:
    #         rospy.loginfo(f"Posições das Juntas: {list(current_joint_angle)}")
    #     else:
    #         rospy.loginfo("Aguardando dados do tópico /ur5/joint_states...")

    #     # Aguarda até o próximo ciclo
    #     rate.sleep()

    # rospy.spin()

if __name__=='__main__':
    try:
        ur5_trajectory()
    except rospy.ROSInterruptException:
        pass




      

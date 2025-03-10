#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import tf.transformations  # Para converter Quaternions em outros formatos

# Função para obter a posição e orientação de um frame em relação a outro
def get_frame_pose(tf_buffer, target_frame, reference_frame='base_link'):
    try:
        transform = tf_buffer.lookup_transform(reference_frame, target_frame, rospy.Time(0), timeout=rospy.Duration(1.0))
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        x = translation.x
        y = translation.y
        z = translation.z
        qx = rotation.x
        qy = rotation.y
        qz = rotation.z
        qw = rotation.w

        return (x, y, z, qx, qy, qz, qw)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("TF error: %s", e)
        return None

# Função para converter um quaternion em RPY
def quaternion_to_rpy(qx, qy, qz, qw):
    rpy = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))
    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]
    return (roll, pitch, yaw)


def pose_ur5():
    rospy.init_node('get_frame_poses')

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10.0)

    links = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0"]

    #obter a posição do "shoulder_link", "upper_arm_link", "forearm_link" em relação ao "base_link"
    for i in range(1, 7):
        link = links[i]
        pose = get_frame_pose(tf_buffer, link, 'base_link')
        if pose:
            x, y, z, qx, qy, qz, qw = pose
    
    return x, y, z, qx, qy, qz, qw
    


# s = pose_ur5()  # Obtem a posição e orientação de um frame em relação a base_link
# print(s)


# d = quaternion_to_rpy(s[3], s[4], s[5], s[6])   # Converte Quaternion para Roll, Pitch, Yaw (RPY)
# print(d)












#***********************************************

# import rospy
# import tf2_ros
# import geometry_msgs.msg
# import tf.transformations  # Para converter Quaternions em outros formatos

# # Função para obter a posição e orientação de um frame em relação a outro
# def get_frame_pose(tf_buffer, target_frame, reference_frame='base_link'):
#     try:
#         transform = tf_buffer.lookup_transform(reference_frame, target_frame, rospy.Time(0), timeout=rospy.Duration(1.0))
#         translation = transform.transform.translation
#         rotation = transform.transform.rotation

#         x = translation.x
#         y = translation.y
#         z = translation.z
#         qx = rotation.x
#         qy = rotation.y
#         qz = rotation.z
#         qw = rotation.w

#         return (x, y, z, qx, qy, qz, qw)
#     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
#         rospy.logwarn("TF error: %s", e)
#         return None

# # Função para converter um quaternion em RPY
# def quaternion_to_rpy(qx, qy, qz, qw):
#     rpy = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))
#     roll = rpy[0]
#     pitch = rpy[1]
#     yaw = rpy[2]
#     return (roll, pitch, yaw)

# if __name__ == '__main__':
#     rospy.init_node('get_frame_poses')

#     tf_buffer = tf2_ros.Buffer()
#     listener = tf2_ros.TransformListener(tf_buffer)

#     rate = rospy.Rate(10.0)

#     links = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0"]

#     #obter a posição do "shoulder_link", "upper_arm_link", "forearm_link" em relação ao "base_link"
#     for i in range(1, 4):
#         link = links[i]
#         pose = get_frame_pose(tf_buffer, link, 'base_link')
#         if pose:
#             x, y, z, qx, qy, qz, qw = pose
#             rospy.loginfo("{} Position (x, y, z): ({}, {}, {})".format(link, x, y, z))
    
#     #obter a posição do "wrist_1_link", "wrist_2_link", "wrist_3_link" em relação ao "base_link"
#     for i in range(4, 7):
#         link = links[i]
#         pose = get_frame_pose(tf_buffer, link, 'base_link')
#         if pose:
#             x, y, z, qx, qy, qz, qw = pose
#             rospy.loginfo("{} Position (x, y, z): ({}, {}, {})".format(link, x, y, z))
#             rospy.loginfo("{} Orientation (Quaternion): ({}, {}, {}, {})".format(link, qx, qy, qz, qw))

#             # Converte Quaternion para Roll, Pitch, Yaw (RPY)
#             roll, pitch, yaw = quaternion_to_rpy(qx, qy, qz, qw)
#             rospy.loginfo("{} Orientation (RPY in radians): (Roll: {}, Pitch: {}, Yaw: {})".format(link, roll, pitch, yaw))


# ***********************************************

# import rospy
# import tf
# import tf.transformations

# if __name__ == '__main__':
#     rospy.init_node('get_ur5_position')

#     listener = tf.TransformListener()

#     rate = rospy.Rate(10.0) # Tenta obter a transformação a 10 Hz

#     while not rospy.is_shutdown():
#         try:
#             # Obtém a transformação de base_link para tool0.
#             # listener.waitForTransform garante que a transformação esteja disponível.
#             listener.waitForTransform('/base_link', '/tool0', rospy.Time(), rospy.Duration(4.0))
#             (trans, rot) = listener.lookupTransform('/base_link', '/tool0', rospy.Time(0))


#             x = trans[0]
#             y = trans[1]
#             z = trans[2]

#             X = -rot[0]
#             Y = -rot[1]
#             Z = -rot[2]
#             W = -rot[3]

#             rospy.loginfo("UR5 Position (x, y, z): ({}, {}, {})".format(x, y, z))
#             rospy.loginfo("UR5 Orientation (x, y, z, w): ({}, {}, {}, {})".format(X, Y, Z, W))

#         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
#             rospy.logwarn("TF error: %s", e)
#             rate.sleep()
#             continue

#         rate.sleep()

        #***********************************************
# import rospy
# import tf2_ros
# import geometry_msgs.msg

# if __name__ == '__main__':
#     rospy.init_node('get_ur5_position')

#     tf_buffer = tf2_ros.Buffer()  # Use tf2_ros.Buffer
#     listener = tf2_ros.TransformListener(tf_buffer)

#     rate = rospy.Rate(10.0)

#     while not rospy.is_shutdown():
#         try:
#             transform = tf_buffer.lookup_transform('base_link', 'elbow_joint', rospy.Time(0), timeout=rospy.Duration(1.0))

#             translation = transform.transform.translation
#             rotation = transform.transform.rotation
#             x = translation.x
#             y = translation.y
#             z = translation.z

#             X = -rotation.x
#             Y = -rotation.y
#             Z = -rotation.z
#             W = -rotation.w

#             rospy.loginfo("UR5 Position (x, y, z): ({}, {}, {})".format(x, y, z))
#             rospy.loginfo("UR5 Orientation (x, y, z, w): ({}, {}, {}, {})".format(X, Y, Z, W))

#         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
#             rospy.logwarn("TF error: %s", e)
#             rospy.sleep(0.1)
#             continue

#         rate.sleep()

        #***********************************************

# import rospy
# import tf2_ros
# import geometry_msgs.msg

# def get_frame_position(tf_buffer, target_frame, reference_frame='base_link'):
#     """
#     Obtém a posição (x, y, z) de um frame específico em relação a um frame de referência.

#     Args:
#         tf_buffer: O buffer TF.
#         target_frame: O nome do frame cuja posição você quer obter (ex: 'shoulder_link').
#         reference_frame: O nome do frame de referência (ex: 'base_link').

#     Returns:
#         Uma tupla (x, y, z) com a posição, ou None se a transformação não estiver disponível.
#     """
#     try:
#         transform = tf_buffer.lookup_transform(reference_frame, target_frame, rospy.Time(0), timeout=rospy.Duration(1.0))
#         translation = transform.transform.translation
#         x = translation.x
#         y = translation.y
#         z = translation.z
#         return (x, y, z)
#     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
#         rospy.logwarn("TF error: %s", e)
#         return None

# if __name__ == '__main__':
#     rospy.init_node('get_frame_positions')

#     tf_buffer = tf2_ros.Buffer()
#     listener = tf2_ros.TransformListener(tf_buffer)

#     rate = rospy.Rate(10.0)

#     while not rospy.is_shutdown():
#         # Obtém a posição do shoulder_link em relação ao base_link
#         shoulder_position = get_frame_position(tf_buffer, 'shoulder_link', 'base_link')
#         if shoulder_position:
#             rospy.loginfo("Shoulder Link Position (x, y, z): {}".format(shoulder_position))

#         # Obtém a posição do wrist_3_link em relação ao base_link
#         wrist_3_position = get_frame_position(tf_buffer, 'wrist_3_link', 'base_link')
#         if wrist_3_position:
#             rospy.loginfo("Wrist 3 Link Position (x, y, z): {}".format(wrist_3_position))

#         # Você pode adicionar mais frames aqui, como elbow_link, wrist_1_link, etc.

#                 # Obtém a posição do tool0 em relação ao base_link
#         tool0_position = get_frame_position(tf_buffer, 'tool0', 'base_link')
#         if wrist_3_position:
#             rospy.loginfo("tool0 Link Position (x, y, z): {}".format(tool0_position))

#         rate.sleep()

        #***********************************************

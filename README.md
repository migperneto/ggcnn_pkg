As questões com a garra foram resolvidas com o "Gazebo grasp fix plugin". Para mais, favor ver:
https://github.com/elena-ecn/pick-and-place/tree/main
https://github.com/JenniferBuehler/gazebo-pkgs/wiki

Adicionando esse trecho de script no final no arquivo .urdf / .xacro:

<gazebo>
   <plugin name="gazebo_grasp_fix" filename="libgazebo_grasp_fix.so">
        <arm>
           <arm_name>ur5</arm_name>
           <palm_link> wrist_3_link  </palm_link>
           <gripper_link> robotiq_85_left_inner_knuckle_link </gripper_link>
           <gripper_link> robotiq_85_left_finger_tip_link </gripper_link>
           <gripper_link> robotiq_85_left_knuckle_link </gripper_link>
           <gripper_link> robotiq_85_left_finger_link </gripper_link>
           <gripper_link> robotiq_85_right_inner_knuckle_link </gripper_link>
           <gripper_link> robotiq_85_right_finger_tip_link </gripper_link>
           <gripper_link> robotiq_85_right_knuckle_link </gripper_link>
           <gripper_link> robotiq_85_right_finger_link </gripper_link>
        </arm>
       <forces_angle_tolerance>100</forces_angle_tolerance>
       <update_rate>4</update_rate>
       <grip_count_threshold>4</grip_count_threshold>
       <max_grip_count>8</max_grip_count>
       <release_tolerance>0.005</release_tolerance>
       <disable_collisions_on_attach>false</disable_collisions_on_attach>
       <contact_topic>__default_topic__</contact_topic>
    </plugin>
</gazebo>

#!/usr/bin/env python3
# license removed for brevity
import rospy
# import tf
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
import autoTest
import conversion as cnvt 

import tf2_ros

if __name__ == '__main__':

    

    try:
        autoTest.main()
        pub = rospy.Publisher('aa_angles', Quaternion, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        while not rospy.is_shutdown():
    
            (trans, rot) = listener.lookupTransform('/link_base', '/link6', rospy.time(0))
            (roll, pitch, yaw) = cnvt.quaternion_to_euler_angle(trans[0], trans[1], trans[2], trans[3])
            eulerVal = autoTest.orientation_angle()
            eulerVal = [roll] + eulerVal
            new_quaternions = cnvt.euler_to_quternion(roll, eulerVal[1], eulerVal[2])
            pub_quat_data = Quaternion
            pub_quat_data.x =   new_quaternions[0]
            pub_quat_data.y =   new_quaternions[1]
            pub_quat_data.z =   new_quaternions[2]
            pub_quat_data.w =   new_quaternions[3]
            pub.publish(pub_quat_data)
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass

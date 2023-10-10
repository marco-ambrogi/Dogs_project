#!/usr/bin/env python3  
import roslib
# roslib.load_manifest('learning_tf')

import rospy
import tf
import time
from gazebo_msgs.msg import ModelStates
# from geometry_msgs.msg import Pose, PoseWithCovariance, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

posizione = None
orientamento = None
pos = None
orient = None
# posa = Pose()
# posa_con_covarianza = PoseWithCovariance()
# posa_con_covarianza_stampata = PoseWithCovarianceStamped()
# covarianza = []
# for i in range(36):
#     covarianza.append(0.0)

def callback_model_states(data):
    #time.sleep(50)
    global posizione, orientamento
    posizione = data.pose[len(data.pose) - 1].position
    orientamento = data.pose[len(data.pose) - 1].orientation
    # posizione = data.pose[3].position
    # orientamento = data.pose[3].orientation

def callback_a1_odom(data):
    #time.sleep(50)
    global pos, orient
    pos = data.pose.pose.position
    orient = data.pose.pose.orientation

def mytf_node():
    global pub_pose_with_covariance_stamped, pub_initial_pose
    prima = True
    rospy.init_node('my_tf_broadcaster', anonymous= True)
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback_model_states)
    rospy.Subscriber('/a1/odom', Odometry, callback_a1_odom)
    # pub_pose_with_covariance_stamped = rospy.Publisher('/pose_with_covariance_stamped', PoseWithCovarianceStamped, queue_size=1)
    br = tf.TransformBroadcaster()
    # rate = rospy.Rate(10.0)
    # while (posizione == None) or (orientamento == None):
    #     pass
    # br.sendTransform((posizione.x, posizione.y, posizione.z),
    #                      (orientamento.x, orientamento.y, orientamento.z, orientamento.w),
    #                      rospy.Time.now(), "base", "map")
    while True:
        while (posizione == None) or (orientamento == None) or (pos == None) or (orient == None):
            pass
        br.sendTransform((posizione.x, posizione.y, posizione.z),
                         (orientamento.x, orientamento.y, orientamento.z, orientamento.w),
                         rospy.Time.now(), "base", "map")
        
        # L'istruzione seguente va commentata se si usa gmapping mentre va decommentata se si usa amcl
        br.sendTransform((pos.x, pos.y, pos.z),
                         (orient.x, orient.y, orient.z, orient.w),
                         rospy.Time.now(), "map", "odom")
                         
        #rate.sleep()
        time.sleep(0.3)
if __name__ == '__main__':
    try:
        # print("MyTfBroadcaster attivo!\n")
        # time.sleep(30)
        mytf_node()
    except rospy.ROSInterruptException:
        #pass
        mytf_node()
    
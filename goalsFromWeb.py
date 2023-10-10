#!/usr/bin/env python3  
import roslib
# roslib.load_manifest('learning_tf')

import rospy
import time
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from actionlib_msgs.msg import GoalID 
from std_msgs.msg import Bool

goal = PoseStamped()
robot_pose = Pose()
num_goals = 0
id_goal = []
action_result = MoveBaseActionResult()
isFollowing = False

def callback_pose(data):
    global robot_pose
    # robotPose = data
    robotPose = data.pose[len(data.pose) - 1]
    robot_pose.position = robotPose.position
    robot_pose.orientation = robotPose.orientation
    pub_robot_pose.publish(robot_pose)


def callback_webgoal(data):
    global goal, id_goal
    if isFollowing == False:
        goal.header = data.goal.target_pose.header
        posizione = data.goal.target_pose.pose.position
        orientamento = data.goal.target_pose.pose.orientation
        goal.pose.position.x = posizione.x
        goal.pose.position.y = posizione.y
        goal.pose.position.z = posizione.z
        goal.pose.orientation.x = orientamento.x
        goal.pose.orientation.y = orientamento.y
        goal.pose.orientation.z = orientamento.z
        goal.pose.orientation.w = orientamento.w
        pub_goals.publish(goal)
        id_goal.append(data.goal_id)
        # print(id_goal)
    else:
        action_result.status.goal_id = data.goal_id
        pub_cancel_goals.publish(action_result)
        time.sleep(0.01)

def callback_goal_raggiunto(data):
    global id_goal, action_result
    for i in range(len(id_goal)):
        action_result.status.goal_id = id_goal[i]
        pub_cancel_goals.publish(action_result)
        time.sleep(0.01)
        #print("canc")
    id_goal = []
    # print(id_goal)

def callback_info_goal_irraggiungibile(data):
    global id_goal, action_result
    # time.sleep(0.5)
    action_result.status.goal_id = id_goal[len(id_goal) - 1]
    pub_cancel_goals.publish(action_result)
    id_goal.pop(len(id_goal) - 1)

def callback_isFollowing(data):
    global isFollowing
    isFollowing = data.data

def callback_ptc(data):
    global points
    points = data
    # print(len(data.data))
    # print()



def my_goals_from_web():
    global pub_goals, pub_robot_pose, pub_cancel_goals
    prima = True
    rospy.init_node('my_goals_from_web', anonymous= True)
    pub_robot_pose = rospy.Publisher('/robot_pose', Pose, queue_size=1)
    pub_goals = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    pub_cancel_goals = rospy.Publisher('/pr2_move_base/result', MoveBaseActionResult, queue_size=1)
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback_pose)
    rospy.Subscriber('/pr2_move_base/goal', MoveBaseActionGoal, callback_webgoal)
    rospy.Subscriber('/info_goal_raggiunto', Bool, callback_goal_raggiunto)
    rospy.Subscriber('/globalPlanner/info_goal_irraggiungibile', Bool, callback_info_goal_irraggiungibile)
    rospy.Subscriber('/isFollowing', Bool, callback_isFollowing)

    rospy.spin()
        

if __name__ == '__main__':
    try:
        print("MyGoalsFromWeb attivo!\n")
        # time.sleep(30)
        my_goals_from_web()
    except rospy.ROSInterruptException:
        #pass
        print("MyGoalsFromWeb attivo!\n")
        my_goals_from_web()



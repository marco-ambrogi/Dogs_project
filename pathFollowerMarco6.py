import math
import time
import rospy
import tf
# import sys

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Quaternion, Pose
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool, Int32, String
from sensor_msgs.msg import Joy
from gazebo_msgs.msg import ModelStates
from datetime import datetime
from rviz_write_button.msg import WriteMsg

path = Path()
robotPose = PoseStamped()
goal = PoseStamped()
newGoal = PoseStamped()
pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
foundObstacle = False
findNewKeyPoint = False
newPoseIndex = 0
distanceTreshold = 0.05 # 0.05
angleTreshold = 0.05 #0.02 

# v_lineare = 0.8
# v_angolare = 0.5

vMax = 0.7
vMin = 0.5
wMax = 0.5
wMin = 0.5
# kw = 1/3
# kv = 1/3

comando = Joy()
comando.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
comando.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
old_robotGoal = None
keyPoses_raggiunte = []
stop = True
robotGoal = None
postpone = False
home_pose = Pose()
home_pose.position.x = 0.0
home_pose.position.y = 0.0
home_pose.position.z = 0.2
home_pose.orientation.x = 0.0
home_pose.orientation.y = 0.0
home_pose.orientation.z = 0.0
home_pose.orientation.w = 1.0


# posizione del percorso nella quale il robot si e' ritrovato dopo aver aggirato l'ostacolo
def callback_newPoseIndex(data):
    global newPoseIndex
    newPoseIndex = data.data
    print("Robot nella posizione ", newPoseIndex, " del percorso")

# si attiva quando viene segnalata la presenza di un ostacolo sul percorso, oppure quando l'ostacolo e' stato superato
def callback_obstacle(data):
    global foundObstacle
    foundObstacle = data.data
    # print("Found obstacle:", foundObstacle)

def callback_pose(data):
    global robotPose
    # robotPose = data
    robotPose = data.pose[len(data.pose) - 1]

def callback_goal(data):
    global robotGoal
    robotGoal = data.pose
    # goals.append(robotGoal)

# se viene pubblicato un nuovo percorso, incomincia ad inseguirlo
def callback_path(data):
    global path
    path = data
    print("-----------------------\n")
    print("Nuovo percorso ricevuto")
    # while stop:
    #     comando.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #     pub_comando.publish(comando)
    #     pass
    #followPath()

def callback_start(data):
    start()

def callback_postpone(data):
    global stop, postpone
    postpone = True
    while datetime.now().strftime("%H:%M") != data.data:
        if postpone == False:
            return
    start()
    postpone = False

def callback_reset_postpone(data):
    global postpone
    postpone = False

def callback_stop(data):
    global stop
    stop = True

def callback_resume(data):
    global stop
    stop = False

def callback_back_to_home(data):
    global robotGoal, keyPoses_raggiunte, stop
    robotGoal = home_pose
    pub_goal_raggiunto.publish(True)
    keyPoses_raggiunte = []
    stop = False
    time.sleep(0.5)
    start()


def start():
    global stop
    stop = False
    pub_isFollowing.publish(True)
    if path.poses != []:
        followPath()
    pub_isFollowing.publish(False)



# funzione che segue il percorso ricevuto
def followPath():
    global findNewKeyPoint, comando, nuovo_percorso, old_robotGoal, keyPoses, keyPoses_raggiunte
    old_robotGoal = robotGoal
    comando.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pub_comando.publish(comando)
    poses = path.poses          # pose del percorso
    keyPoses = []               # lista punti con cambio di orientamento
    keyPoses.append(poses[0])   # punto di partenza

    print("Lunghezza percorso: ", len(path.poses))

    # estrazione dei keypoint dal path
    for i in range(1, len(poses) - 1): #prima era len(poses) - 1
        if poses[i].pose.orientation != poses[i - 1].pose.orientation:
            keyPoses.append(poses[i])
    keyPoses.append(poses[len(poses) - 1])
    print("Numero keyPoints: ", len(keyPoses))
    
    i = 1

    # inizio l'inseguimento del percorso, posa dopo posa
    while i < len(keyPoses):
        if (robotGoal != old_robotGoal):
            comando.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            pub_comando.publish(comando)
            return
        nextPose = keyPoses[i].pose
        if keyPoses[i].pose in keyPoses_raggiunte:
            i = i + 1
            continue
        print("- Movimento verso la posizione numero: ", keyPoses[i].header.seq)
        goToPose(nextPose) #orienta e muovi in direzione del prossimo keypoint

        if findNewKeyPoint: # verifico se devo trovare un nuovo obiettivo, dovuto all'aggiramento di un'ostacolo
            print("--- Ostacolo Aggirato, cerco nuovo keypoint")
            j = newPoseIndex
            # scorro la lista di pose, a partire dalla nuova posizione del robot, per vedere dove sta il prossimo keypoint
            while (poses[j].pose.orientation == poses[newPoseIndex].pose.orientation) and (j < len(poses)-1):
                j = j + 1
            h = poses[j].header.seq
            k = i
            # ne estraggo ora l'indice associato nella lista dei keypoint
            while (int(keyPoses[k].header.seq) != h) and (k < len(keyPoses)-1): 
                k = k + 1
            i = k
            findNewKeyPoint = False
            # ricomincio con la solita routine ma da un'altra posizione della lista
            print("--- Prossimo keypoint nella posizione: ", h)
        else:
            i = i + 1
    # mi assicuro di avere l'orientamento desiderato
    setOrientation(nextPose)
    print("Goal Raggiunto!")
    print("-----------------------\n")
    pub_goal_raggiunto.publish(True)
    keyPoses_raggiunte = []


# calcolo la distanza tra due posizioni nello spazio
def distanceFromPoses(p1, p2):
    dist = math.sqrt(
        math.pow(float(p1.position.x) - float(p2.position.x), 2) + math.pow(float(p1.position.y) - float(p2.position.y),2))
    return dist

# ricavo l'orientamento intorno all'asse z associato a quaternione della posa fornita
def getYawFromPose(thisPose):
    qPose = [thisPose.orientation.x, thisPose.orientation.y, thisPose.orientation.z, thisPose.orientation.w]
    thetaPose = float(tf.transformations.euler_from_quaternion(qPose)[2])
    #  preferisco lavorare con angoli positivi: [0 -> 2*Pi]
    if thetaPose < 0:
        thetaPose = 2 * math.pi + thetaPose
    return thetaPose

# ricavo la direzione da seguire per raggiungere p2 partendo da p1
def angleFromPoses(p1, p2):
    thetaTarget = math.atan2(float(p2.position.y) - float(p1.position.y),
                             float(p2.position.x) - float(p1.position.x))  # atan2(y, x) restituisce l'arcotangente di y/x in radianti.
    if thetaTarget < 0:
        thetaTarget = thetaTarget + 2 * math.pi

    q = [p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w]
    thetaRobot = float(tf.transformations.euler_from_quaternion(q)[2])
    if thetaRobot < 0:
        thetaRobot = thetaRobot + 2 * math.pi

    thetaDist = thetaTarget - thetaRobot

    if thetaDist >= math.pi:
        thetaDist = thetaDist - 2 * math.pi
    elif thetaDist <= -math.pi:
        thetaDist = thetaDist + 2 * math.pi

    return thetaDist

# oriento il robot secondo la posa specificata, funzione utilizzata solo per l'ultima posa del percorso
def setOrientation(goalPose):
    global comando
    print("- In rotazione")
    r = rospy.Rate(10)
    # msg_vel = Twist()
    # pub_vel.publish(msg_vel)
    comando.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pub_comando.publish(comando)
    thetaTarget = getYawFromPose(goalPose)
    # thetaRobot = getYawFromPose(robotPose.pose.pose)
    thetaRobot = getYawFromPose(robotPose)
    angleDist = thetaTarget - thetaRobot # calcolo l'errore nell'orientamento

    # faccio in modo che sia nell'intervallo [-Pi, +Pi]
    if angleDist > math.pi:
        angleDist = angleDist - 2 * math.pi
    elif angleDist < -math.pi:
        angleDist = angleDist + 2 * math.pi
    # utilizzo questo errore per regolare la velocita' di rotazione e il segno per il verso
    while abs(angleDist) > angleTreshold:
        if (robotGoal != old_robotGoal):
            comando.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            pub_comando.publish(comando)
            return
        thetaTarget = getYawFromPose(goalPose)
        # thetaRobot = getYawFromPose(robotPose.pose.pose)
        thetaRobot = getYawFromPose(robotPose)
        angleDist = thetaTarget - thetaRobot

        if angleDist >= math.pi:
            angleDist = angleDist - 2 * math.pi
        elif angleDist <= -math.pi:
            angleDist = angleDist + 2 * math.pi
        # la velocita' e' limitata da un valore masimo wMax
        if abs(angleDist) > angleTreshold:
            if angleDist > 0:
                # w = min(kw * angleDist, wMax)
                # comando.axes[0] = v_angolare
                v_ang = min(angleDist, wMax)
                v_ang = min(v_ang, wMin)
                comando.axes[0] = v_ang
                comando.axes[5] = 1.0

            else:
                # w = max(kw * angleDist , -wMax)
                v_ang = max(angleDist, -wMax)
                v_ang = min(v_ang, -wMin)
                comando.axes[0] = v_ang
                # comando.axes[0] = -v_angolare
                comando.axes[5] = 1.0

        # msg_vel.angular.z = w
        # pub_vel.publish(msg_vel)
        pub_comando.publish(comando)
        r.sleep()
    # msg_vel.angular.z = 0.0
    # pub_vel.publish(msg_vel)
    comando.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pub_comando.publish(comando)
    print("-- Orientamento Raggiunto")


def goToPose(goalPose):
    # dist = distanceFromPoses(robotPose.pose.pose, goalPose)
    dist = distanceFromPoses(robotPose, goalPose)
    global findNewKeyPoint, comando, keyPoses_raggiunte
    comando.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    comando.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    pub_comando.publish(comando)
    r = rospy.Rate(5) # frequenza con la quale controllo la posizione e la velocita'
    print("-- Distanza dal goal:", dist)
    print("-- In Movimento")

    while dist > distanceTreshold:  # finche' non sono sufficientemente vicino all'obiettivo
        if stop:
            comando.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            pub_comando.publish(comando)
            time.sleep(0.1)
            while stop:
                pass
        if (robotGoal != old_robotGoal):
            comando.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            pub_comando.publish(comando)
            return
        #calcolo distanze e errore di orientamento
        # dist = distanceFromPoses(robotPose.pose.pose, goalPose)
        # angleDist = angleFromPoses(robotPose.pose.pose, goalPose)
        dist = distanceFromPoses(robotPose, goalPose)
        angleDist = angleFromPoses(robotPose, goalPose)

        if foundObstacle: # se e' stato segnalato un ostacolo, attendo che l'obstacle avoidance faccia il suo lavoro
            print("--- Incontrato ostacolo!")
            while foundObstacle:
                time.sleep(1)
            findNewKeyPoint = True #una volta riavuto il controllo del robot, segnalo la necessita' di cercare il nuovo keypoint
            return

        # la velocita' di movimento e' proporzionale alla distanza "dist" e limitata da da "vMax"
        # v = min(kv*dist,vMax)
        v_lin = min(dist, vMax)
        v_lin = max(v_lin, vMin) 
        # comando.axes[4] = v_lineare
        comando.axes[4] = v_lin
        # la velocita' di rotazione e' proporzionale all'errore di orientamento "angleDist" e limitata da "wMax"
        if abs(angleDist) > angleTreshold:
            if angleDist > 0:
                # w = min(kw * angleDist, wMax)
                v_ang = min(angleDist, wMax)
                v_ang = min(v_ang, wMin)
                comando.axes[0] = v_ang
                # comando.axes[0] = v_angolare
                comando.axes[5] = 1.0
            else:
                # w = max(kw * angleDist, -wMax)
                v_ang = min(angleDist, -wMax)
                v_ang = min(v_ang, -wMin)
                comando.axes[0] = v_ang
                # comando.axes[0] = -v_angolare
                comando.axes[5] = 1.0
            if abs(angleDist) > 10*angleTreshold:
                # v = 0.0
                comando.axes[4] = 0.0
        else:
            # w = 0.0
            comando.axes[0] = 0.0
            comando.axes[5] = 1.0

        # msg_vel.linear.x = v
        # msg_vel.angular.z = w
        # pub_vel.publish(msg_vel)
        pub_comando.publish(comando)
        r.sleep()
    # msg_vel.linear.x = 0.0
    # msg_vel.angular.z = 0.0
    # pub_vel.publish(msg_vel)
    comando.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    comando.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    pub_comando.publish(comando)

    print("-- Posizione raggiunta")

    keyPoses_raggiunte.append(goalPose)
    # keyPoses.remove(goalPose)


def pathFollower():
    global pub_path, pub_comando, pub_posa_raggiunta, pub_goal_raggiunto, pub_isFollowing
    rospy.init_node('pathFollower', anonymous=True)
    
    # rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, callback_pose)  # per avere la robotPose corrente del turtlebot stimata con AMCL
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback_pose)  
    rospy.Subscriber('globalPlanner/path', Path, callback_path)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_goal)
    rospy.Subscriber('/user/stop', Bool, callback_stop)
    rospy.Subscriber('/user/resume', Bool, callback_resume)
    rospy.Subscriber('/user/start', Bool, callback_start)
    rospy.Subscriber('/rviz/start', WriteMsg, callback_start)
    rospy.Subscriber('/user/postpone', String, callback_postpone)
    rospy.Subscriber('/user/reset_postpone', Bool, callback_reset_postpone)
    rospy.Subscriber('/user/back_to_home', Bool, callback_back_to_home)
    # rospy.Subscriber('/globalPlanner/info_goal_irraggiungibile', Bool, callback_info_goal_irraggiungibile)
    pub_path = rospy.Publisher('globalPlanner/path', Path, queue_size=1)
    pub_comando = rospy.Publisher('/a1_joy/joy_ramped', Joy, queue_size=1)
    # pub_posa_raggiunta = rospy.Publisher('/info_posizione_raggiunta', Pose, queue_size=1)
    pub_goal_raggiunto = rospy.Publisher('/info_goal_raggiunto', Bool, queue_size=1)
    pub_isFollowing = rospy.Publisher('/isFollowing', Bool, queue_size=1)
    pub_isFollowing.publish(False)
    time.sleep(0.1)
    comando.buttons = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] # X
    pub_comando.publish(comando)
    time.sleep(3)
    comando.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] # L1
    pub_comando.publish(comando)
    time.sleep(1)
    comando.buttons = [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] # CERCHIO
    pub_comando.publish(comando)
    time.sleep(1)
    comando.buttons = [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0] # R2
    pub_comando.publish(comando)
    # time.sleep(1)
    print("Robot pronto a muoversi!")
    rospy.spin()  # impedisce semplicemente al nodo di uscire fin quando non viene arrestato


if __name__ == '__main__':
    try:
        print("Path follower attivo!")
        pathFollower()
    except rospy.ROSInterruptException:
        pass

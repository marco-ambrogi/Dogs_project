import math

import tf.transformations
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import rospy
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool
import time

#pub_vel = rospy.Publisher('player0/cmd_vel', Twist, queue_size=1)
pub_path = rospy.Publisher('globalPlanner/path', Path, queue_size=1)
pub_info_goal_irraggiungibile = rospy.Publisher('globalPlanner/info_goal_irraggiungibile', Bool, queue_size=1)
pub_start = rospy.Publisher('user/start', Bool, queue_size=1)

freeSpaceGraph = 0
nodesPath = 0
robot_pose = 0
goal_pose = 0
mapResolution = 0
width = 0
height = 0
inflated_grid_map = 0
graphAvailable = False
msg_vel = Twist()
msg_vel.linear.x = 0.0
msg_vel.angular.z = 0.2

last_goal = None
full_path = Path()
full_path.header.frame_id = "map"
# num_paths = 0
# paths = Path()
# paths.header.frame_id = "map"
# paths = []
home_pose = Pose()
home_pose.position.x = 0.0
home_pose.position.y = 0.0
home_pose.position.z = 0.2
home_pose.orientation.x = 0.0
home_pose.orientation.y = 0.0
home_pose.orientation.z = 0.0
home_pose.orientation.w = 1.0

saved_path = Path()
saved_path.header.frame_id = "map"
repeat_path = False


# rappresentazione mappa modificata con i margini di sicurezza e il percorso del robot
def showMapWithRobot_Goal_Path():
    global nodesPath
    robot_node = findNodeInGraph(robot_pose)
    goal_node = findNodeInGraph(goal_pose)
    robot_in_grid = np.copy(inflated_grid_map)

    # print(goal_node.split("/"))
    y, x = goal_node.split("/")
    # print(goal_node)
    x = int(x)
    y = int(y)
    robot_in_grid[y, x] = 200

    y, x = robot_node.split("/")
    x = int(x)
    y = int(y)
    robot_in_grid[y, x] = 250

    for v in nodesPath:
        y, x = v.split("/")
        x = int(x)
        y = int(y)
        robot_in_grid[y, x] = 150

    # plt.figure(1)
    f1 = plt.figure(num=1, clear=True)
    plt.imshow(np.flipud(robot_in_grid[1830:2350, 1830:2150]))
    plt.title("Inflated GridMap")
    plt.ion()
    plt.pause(0.001)

plt.show()


# creazione safe zone intorno agli ostacoli
def inflation(map, factor):
    width = map.shape[0]
    height = map.shape[1]
    inflated_map = np.copy(map)
    for i in range(height):
        for j in range(width):
            if map[i, j] == 100:
                for h in range(i - factor, i + factor+1):
                    for k in range(j - factor, j + factor+1):
                        if math.floor(math.sqrt(math.pow(h - i, 2) + math.pow(k - j, 2))) <= factor:
                            inflated_map[h, k] = 100
    return inflated_map


# costruisce e rende disponibile il grafo dello spazio libero
def buildFreeSpaceGraph(map_msg):
    global freeSpaceGraph
    global mapResolution
    global height
    global width
    global inflated_grid_map

    safeDist = 50  # cm
    grid_map = np.asarray(map_msg.data)
    height = map_msg.info.height
    width = map_msg.info.width
    mapResolution = map_msg.info.resolution

    # print("Resolution :", resolution)
    grid_map = grid_map.reshape(height, width)

    inflated_grid_map = inflation(grid_map, factor=int(safeDist/(100 * mapResolution)))


    g = nx.Graph()

    for i in range(height):
        for j in range(width):
            if inflated_grid_map[i, j] == 0:
                nameC = str(i) + "/" + str(j)
                # print(nameC)
                g.add_node(nameC)

                if inflated_grid_map[i - 1, j + 1] == 0:
                    nameNE = str(i - 1) + "/" + str(j + 1)
                    # print(nameNE)
                    g.add_node(nameNE)
                    g.add_edge(nameC, nameNE, weight=7.07)
                if inflated_grid_map[i, j + 1] == 0:
                    nameE = str(i) + "/" + str(j + 1)
                    # print(nameE)
                    g.add_node(nameE)
                    g.add_edge(nameC, nameE, weight=5)

                if inflated_grid_map[i + 1, j + 1] == 0:
                    nameSE = str(i + 1) + "/" + str(j + 1)
                    # print(nameSE)
                    g.add_node(nameSE)
                    g.add_edge(nameC, nameSE, weight=7.07)
                if inflated_grid_map[i + 1, j] == 0:
                    nameS = str(i + 1) + "/" + str(j)
                    # print(nameS)
                    g.add_node(nameS)
                    g.add_edge(nameC, nameS, weight=5)

    freeSpaceGraph = g
    print("Grafo dello spazio libero Pronto")

# restituisce l'ID del nodo associato alla posizione
def findNodeInGraph(myPose):
    i = int(height / 2 + myPose.position.y / mapResolution)
    j = int(width / 2 + myPose.position.x / mapResolution)
    return str(i) + "/" + str(j)

# costruisce e pubblica il path nella mappa partendo da un percorso "nodesPath" sul grafo
def rosPathFromList():
    global pub_path, paths, full_path
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()

    lenPath = len(nodesPath)
    u = nodesPath[0]

    dirMatrix = [[-3 * math.pi / 4, -math.pi / 2,  -math.pi / 4],
                 [math.pi, 0, 0],
                 [3*math.pi / 4, math.pi / 2, math.pi / 4]]
    #print("matrice direzioni: \n",dirMatrix)
    orientations = []
    for h in range(0, lenPath - 2):
        u = nodesPath[h]
        v = nodesPath[h + 1]

        iu, ju = u.split("/")
        iv, jv = v.split("/")
        di = int(iv) - int(iu) + 1
        dj = int(jv) - int(ju) + 1
        #print(di, dj)
        orientations.append(dirMatrix[di][dj])
    h=0
    #print("Orientamenti",orientations)
    while h < lenPath - 2:
        u = nodesPath[h]
        iu, ju = u.split("/")

        yu = mapResolution * (int(iu) - height / 2)
        xu = mapResolution * (int(ju) - width / 2)

        p = PoseStamped()
        p.pose.position.x = xu
        p.pose.position.y = yu
        p.pose.position.z = 0
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, orientations[h])
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        p.header.frame_id = "map"
        p.header.seq = h
        p.header.stamp = path.header.stamp
        path.poses.append(p)

        h = h + 1

    # si aggiunge ora l'ultima posa, quella del goal
    h = lenPath - 1
    #print(lenPath, h)
    u = nodesPath[h]
    iu, ju = u.split("/")

    yu = mapResolution * (int(iu) - height / 2)
    xu = mapResolution * (int(ju) - width / 2)

    p = PoseStamped()
    p.pose.position.x = xu
    p.pose.position.y = yu
    p.pose.position.z = 0
    p.pose.orientation = goal_pose.orientation
    p.header.frame_id = "map"
    p.header.seq = h
    p.header.stamp = path.header.stamp
    path.poses.append(p)
    # pubblico il percorso per renderlo disponibile
    # pub_path.publish(path)

    full_path.poses = full_path.poses + path.poses
    pub_path.publish(full_path)

    # paths.append(path)
    # for i in range(len(paths)):
    #     full_path.poses = full_path.poses + paths[i].poses
    # pub_path.publish(full_path)

# trova il percorso dalla posa attuale del robot "robot_pose" alla posa "goal_Pose"
def findShortestPath(goal_pose):
    global freeSpaceGraph
    global robot_pose
    global nodesPath

    nodesPath = 0
    if last_goal != None:
        robot_node = findNodeInGraph(last_goal)
    else:
        robot_node = findNodeInGraph(robot_pose)
    goal_node = findNodeInGraph(goal_pose)

    print("In cerca del Percorso:\n PARTENZA: \n", robot_pose.position, "\n ARRIVO : \n", goal_pose.position)
    #print("Nodes for path from: \n", robot_node, "\n to: \n", goal_node)

    # la ricerca potrebbe non andare a buon fine, generando un'eccezione
    try:
        nodesPath = nx.shortest_path(freeSpaceGraph, robot_node, goal_node, weight="weight",method="dijkstra") # Possibili metodi: 'dijkstra', 'bellman-ford'
    except nx.NodeNotFound:
        print("Percorso inesistente!")
        pub_info_goal_irraggiungibile.publish(True)
    else: print("Percorso Trovato")
    #print(nodesPath)

# se e' stato lanciato il file ROSLaunch, la mappa, il robot e la sua sensoristica e' attiva
def callback_map(map_msg):
    global graphAvailable
    print("Ricevuta GridMap")
    # pub_vel.publish(msg_vel) #impongo un movimento rotazionale al robot per migliorare la stima della sua posa
    buildFreeSpaceGraph(map_msg)
    graphAvailable = True
    # dopo qualche secondo il grafo e' pronto, suppongo sia buona anche la stima della posizione del robot, quindi lo posso fermare
    # pub_vel.publish(Twist())

def callback_pose(pose_msg):
    global robot_pose
    # robot_pose = pose_msg.pose.pose
    robot_pose = pose_msg.pose[len(pose_msg.pose) - 1]
    #print("Robot pose received: \n", robot_pose)

# quando viene fornita una posa obiettivo, cerco il percorso
def callback_goal(goal_msg):
    global goal_pose, last_goal #, num_paths
    # num_paths = num_paths + 1
    goal_pose = goal_msg.pose
    if graphAvailable: # prima era and goal_pose != 0
        findShortestPath(goal_pose)
        #showMapWithRobot_Goal_Path() #decommentare se si vuole vedere la mappa con le zone di sicurezza
        rosPathFromList()
        print("Percorso Disponibile")
    last_goal = goal_pose

# def callback_info_posa_raggiunta(data):
#     global paths, num_paths
#     # paths.pop(0)
#     num_paths = num_paths - 1

def callback_info_goal_raggiunto(data):
    global full_path, saved_path, last_goal
    last_goal = robot_pose
    if repeat_path:
        pub_path.publish(saved_path)
        pub_start.publish(True)
    else:
        full_path.poses = []
        pub_path.publish(full_path)

def callback_back_to_home(data):
    global full_path, last_goal, repeat_path
    repeat_path = False
    last_goal = robot_pose
    full_path.poses = []
    pub_path.publish(full_path)
    time.sleep(0.1)
    goal_pose = home_pose
    if graphAvailable: # prima era and goal_pose != 0
        findShortestPath(goal_pose)
        #showMapWithRobot_Goal_Path() #decommentare se si vuole vedere la mappa con le zone di sicurezza
        rosPathFromList()
        print("Percorso Disponibile")
    last_goal = goal_pose

def callback_save_path(data):
    global saved_path
    saved_path.poses = full_path.poses

def callback_repeat_saved_path(data):
    global repeat_path, saved_path
    repeat_path = data.data
    if repeat_path:
        pub_path.publish(saved_path)



def startGlobalPlanner():
    # inizializzazione nodo
    rospy.init_node('globalPlanner', anonymous=True)
    # ricezione mappa
    rospy.Subscriber('map', OccupancyGrid, callback_map)
    # ricezione posa robot
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback_pose)
    # rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, callback_pose)
    # ricezione posa target
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_goal)
    # rospy.Subscriber('/info_posizione_raggiunta', Pose, callback_info_posa_raggiunta)
    rospy.Subscriber('/info_goal_raggiunto', Bool, callback_info_goal_raggiunto)
    rospy.Subscriber('/user/back_to_home', Bool, callback_back_to_home)

    rospy.Subscriber('/user/save_path', Bool, callback_save_path)
    rospy.Subscriber('/user/repeat_saved_path', Bool, callback_repeat_saved_path)
    rospy.spin()


if __name__ == '__main__':
    try:
        startGlobalPlanner()
    except rospy.ROSInterruptException:
        pass

import math
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import Int32

from sensor_msgs.msg import Joy
comando = Joy()
comando.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
comando.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
from gazebo_msgs.msg import ModelStates
import time

posa = None
misura = None
path = None
goal = None
rate = None
distanze = [1000 for x in range(359)]


def callback_pose(data):
    global posa
    # posa = data  # posa del robot
    posa = data.pose[len(data.pose) - 1]
    # posa = data.pose[3]


def callback_measures(data):
    global misura, distanze
    misura = data
    distanze = misura.ranges  # vettore delle misure laser
    # print(distanze)


def callback_path(data):
    global path
    path = data  # percorso globale


def callback_goal(data):
    global goal
    goal = data


def ricerca_ostacolo_frontale(ang_front):
    global ostacolo_frontale, distanze
    ostacolo_frontale = False
    for i in range(int(180 - ang_front/2), int(180 + ang_front/2)):
        if distanze[i] < soglia:
            # print("C'è un ostacolo di fronte!")
            ostacolo_frontale = True
            break


def ricerca_ostacolo_destra(ang_destra):
    global ostacolo_destra
    ostacolo_destra = False
    for i in range(90, int(90 + ang_destra/2)):
        if distanze[i] < soglia:
            # print("C'è un ostacolo a destra!")
            ostacolo_destra = True
            break


def ricerca_ostacolo_sinistra(ang_sinistra):
    global ostacolo_sinistra
    ostacolo_sinistra = False
    for i in range(int(270 - ang_sinistra/2), 270):
        if distanze[i] < soglia:
            # print("C'è un ostacolo a sinistra!")
            ostacolo_sinistra = True
            break


def giro(ang_destra, ang_sinistra):
    global antiorario
    antiorario = False
    destra = 0
    sinistra = 0
    # for i in range(int(90 - ang_destra/2), int(90 + ang_destra/2)):
    for i in range(int(90), int(180)): # conto i punti liberi a destra
        if distanze[i] > (soglia * 2.5): 
            destra = destra + 1
    # print(destra)
    # for i in range(int(270 - ang_sinistra/2), int(270 + ang_sinistra/2)):
    for i in range(int(180), int(270)): # conto i punti liberi a sinistra
        if distanze[i] > (soglia * 2.5):
            sinistra = sinistra + 1
    # print(sinistra)
    if destra > sinistra:
        antiorario = True


def distanzaTraDuePunti(p1, p2):
    d = math.sqrt(math.pow(float(p1.x) - float(p2.x), 2) + math.pow(float(p1.y) - float(p2.y), 2))
    return d


def wall_following():
    global msg_vel, num, last_pose
    print("Aggiramento ostacolo...")
    giro(angolo_destra, angolo_sinistra)
    if antiorario:
        # msg_vel.angular.z = -v_angolare
        comando.axes[0] = -v_ang # ruota in senso antiorario
        comando.axes[5] = 1.0
    else:
        # msg_vel.angular.z = v_angolare
        comando.axes[0] = v_ang # ruota in senso orario
        comando.axes[5] = 1.0
    # pub_vel.publish(msg_vel)
    pub_comando.publish(comando)
    while ostacolo_frontale:
        ricerca_ostacolo_frontale(angolo_frontale)
        # pub_vel.publish(msg_vel)
        pub_comando.publish(comando)
        rate.sleep()
    # msg_vel.angular.z = 0.0
    # msg_vel.linear.x = v_lineare
    # pub_vel.publish(msg_vel)
    comando.axes[0] = 0.0
    comando.axes[5] = 0.0
    comando.axes[4] = v_lin
    pub_comando.publish(comando)
    
    posa1 = posa
    dist_pg = float("inf")  # distanza tra la posa attuale e un punto del percorso globale
    dist_pp = 0  # distanza tra la posa attuale e posa1

    secondi = time.time()

    while (dist_pg) > 0.1 or (dist_pp < 0.2): # 0.1 0.2
        if ((time.time() - secondi) > 90): # se si sta aggirando l'ostacolo per piu' di 90 secondi probabilmente c'e' un problema e quindi si smette di aggirare l'ostacolo e si prova a puntare verso il goal
            num = path.poses[len(path.poses) - 2].header.seq
            break
        ricerca_ostacolo_frontale(angolo_frontale)
        ricerca_ostacolo_destra(angolo_destra)
        ricerca_ostacolo_sinistra(angolo_sinistra)
        if ostacolo_frontale:
            if antiorario == False:
                # msg_vel.angular.z = v_angolare
                # msg_vel.linear.x = 0.0
                comando.axes[0] = v_ang # ruota in senso orario
                comando.axes[5] = 1.0
                comando.axes[4] = 0.0
            else:
                # msg_vel.angular.z = -v_angolare
                # msg_vel.linear.x = 0.0
                comando.axes[0] = -v_ang # ruota in senso antiorario
                comando.axes[5] = 1.0
                comando.axes[4] = 0.0
        else:
            if antiorario == False:
                if ostacolo_destra:
                    # msg_vel.angular.z = 0.0
                    # msg_vel.linear.x = v_lineare
                    comando.axes[0] = 0.0
                    comando.axes[5] = 0.0
                    comando.axes[4] = v_lin
                else:
                    # msg_vel.angular.z = -v_angolare
                    # msg_vel.linear.x = v_lineare/4
                    comando.axes[0] = -v_ang
                    comando.axes[5] = 1.0
                    comando.axes[4] = v_lin # / 4
            else:
                if ostacolo_sinistra:
                    # msg_vel.angular.z = 0.0
                    # msg_vel.linear.x = v_lineare
                    comando.axes[0] = 0.0
                    comando.axes[5] = 0.0
                    comando.axes[4] = v_lin
                else:
                    # msg_vel.angular.z = v_angolare
                    # msg_vel.linear.x = v_lineare/4
                    comando.axes[0] = v_ang
                    comando.axes[5] = 1.0
                    comando.axes[4] = v_lin # / 4
        # pub_vel.publish(msg_vel)
        pub_comando.publish(comando)
        dist_pp = distanzaTraDuePunti(posa.position, posa1.position)
        for i in range(0, len(path.poses) - 1):
            dist_pg = distanzaTraDuePunti(posa.position, path.poses[i].pose.position)
            if dist_pg <= 0.1:
                num = path.poses[i].header.seq
                break
    # msg_vel = Twist()
    # pub_vel.publish(msg_vel)
    comando.axes[0] = 0.0
    comando.axes[5] = 0.0
    comando.axes[4] = 0.0
    pub_comando.publish(comando)
    last_pose = posa  # posa in cui e' terminato il wall following
    rate.sleep()
    print("Ostacolo superato!")
    print()


def obstacle_avoidance_node():
    global rate, msg_vel, pub_vel, pub_obstacle, pub_num_seq, soglia, last_pose, pub_comando, v_ang, v_lin # v_angolare, v_lineare,
    global angolo_frontale, angolo_destra, angolo_sinistra
    rospy.init_node('obstacle_avoidance', anonymous=True)
    # pub_vel = rospy.Publisher('player0/cmd_vel', Twist, queue_size=1)  # per mandare comandi di velocità al turtlebot
    pub_obstacle = rospy.Publisher('/obstacleAvoidance', Bool, queue_size=1)  # per fermare la navigazione sul percorso globale
    pub_num_seq = rospy.Publisher('/obstacleAvoidance/newNumSeq', Int32, queue_size=1)  # per far riprendere la navigazione sul percorso globale
    # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback_pose)  # per avere la posa corrente del turtlebot stimata con AMCL
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback_pose)
    rospy.Subscriber('/velodyne_scan', LaserScan, callback_measures)  # per avere i dati laser dal turtlebot
    rospy.Subscriber('globalPlanner/path', Path, callback_path)  # per avere il percorso globale
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_goal)  # per avere il goal da rviz

    pub_comando = rospy.Publisher('/a1_joy/joy_ramped', Joy, queue_size=1)

    rate = rospy.Rate(10)
    raggio_turtlebot = 0.5 # 0.25
    soglia = raggio_turtlebot + 0.3
    angolo_frontale = 90
    angolo_destra = angolo_frontale
    angolo_sinistra = angolo_destra

    # v_lineare = 0.1
    # v_angolare = 0.1
    v_lin = 0.5
    v_ang = 0.5
    # msg_vel = Twist()
    # pub_vel.publish(msg_vel)
    comando_ini = Joy()
    comando_ini.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    comando_ini.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    pub_comando.publish(comando_ini)

    while not rospy.is_shutdown():
        while posa == None:  # attesa ricezione posa iniziale:
            pass

        # Inizializzo last_pose con dei valori improbabili perche' rappresentano un punto fuori dalla mappa:
        last_pose = posa
        # last_pose.pose.pose.position.x = 12.0
        # last_pose.pose.pose.position.y = 5.0
        last_pose.position.x = 12.0
        last_pose.position.y = 5.0

        while goal == None:  # attesa ricezione punto di destinazione:
            pass

        while True:
            ricerca_ostacolo_frontale(angolo_frontale)
            # dist_posa_last = distanzaTraDuePunti(posa.pose.pose.position, last_pose.pose.pose.position)
            # dist_posa_goal = distanzaTraDuePunti(posa.pose.pose.position, goal.pose.position)
            dist_posa_last = distanzaTraDuePunti(posa.position, last_pose.position)
            dist_posa_goal = distanzaTraDuePunti(posa.position, goal.pose.position)
            if (ostacolo_frontale) and (dist_posa_last > 0.2) and (dist_posa_goal > 0.15): # 0.2 0.15
                pub_obstacle.publish(True)
                # msg_vel.linear.x = 0.0
                # pub_vel.publish(msg_vel)
                comando.axes[0] = 0.0
                comando.axes[5] = 0.0
                comando.axes[4] = 0.0
                pub_comando.publish(comando)
                rate.sleep()
                pub_obstacle.publish(True)
                # pub_vel.publish(msg_vel)
                comando.axes[0] = 0.0
                comando.axes[5] = 0.0
                comando.axes[4] = 0.0
                pub_comando.publish(comando)
                wall_following()
                # msg_vel = Twist()
                # pub_vel.publish(msg_vel)
                comando.axes[0] = 0.0
                comando.axes[5] = 0.0
                comando.axes[4] = 0.0
                pub_comando.publish(comando)
                pub_num_seq.publish(num)
                rate.sleep()
                pub_num_seq.publish(num)
                pub_obstacle.publish(False)


if __name__ == '__main__':
    try:
        print("Rilevamento ostacoli attivo!")
        print()
        obstacle_avoidance_node()
    except rospy.ROSInterruptException:
        pass

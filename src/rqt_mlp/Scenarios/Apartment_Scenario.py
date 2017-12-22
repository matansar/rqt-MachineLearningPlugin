import os
import subprocess
import time
import random
import rospy
from move_base_msgs.msg import MoveBaseActionResult
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped


SLEEPING_TIME = 15 ### related to computer's force
rosbag_process = None
start_scenarios_time = None
logging_msg = None


class area: 
  def __init__(self, x_min, x_max, y_min, y_max):
    self.x_min = x_min 
    self.x_max = x_max
    self.y_min = y_min
    self.y_max = y_max



def Run_Scenario(scen_obj, source_x, source_y, angle, world = "building.world", mapping = "building.yaml"):
    global start_scenarios_time, logging_msg
    source_x, source_y, angle = float(source_x), float(source_y), float(angle)
    start_scenarios_time = rospy.Time.now().to_sec()
    logging_msg = "source x = %s, source y = %s, angle = %s, mapping = %s, world = %s" % (round(source_x,2), round(source_y,2), round(angle,4), mapping, world)
    ros_launch = "roslaunch robotican_armadillo armadillo.launch kinect2:=true lidar:=true move_base:=true " \
		 "moveit:=false use_sim_time:=true robot_localization:=true arm:=false controllers:=true " \
                 "amcl:=true hector_slam:=false gmapping:=false gazebo:=true have_map_file:=true map_file:=\"`rospack find rqt_mlp`/src/rqt_mlp/Scenarios/Extentions/maps/%s\"  world_name:=\"`rospack find rqt_mlp`/src/rqt_mlp/Scenarios/Extentions/worlds/%s\" " % (mapping ,world)

    #location = "x:=%s y:=%s Y:=%s" % (0,0,0)
    location = "x:=%s y:=%s Y:=%s" % (source_x, source_y, angle)
    launch_cmd = ros_launch + location
    subprocess.Popen(launch_cmd, shell=True)
    time.sleep(SLEEPING_TIME)
    run_rviz()
    apply_diagnostic()
    apply_simulation(scen_obj)
    #record_start(scen_obj)
    scen_obj.generate_bag()

def record_start(scen_obj):
  topics = scen_obj.get_topics()
  rosbag_cmd = "rosbag record " + reduce(lambda acc,curr: acc + " " + curr, topics, " ")
  return subprocess.Popen(rosbag_cmd, shell=True)

def record_end(scen_obj):
  import rosnode, rosgraph
  nodes = rosnode.get_nodes_by_machine(rosgraph.network.get_host_name())
  record_node = filter(lambda node_name: "record" in node_name, nodes)[0]
  rosbag_cmd = "rosnode kill " + record_node
  subprocess.Popen(rosbag_cmd, shell=True)
  

def run_rviz():
    rviz = "rosrun rviz rviz" #-d /home/lab/dwa.rviz"
    subprocess.Popen(rviz, shell=True)

def apply_diagnostic():
    ros_profiler = "rosrun rosdiagnostic rosdiagnostic"
    subprocess.Popen(ros_profiler, shell=True)
    pass
  
def logging_params(logging_msg):
    import inspect
    import logging
    logging_file = os.path.dirname(
        os.path.abspath(inspect.getfile(inspect.currentframe()))) + "/Extentions/log/scenarios_logger.log"
    logger = logging.getLogger("scenarios_logger")

    hdlr = logging.FileHandler(logging_file)
    formatter = logging.Formatter('%(asctime)s %(message)s')
    hdlr.setFormatter(formatter)
    logger.addHandler(hdlr)
    logger.info(logging_msg)

 
next_goal = 1
waiting = None
def when_arrived(msg, args):
  global next_goal, logging_msg, start_scenarios_time, waiting
  print "entered entered entered enteredentered entered entered "
  scen_obj, goals = args[0], args[1]
  if msg.status.status == 3 and next_goal >= len(goals): #end
    end_time_scenarios = rospy.Time.now().to_sec()
    logging_params(logging_msg + ", duration = %s" % (round(end_time_scenarios - start_scenarios_time,2)))
    waiting.unregister()
    #pub.unregister()
    time.sleep(2)
    scen_obj.close_bag()
    #record_end(scen_obj)
    #time.sleep(10)
    print "shut down"
  elif msg.status.status == 3:
    publish(create_goal_msg(goals[next_goal], next_goal))
    #pub.publish(create_goal_msg(goals[next_goal], next_goal))
    next_goal = next_goal + 1
    print next_goal
    

#def create_simple_goal_msg(goal, seq):
  #x = goal[0]
  #y = goal[1]
  #msg = PoseStamped()
  #msg.header.seq = seq
  #msg.header.stamp = rospy.Time.now()
  #msg.header.frame_id = 'map'
  #msg.pose.position.x = x
  #msg.pose.position.y = y
  #return msg
  
def create_goal_msg(goal, seq):
  import math, random, tf
  x = goal[0]
  y = goal[1]
  orient = tf.transformations.quaternion_from_euler(0, 0, random.uniform(0, 2 * math.pi))
  goal = "\'{ header: { stamp: now, frame_id: \"/map\" }, " \
	   "goal: { target_pose: { header:{ seq: %s, stamp: now, frame_id: \"/map\" }, pose: " \
	   "{ position: {x: %s, y: %s}, orientation: { x: %s, y: %s, z: %s, w: %s} } } } }\'" % (seq, x, y, orient[0], orient[1], orient[2], orient[3])
  return goal
  #msg = MoveBaseActionGoal()
  #msg.header.seq = seq
  #t = rospy.Time() #.now()
  #msg.header.stamp = t
  #msg.header.frame_id = ''
  #msg.goal.target_pose.header.seq = seq
  #msg.goal.target_pose.header.stamp = t
  #msg.goal.target_pose.header.frame_id = 'map'
  #msg.goal.target_pose.pose.position.x = x
  #msg.goal.target_pose.pose.position.y = y
  #orient = tf.transformations.quaternion_from_euler(0, 0, random.uniform(0, 2 * math.pi))
  #msg.goal.target_pose.pose.orientation.x = orient[0]
  #msg.goal.target_pose.pose.orientation.y = orient[1]
  #msg.goal.target_pose.pose.orientation.z = orient[2]
  #msg.goal.target_pose.pose.orientation.w = orient[3]
  ##print msg.goal.target_pose.pose.orientation.x
  ##print msg.goal.target_pose.pose.orientation.y
  ##print msg.goal.target_pose.pose.orientation.z
  ##print msg.goal.target_pose.pose.orientation.w
  ##time.sleep(20)
  ##msg.goal.target_pose.pose.orientation.z = 1
  #return msg
  
def publising_goals(scen_obj, goals):
  import time
  global waiting
  #print "new new new new new new"
  #pub = None #rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
  waiting = rospy.Subscriber('/move_base/result', MoveBaseActionResult, when_arrived, (scen_obj, goals))
  msg = create_goal_msg(goals[0],0)
  time.sleep(2)
  #pub.publish(msg)
  publish(msg)

def create_goals(areas):
  goals = []
  for area in areas:
    goal_x = random.uniform(area.x_min, area.x_max)
    goal_y = random.uniform(area.y_min, area.y_max)
    goals.append((goal_x, goal_y))
  return goals

def publish(msg):
  goal = "rostopic pub /move_base/goal move_base_msgs/MoveBaseActionGoal " + msg
  subprocess.Popen(goal, shell=True)
  print "A Goal is Published: " + str(msg)
  
def apply_simulation(scen_obj):
    import threading
    from multiprocessing import Process

    print "simulation started..."
    area_1 = area(3.63597559929, 4.56262493134, -1, -0.1)	
    area_2 = area(-5.22181129456,-4.34186172485,-0.946628332138,-0.340723633766)
    #area_3 = area(-8.187541008, -7.81834888458, -1.72615361214, -0.75048917532)
    area_3 = area(-5.32811450958,-3.61236262321,7.99595689774,8.32002067566)
    area_4 = area(-0.504257440567, -0.4 ,6.49937057495 , 7.66467809677)
    areas = [area_1,area_2,area_3,area_4]
    random.shuffle(areas)
    goals = create_goals(areas)
    publising_goals(scen_obj, goals)
    #thread = threading.Thread(target = publising_goals, args = (scen_obj, goals))
    #thread.start()

    #p = Process(target=publising_goals, args= (scen_obj, goals))
    #p.start()
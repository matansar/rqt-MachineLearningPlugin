import os
import subprocess
import time
import rospy
from move_base_msgs.msg import MoveBaseActionResult

def Run_Scenario(scen_obj, source_x, source_y, angle, distance, world = "empty.world"):
    source_x, source_y, angle, distance = float(source_x), float(source_y), float(angle), float(distance)
    ros_launch = "roslaunch robotican_armadillo armadillo.launch lidar:=true move_base:=true " \
                 "gmapping:=true gazebo:=true gui:=true world_name:=\"`rospack find rqt_mlp`/src/rqt_mlp/Scenarios/Extentions/worlds/%s\" " % world

    location = "x:=%s y:=%s Y:=%s" % (source_x, source_y, angle)
    launch_cmd = ros_launch + location
    subprocess.Popen(launch_cmd, shell=True)
    time.sleep(10)
    apply_statistics()
    apply_simulation(scen_obj, distance)
    scen_obj.generate_bag()

def apply_statistics():
    enable_statistics = "rosparam set enable_statistics true"
    ros_profiler = "rosrun rosprofiler rosprofiler"
    subprocess.Popen(enable_statistics, shell=True)
    subprocess.Popen(ros_profiler, shell=True)

def apply_simulation(scen_obj, distance):
    goal = "rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \'{ header: { frame_id: \"/map\" }, " \
           "pose: { position: { x: %s, y: %s }, orientation: { x: 0, y: 0, z: 0, w: 1 } } }\'" % (distance, 0)
    subprocess.Popen(goal, shell=True)
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, is_arrived, scen_obj)

def is_arrived(msg, scen_obj):
    import inspect, os
    if msg.status.status == 3:
        # TODO need to be removed
        # time.sleep(2)
        scen_obj.close_bag()

        # restart_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + "/Extentions/scripts/restart.sh"
        # subprocess.Popen(restart_path, shell=True)

        # roscore_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + "/Extentions/scripts/roscore.sh"
        # subprocess.Popen(roscore_path, shell=True)
        # close_stat = "killall rosrun"
        # close_stat = "killall rosrun"
        # close_rostopic = "killall rostopic"
        # close = "killall roslaunch"
        #  TODO delete the function and the arguments
        # subprocess.Popen(close_stat, shell=True)
        # subprocess.Popen(close_rostopic, shell=True)
        # subprocess.Popen(close, shell=True)

        #while True:
        print "shut down"




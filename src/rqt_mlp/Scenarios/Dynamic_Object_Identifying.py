import os
import subprocess
import time
import rospy
from move_base_msgs.msg import MoveBaseActionResult

def Run_Scenario(scen_obj, source_x, source_y, angle, distance):
    source_x, source_y, angle, distance = float(source_x), float(source_y), float(angle), float(distance)
    world = "object_identifying.world"
    obj = "box"
    obj_x, obj_y = calc_obj_loaction(source_x, source_y, angle, distance)
    apply_obj_world(world, obj, obj_x, obj_y)
    apply_scenario(scen_obj, source_x, source_y, angle, distance, world)

def calc_obj_loaction(source_x, source_y, angle, distance):
    import math
    goal_x = source_x + distance * math.cos(angle)
    goal_y = source_y + distance * math.sin(angle)
    obj_x = (source_x + goal_x)
    obj_y = (source_y + goal_y)
    if obj_x > 0:
      obj_x = obj_x -0.5
    if obj_y > 0:
      obj_y = obj_y -0.5
    return obj_x, obj_y

def apply_obj_world(world, obj, obj_x,obj_y):
    import inspect, os
    world_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + "/Extentions/worlds/%s" % world
    model_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + "/Extentions/models/%s/%s.sdf" % (obj,obj)
    obj_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + "/Extentions/models/%s" % obj
    pose = "%s %s -0.34 0 0 0" % (obj_x, obj_y)
    write_to_xml(world_path, './/include/uri', obj_path)
    write_to_xml(world_path, './/include/pose', pose)
    write_to_xml(model_path, './/script/uri', obj_path + "/material/scripts")
    write_to_xml(model_path, './/script/uri', obj_path + "/material/textures", 1)


def write_to_xml(path_file, xml_path, data, location = 0):
    from xml.etree import ElementTree as ET
    tree = ET.parse(path_file)
    print tree
    (tree.findall(xml_path))[location].text = data
    tree.write(path_file)

def apply_scenario(scen_obj, source_x, source_y, angle, distance, world):
    import Source_Dest_Scenario as sds
    sds.Run_Scenario(scen_obj, source_x, source_y, angle, distance, world)
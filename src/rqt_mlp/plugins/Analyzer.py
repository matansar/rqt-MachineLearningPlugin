import os
import glob
import json
import time

from ExtractFeatures import *
from AnomalyDetection import *

class Analyzer(object):

    def __init__(self, dir_path, time_frame, threshold, topics, rules_file_path):
        self.time_frame = time_frame
        self.dir_path = dir_path
        self.rules_file_path = rules_file_path
        self.threshold = threshold
        self.processed_files = []
        self.features_extractor = ExtractFeatures(topics, (time_frame-0.1)/time_frame, get_specific_features_options(), get_general_features_options())
        self.predictions = []

    """
        Return a list of .bag files sorted by creation time
    """
    def get_files_by_creation_time(self):
        files = filter(os.path.isfile, glob.glob(self.dir_path + "/*.bag"))
        files.sort(key=lambda x: os.path.getmtime(x))
        return files

    """
        Return the newest .bag file that have not been processed yet
    """
    def get_next_file(self):
        files = self.get_files_by_creation_time()
        for file in files :
            if not file in self.processed_files:
                return file
        return None


    def get_bag_prediction(self, file):
        df = self.features_extractor.generate_features(file)
        print "#######################Testing the following file : {0}#######################3".format(file)
        write_to_csv("{0}.csv".format(file), df)
        with open(self.rules_file_path, "rb") as f:
            raw_rules = f.read()
        rules = json.loads(raw_rules)
        prediction = apply_rba(df, rules)
        self.predictions += prediction
        print "################ Prediction : {0} ################".format(prediction)
        print "################ All predictions : {0} ################".format(self.predictions)
        print "################ Longest invalid prediction : {0} ################".format(self.get_longest_invalid_seq_length())

    def get_longest_invalid_seq_length(self):
        ret = 0
        for i in range(len(self.predictions)):
            count = 0
            if self.predictions[i] != Conf.NEGATIVE_LABEL:
                continue
            count += 1
            for j in range(i + 1, len(self.predictions)):
                if self.predictions[j] != Conf.NEGATIVE_LABEL:
                    break
                count += 1
            ret = max(ret, count)
        return ret

    def handle_invalid_file(self, file):
        self.processed_files += [file] # TODO : Delete this line if exception is not raised
        raise Exception("A feature vector longer than the threshold was found,"
                        "stopping the current execution. Predictions : {0}".format(self.predictions))


    """
        Manages the process of updating the feature vector and testing it's validity
    """
    def analyze(self):
        while True :
            current_file = self.get_next_file()
            if current_file is None:
                print("#################No file to process####################")
                time.sleep(self.time_frame / 2)
                continue
            print("###################Processing the following file : {0}#################".format(current_file))
            self.get_bag_prediction(current_file)
            if self.get_longest_invalid_seq_length() > self.threshold :
                self.handle_invalid_file(current_file)
            self.processed_files += [current_file]
            time.sleep(self.time_frame / 2)


if __name__ == "__main__":
    topics = ['/map_metadata', '/move_base/local_costmap/footprint', '/move_base/feedback', '/place/result', '/move_base/global_costmap/static/parameter_descriptions', '/scan', '/move_group/sense_for_plan/parameter_updates', '/kinect2/parameter_descriptions', '/move_base/NavfnROS/plan', '/move_base/status', '/move_group/ompl/parameter_updates', '/robot_state', '/place/feedback', '/pickup/status', '/move_base/global_costmap/footprint', '/move_group/result', '/move_base/DWAPlannerROS/global_plan', '/statistics', '/arm_trajectory_controller/follow_joint_trajectory/cancel', '/move_group/status', '/kinect2/parameter_updates', '/kinect2/qhd/image_color/theora/parameter_descriptions', '/poseupdate', '/pan_tilt_trajectory_controller/follow_joint_trajectory/status', '/move_base/current_goal', '/pan_tilt_trajectory_controller/command', '/move_group/planning_scene_monitor/parameter_updates', '/pickup/result', '/gripper_controller/gripper_cmd/goal', '/host_diagnostic', '/move_base/local_costmap/costmap', '/move_group/trajectory_execution/parameter_descriptions', '/move_group/display_planned_path', '/nav_vel', '/gazebo/model_states', '/pan_tilt_trajectory_controller/point_head_action/result', '/mobile_base_controller/odom', '/move_base/parameter_updates', '/move_group/monitored_planning_scene', '/move_group/trajectory_execution/parameter_updates', '/gazebo/parameter_descriptions', '/kinect2/qhd/image_color/compressedDepth/parameter_updates', '/move_base/DWAPlannerROS/cost_cloud', '/move_base/local_costmap/parameter_updates', '/move_base/goal', '/move_base/global_costmap/inflation_global/parameter_updates', '/node_diagnostic', '/move_base/global_costmap/static/parameter_updates', '/move_group/plan_execution/parameter_updates', '/move_base/DWAPlannerROS/parameter_updates', '/gripper_controller/gripper_cmd/result', '/gripper_controller/current_gap', '/arm_trajectory_controller/follow_joint_trajectory/feedback', '/tf', '/move_base/DWAPlannerROS/trajectory_cloud', '/kinect2/qhd/image_color/compressed/parameter_updates', '/move_group/ompl/parameter_descriptions', '/slam_gmapping/entropy', '/move_base/local_costmap/inflation/parameter_descriptions', '/arm_trajectory_controller/follow_joint_trajectory/result', '/move_base/local_costmap/costmap_updates', '/arm_trajectory_controller/follow_joint_trajectory/status', '/move_base/local_costmap/parameter_descriptions', '/move_base/DWAPlannerROS/parameter_descriptions', '/tf_static', '/move_base/local_costmap/obstacles_laser/parameter_descriptions', '/diagnostics', '/cmd_vel', '/pickup/feedback', '/move_base/global_costmap/inflation_global/parameter_descriptions', '/kinect2/qhd/image_color/compressedDepth/parameter_descriptions', '/move_group/feedback', '/joint_states', '/place/status', '/gripper_controller/gripper_cmd/cancel', '/kinect2/qhd/image_color/compressed/parameter_descriptions', '/arm_trajectory_controller/follow_joint_trajectory/goal', '/pan_tilt_trajectory_controller/follow_joint_trajectory/feedback', '/move_group/sense_for_plan/parameter_descriptions', '/move_base/global_costmap/parameter_updates', '/move_base/global_costmap/costmap', '/move_group/display_contacts', '/move_base/local_costmap/obstacles_laser/parameter_updates', '/gazebo/parameter_updates', '/kinect2/qhd/image_color/compressed', '/move_base/parameter_descriptions', '/mobile_base_controller/cmd_vel', '/gripper_controller/gripper_cmd/feedback', '/rosout_agg', '/pan_tilt_trajectory_controller/follow_joint_trajectory/result', '/slam_cloud', '/clock', '/pan_tilt_trajectory_controller/point_head_action/feedback', '/execute_trajectory/result', '/move_base/local_costmap/inflation/parameter_updates', '/rosout', '/move_group/plan_execution/parameter_descriptions', '/execute_trajectory/feedback', '/slam_out_pose', '/execute_trajectory/status', '/move_base/global_costmap/parameter_descriptions', '/kinect2/qhd/image_color/theora/parameter_updates', '/pan_tilt_trajectory_controller/state', '/kinect2/qhd/camera_info', '/arm_trajectory_controller/state', '/move_group/planning_scene_monitor/parameter_descriptions', '/gripper_controller/gripper_cmd/status', '/move_base/result', '/move_base/DWAPlannerROS/local_plan', '/pan_tilt_trajectory_controller/point_head_action/status']
    analyzer = Analyzer("/home/lab/bags/18", 1, 3, topics, "/home/lab/thesis/software/itamar.json")
    analyzer.analyze()

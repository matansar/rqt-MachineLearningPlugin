import os
import glob
import json
import time
import logging

from ExtractFeatures import *
# from AnomalyDetection import *
import RuleBasedAnomaly as RBA
from Style import Configure as Conf

logger = logging.getLogger("Analyzer")

class Analyzer(object):

    TIME_LOWER_BOUND = 1.75

    def __init__(self, dir_path, time_frame, threshold, topics, rules_file_path):
        self.time_frame = time_frame
        self.dir_path = dir_path
        self.rules_file_path = rules_file_path
        self.threshold = threshold
        self.processed_files = []
        self.features_extractor = ExtractFeatures(topics, time_frame, get_specific_features_options(), get_general_features_options())
        self.predictions = []
        self.time_passed = 0
        self.rba = {}
        self.stop = True
        self.old_predictions = []

        # Defined Logging
        hdlr = logging.FileHandler('/var/tmp/analyzer.log')
        formatter = logging.Formatter('%(asctime)s %(levelname)s %(message)s')
        hdlr.setFormatter(formatter)
        logger.addHandler(hdlr)

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


    def remove_rows_from(self, delete, *datasets_types):
        def remove_from(df):
            return df[delete: -delete]
        if delete == 0:
            return datasets_types
        ret = []
        for dfs in datasets_types:
            ret.append(map(lambda df: remove_from(df), dfs))
        return ret

    def get_bag_prediction(self, file, rules):
        df = self.features_extractor.generate_features(file)
        if df.empty:
            return
        print "#######################Testing the following file : {0}#######################".format(file)
        logger.info("#######################Testing the following file : {0}#######################".format(file))
        df = df.reset_index(drop=True)
        #if not self.processed_files:
        # try:
        #     df = df.drop(df.index[:5])
        #     df = df.drop(df.index[-5:])
        # except Exception as e:
        #     pass
        # write_to_csv("{0}.csv".format(file), df)
        # print "#######################Bag was converted to CSV #######################"
        try:
            prediction = self.apply_rba(self.rba, df, rules)
            self.predictions += prediction
        except Exception as e:
            print "################### Exception message : {0} ###############".format(str(e))
            logger.warning("################### Exception message : {0} ###############".format(str(e)))
        print "################ Prediction : {0} ################".format(prediction)
        logger.info("################ Prediction : {0} ################".format(prediction))
        print "################ All predictions : {0} ################".format(self.predictions)
        logger.info("################ All predictions : {0} ################".format(self.predictions))
        print "################ Longest invalid prediction : {0} ################".format(self.get_longest_invalid_seq_length())
        logger.info("################ Longest invalid prediction : {0} ################".format(self.get_longest_invalid_seq_length()))

    def apply_rba(self, rba, dataset, rules):
        type = Conf.TRAINING
        # new_datasets = []

        rules_functions = [rba.transform_same_digits_number,
                           rba.transform_positive_values,
                           rba.transform_negative_values,
                           rba.transform_not_negative_values,
                           rba.transform_coverage_percentage_columns,
                           rba.transform_exactly_one_value,
                           rba.transform_corresponding_columns]
        pred = 1
        for rule_function in rules_functions:
            pred = rule_function(type, dataset)
            if pred[0] == 0:
                print "Failed on {0}".format(rule_function.__name__)
                break

        # pred = rba.transform_same_digits_number(type, dataset, rules)
        # pred = intersection_labeling(pred, rba.transform_positive_values(type, dataset, rules))
        # pred = intersection_labeling(pred, rba.transform_negative_values(type, dataset, rules))
        # pred = intersection_labeling(pred, rba.transform_not_negative_values(type, dataset, rules))
        # pred = intersection_labeling(pred, rba.transform_coverage_percentage_columns(type, dataset, rules))
        # #pred = intersection_labeling(pred, rba.transform_corresponding_columns(type, dataset))
        # pred = intersection_labeling(pred, rba.transform_exactly_one_value(type, dataset, rules))
        # # removable_cols = rba.get_removable_columns()
        # #new_datasets.append(DS.Datasets.remove_columns(dataset, removable_cols))
        return pred

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
        # raise Exception("A feature vector longer than the threshold was found,"
        #                 "stopping the current execution. Predictions : {0}".format(self.predictions))


    """
        Manages the process of updating the feature vector and testing it's validity
    """
    def analyze(self):
        with open(self.rules_file_path, "rb") as f:
            raw_rules = f.read()
        rules = json.loads(raw_rules)
        self.rba = RBA.RuleBasedAnomaly()
        self.rba.fit(rules)
        while self.stop:
            current_file = self.get_next_file()
            if current_file is None:
                print("#################No file to process####################")
                time.sleep(self.time_frame / 2)
                if self.old_predictions:
                    print self.old_predictions
                continue
            self.time_passed += self.time_frame
            if self.time_passed < Analyzer.TIME_LOWER_BOUND:
                self.processed_files += [current_file]
                time.sleep(self.time_frame / 2)
                continue
            print("###################Processing the following file : {0}#################".format(current_file))
            self.get_bag_prediction(current_file, rules)
            if self.get_longest_invalid_seq_length() > self.threshold:
                self.handle_invalid_file(current_file)
                # self.stop = False
                self.old_predictions.append(self.predictions)
                self.predictions = []
                print "A feature vector longer than the threshold was found, stopping the current execution. Predictions : {0}".format(self.predictions)
                logger.info("A feature vector longer than the threshold was found, stopping the current execution. Predictions : {0}".format(self.predictions))
            self.processed_files += [current_file]
            time.sleep(self.time_frame / 2)


if __name__ == "__main__":
    topics = ['/clock', '/diagnostics', '/gazebo/model_states', '/gazebo/parameter_descriptions', '/gazebo/parameter_updates', '/gripper_controller/gripper_cmd/status', '/host_diagnostic', '/joint_states', '/kinect2/parameter_descriptions', '/kinect2/parameter_updates', '/kinect2/qhd/camera_info', '/kinect2/qhd/image_color/compressed', '/kinect2/qhd/image_color/compressed/parameter_descriptions', '/kinect2/qhd/image_color/compressed/parameter_updates', '/kinect2/qhd/image_color/compressedDepth/parameter_descriptions', '/kinect2/qhd/image_color/compressedDepth/parameter_updates', '/kinect2/qhd/image_color/theora', '/kinect2/qhd/image_color/theora/parameter_descriptions', '/kinect2/qhd/image_color/theora/parameter_updates', '/map_metadata', '/mobile_base_controller/cmd_vel', '/mobile_base_controller/odom', '/move_base/DWAPlannerROS/parameter_descriptions', '/move_base/DWAPlannerROS/parameter_updates', '/move_base/global_costmap/costmap', '/move_base/global_costmap/footprint', '/move_base/global_costmap/inflation_global/parameter_descriptions', '/move_base/global_costmap/inflation_global/parameter_updates', '/move_base/global_costmap/parameter_descriptions', '/move_base/global_costmap/parameter_updates', '/move_base/global_costmap/static/parameter_descriptions', '/move_base/global_costmap/static/parameter_updates', '/move_base/local_costmap/costmap', '/move_base/local_costmap/costmap_updates', '/move_base/local_costmap/footprint', '/move_base/local_costmap/inflation/parameter_descriptions', '/move_base/local_costmap/inflation/parameter_updates', '/move_base/local_costmap/obstacles_laser/parameter_descriptions', '/move_base/local_costmap/obstacles_laser/parameter_updates', '/move_base/local_costmap/parameter_descriptions', '/move_base/local_costmap/parameter_updates', '/move_base/parameter_descriptions', '/move_base/parameter_updates', '/move_base/status', '/move_group/monitored_planning_scene', '/move_group/ompl/parameter_descriptions', '/move_group/ompl/parameter_updates', '/move_group/plan_execution/parameter_descriptions', '/move_group/plan_execution/parameter_updates', '/move_group/planning_scene_monitor/parameter_descriptions', '/move_group/planning_scene_monitor/parameter_updates', '/move_group/sense_for_plan/parameter_descriptions', '/move_group/sense_for_plan/parameter_updates', '/move_group/status', '/move_group/trajectory_execution/parameter_descriptions', '/move_group/trajectory_execution/parameter_updates', '/my_find_object/bw/compressed', '/my_find_object/bw/compressed/parameter_descriptions', '/my_find_object/bw/compressedDepth/parameter_descriptions', '/my_find_object/bw/compressedDepth/parameter_updates', '/my_find_object/bw/theora/parameter_descriptions', '/my_find_object/hsv_filterd/compressed', '/my_find_object/hsv_filterd/compressed/parameter_descriptions', '/my_find_object/hsv_filterd/compressed/parameter_updates', '/my_find_object/hsv_filterd/compressedDepth/parameter_descriptions', '/my_find_object/hsv_filterd/compressedDepth/parameter_updates', '/my_find_object/hsv_filterd/theora', '/my_find_object/hsv_filterd/theora/parameter_descriptions', '/my_find_object/hsv_filterd/theora/parameter_updates', '/my_find_object/result/compressed/parameter_descriptions', '/my_find_object/result/compressed/parameter_updates', '/my_find_object/result/compressedDepth/parameter_descriptions', '/my_find_object/result/compressedDepth/parameter_updates', '/my_find_object/result/theora', '/my_find_object/result/theora/parameter_descriptions', '/my_find_object/result/theora/parameter_updates', '/node_diagnostic', '/pan_tilt_trajectory_controller/follow_joint_trajectory/status', '/pan_tilt_trajectory_controller/point_head_action/status', '/pan_tilt_trajectory_controller/state', '/poseupdate', '/scan', '/slam_cloud', '/slam_gmapping/entropy', '/slam_out_pose', '/statistics', '/tf', '/my_find_object/bw/compressed/parameter_updates', '/my_find_object/bw/theora/parameter_updates']
    analyzer = Analyzer("/home/lab/bags/online_test", 0.25, 6, topics, "/home/lab/thesis/software/rules.json")
    analyzer.analyze()

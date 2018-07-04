import os
import glob
import time

from ExtractFeatures import *
from AnomalyDetection import *

class Analyzer(object):

    def __init__(self, dir_path, time_frame, threshold, topics):
        self.time_frame = time_frame
        self.dir_path = dir_path
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
        prediction = apply_rba(df)
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
                        "stopping the current execution")


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
            print("###################Processing the following file : {0}#################3".format(current_file))
            self.get_bag_prediction(current_file)
            if self.get_longest_invalid_seq_length() > self.threshold :
                self.handle_invalid_file(current_file)
            self.processed_files += [current_file]
            time.sleep(self.time_frame / 2)

"""
if __name__ == "__main__":
    analyzer = Analyzer("/home/lab/bags/online", 5, 3)
    analyzer.analyze()
"""
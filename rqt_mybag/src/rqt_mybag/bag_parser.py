import ExtractFeatures as E
from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtWidgets import QMessageBox, QLineEdit, QLabel, QFileDialog, QWidget, QVBoxLayout, QCheckBox, QScrollArea, QPushButton
from MyQCheckBox import MyQCheckBox
import logging

#save choose of user
logger_topic = logging.getLogger("logger")

class BagParser(QWidget):

    def __init__(self, bag_files, listtopics, duration):
        super(BagParser, self).__init__()

        # window title
        self.setWindowTitle("Making csv file")
        # size of window
        self.resize(960, 720)
        #self.showFullScreen()
        #self.setWindowState(Qt.WindowMaximized)

        # print listtopics
        # print E.get_general_features_options()
        # print E.get_specific_features_options()

        self.topics_items = dict()
        self.topics_items["0"] = listtopics
        self.topics_items["1"] = E.get_general_features_options()
        self.topics_items["2"] = E.get_specific_features_options()

        print self.topics_items

        #path to bag file
        self.bag_files = bag_files

        self.selected_bag_topics = []
        self.selected_specific_features = []
        self.selected_general_features = []

        self.items_list_topics = []

        self.area = QScrollArea(self)
        self.areagen = QScrollArea(self)
        self.areaspec = QScrollArea(self)
        self.main_widget = QWidget(self.area)
        self.main_widget1 = QWidget(self.areagen)
        self.main_widget2 = QWidget(self.areaspec)
        self.ok_button = QPushButton("Export To CSV", self)
        #self.ok_button.setFixedSize(150, 30)
        self.ok_button.clicked.connect(self.onButtonClicked)
        self.choose_button = QPushButton("Get Last Export Choose", self)
        self.choose_button.clicked.connect(self.onButtonChooseCliked)
        self.ok_button.setEnabled(False)

        self.label1 = QLabel("Choose topic from bag file", self)
        self.label1.setAlignment(Qt.AlignCenter)

        self.label2 = QLabel("Choose statistics features on the above topics", self)
        self.label2.setAlignment(Qt.AlignCenter)

        self.label3 = QLabel("Choose more specific features on the above topics", self)
        self.label3.setAlignment(Qt.AlignCenter)

        self.label4 = QLabel("Window time", self)
        self.label4.setAlignment(Qt.AlignCenter)

        self.duration = duration

        self.label5 = QLabel("Duration Time: " + str("%.1f" % duration), self)
        self.label5.setAlignment(Qt.AlignCenter)

        self.main_vlayout = QVBoxLayout(self)
        self.main_vlayout.addWidget(self.label1)
        self.main_vlayout.addWidget(self.area)
        self.main_vlayout.addWidget(self.label2)
        self.main_vlayout.addWidget(self.areagen)
        self.main_vlayout.addWidget(self.label3)
        self.main_vlayout.addWidget(self.areaspec)
        self.main_vlayout.addWidget(self.choose_button)
        self.main_vlayout.addWidget(self.label4)
        self.window = QLineEdit(self)
        self.main_vlayout.addWidget(self.window)
        self.window.setText("1")
        self.main_vlayout.addWidget(self.label5)
        self.main_vlayout.addWidget(self.ok_button)
        #self.main_vlayout.addWidget(self.from_nodes_button)
        self.setLayout(self.main_vlayout)

        self.selection_vlayout = QVBoxLayout(self)
        self.item_all = MyQCheckBox("All", self, self.selection_vlayout, None)
        self.item_all.stateChanged.connect(lambda x: self.updateList(x, self.item_all, None))
        self.selection_vlayout.addWidget(self.item_all)
        topic_data_list = listtopics
        topic_data_list.sort()
        for topic in topic_data_list:
            self.addCheckBox(topic, self.selection_vlayout, self.selected_bag_topics)

        self.selection_vlayout1 = QVBoxLayout(self)
        self.item_all1 = MyQCheckBox("All", self, self.selection_vlayout1, None)
        self.item_all1.stateChanged.connect(lambda x: self.updateList(x, self.item_all1, None))
        self.selection_vlayout1.addWidget(self.item_all1)
        topic_data_list1 = E.get_general_features_options()
        topic_data_list1.sort()
        for topic in topic_data_list1:
            self.addCheckBox(topic, self.selection_vlayout1, self.selected_general_features)

        self.selection_vlayout2 = QVBoxLayout(self)
        self.item_all2 = MyQCheckBox("All", self, self.selection_vlayout2, None)
        self.item_all2.stateChanged.connect(lambda x: self.updateList(x, self.item_all2, None))
        self.selection_vlayout2.addWidget(self.item_all2)
        topic_data_list2 = E.get_specific_features_options()
        topic_data_list2.sort()
        for topic in topic_data_list2:
            self.addCheckBox(topic, self.selection_vlayout2, self.selected_specific_features)

        self.main_widget.setLayout(self.selection_vlayout)
        self.main_widget1.setLayout(self.selection_vlayout1)
        self.main_widget2.setLayout(self.selection_vlayout2)

        self.area.setWidget(self.main_widget)
        self.areagen.setWidget(self.main_widget1)
        self.areaspec.setWidget(self.main_widget2)
        self.show()

    def addCheckBox(self, topic, selection_vlayout, selected_list):
        item = MyQCheckBox(topic, self, selection_vlayout, selected_list)
        item.stateChanged.connect(lambda x: self.updateList(x, item, topic))
        self.items_list_topics.append(item)
        selection_vlayout.addWidget(item)

    def changeTopicCheckState(self, topic, state):
        for item in self.items_list_topics:
            if item.text() == topic:
                item.setCheckState(state)
                return

    def updateList(self, state, item_clicked, topic=None, force_update_state=False):
        if topic is None:  # The "All" checkbox was checked / unchecked
            print "if topic is None"
            if state == Qt.Checked:
                self.item_all.setTristate(False)
                for item in self.items_list_topics:
                    if item.checkState() == Qt.Unchecked and \
                            item.selection_vlayout == item_clicked.selection_vlayout:
                        item.setCheckState(Qt.Checked)
            elif state == Qt.Unchecked:
                self.item_all.setTristate(False)
                for item in self.items_list_topics:
                    if item.checkState() == Qt.Checked and \
                            item.selection_vlayout == item_clicked.selection_vlayout:
                        item.setCheckState(Qt.Unchecked)
        else:
            print "else:"
            if state == Qt.Checked:
                item_clicked.selected_list.append(topic)
                print item_clicked.selected_list
            else:
                item_clicked.selected_list.remove(topic)
                #if self.item_all.checkState() == Qt.Checked:
                #    self.item_all.setCheckState(Qt.PartiallyChecked)

        if self.selected_specific_features != []:
            if self.selected_bag_topics == [] and self.selected_general_features == []:
                self.ok_button.setEnabled(True)
            elif self.selected_bag_topics != [] and self.selected_general_features != []:
                self.ok_button.setEnabled(True)
            else:
                self.ok_button.setEnabled(False)
        else:
            if self.selected_bag_topics == [] or self.selected_general_features == []:
                self.ok_button.setEnabled(False)
            else:
                self.ok_button.setEnabled(True)

    def onButtonChooseCliked(self):
        for checkbox in self.items_list_topics:
                checkbox.setCheckState(Qt.Unchecked)
        with open("/var/tmp/logger.log", 'r') as f:
            topics = f.read().splitlines()
        for checkbox in self.items_list_topics:
            if checkbox.text() in topics:
                checkbox.setCheckState(Qt.Checked)

    def onButtonClicked(self):
        window = self.window.text()
        try:
            val = float(window)
        except ValueError:
            QMessageBox.about(self, "Error in Window Time", "That's not a number!")
            return
        if val >= self.duration:
            QMessageBox.about(self, "Error in Window Time", "time need to be smaller than: " + str(self.duration))
            return
        # Defined Logging
        handler = logging.FileHandler('/var/tmp/logger.log', mode='w')
        logger_topic.addHandler(handler)
        filename = QFileDialog.getSaveFileName(self, self.tr('csv File'), '.', self.tr('csv (*.csv)'))
        if filename[0] != '':
            topics = self.selected_bag_topics
            specific_features_selection = self.selected_specific_features
            general_features_selection = self.selected_general_features
            for topic in topics:
                logger_topic.info(topic)
            for topic in specific_features_selection:
                logger_topic.info(topic)
            for topic in general_features_selection:
                logger_topic.info(topic)
            ef = E.ExtractFeatures(topics, float(window), specific_features_selection, general_features_selection)
            counter = 0
            for bag_file in self.bag_files:
                df = ef.generate_features(bag_file)
                if len(self.bag_files) == 1:
                    counter = -1
                temp = get_corrent_file_name(filename[0], ".csv", counter)
                # temp = "%s_%s%s" % (filename[0],counter,".csv")
                E.write_to_csv(temp, df)
                counter = counter + 1
            QMessageBox.about(self, "csv export", "csv was exported successfuly")
        # df = ef.generate_features(self.bag_file)

def get_corrent_file_name(filename, suffix, i = -1):
    file_suffix = filename[:-len(suffix)]
    if file_suffix == suffix and i >= 0:
        ret = "%s_%s%s" % (filename[:-len(suffix)], i, suffix)
    elif file_suffix != suffix and i >= 0:
        ret = "%s_%s%s" % (filename, i, suffix)
    elif file_suffix != suffix and i < 0:
        ret = "%s%s" % (filename, suffix)
    else:
        ret = filename
    return ret
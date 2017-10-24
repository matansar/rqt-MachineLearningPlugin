
import TimeSeriesFeatures as TS
from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtWidgets import QFormLayout, QMessageBox, QFileDialog, QLineEdit, QRadioButton, QButtonGroup, QLabel, QWidget, QVBoxLayout, QCheckBox, QScrollArea, QPushButton
from MyQCheckBox import MyQCheckBox
import logging

#save choose of user
logger_topic = logging.getLogger("logger_history")

class HistorySelection(QWidget):

    def __init__(self):
        super(HistorySelection, self).__init__()
        self.setWindowTitle("Extracting Window Features")
        self.resize(550, 440)

        per_title, pre = TS.get_time_series_pre_feature_options()
        glob_title, glob = TS.get_global_time_series_features_options()

        self.history_items = dict()
        self.history_items[per_title] = pre
        self.history_items[glob_title] = glob

        print self.history_items

        self.group_selected_items = dict()
        self.group_areas = dict()
        self.group_main_widget = dict()
        self.group_selection_vlayout = dict()
        self.group_item_all = dict()
        self.main_vlayout = QFormLayout(self)
        self.group_label = dict()

        self.items_list = []
        self.selected_topics = []
        self.files = []

        # layout = QFormLayout()

        self.client_answers = QLineEdit()
        self.client_answers1 = QLineEdit()

        self.client_answers.setEnabled(False)
        self.client_answers1.setEnabled(False)

        self.ok_button = QPushButton("Select CSV...", self)
        self.ok_button.clicked.connect(self.onButtonClicked)

        self.main_vlayout.addRow(self.ok_button, self.client_answers)

        # self.main_vlayout.addWidget(self.ok_button)

        # self.main_vlayout.addWidget(self.ok_button)

        for group_name in self.history_items:
            self.group_selected_items[group_name] = []
            self.group_areas[group_name] = QScrollArea(self)
            self.group_main_widget[group_name] = QWidget(self.group_areas[group_name])
            self.group_label[group_name] = QLabel(group_name, self)
            self.group_label[group_name].setAlignment(Qt.AlignCenter)
            self.main_vlayout.addWidget(self.group_label[group_name])
            self.main_vlayout.addWidget(self.group_areas[group_name])
            self.group_selection_vlayout[group_name] = QVBoxLayout(self)
            self.group_item_all[group_name] = MyQCheckBox("All", self, self.group_selection_vlayout[group_name], None)
            self.MakeCheckBoxList(self.group_selection_vlayout[group_name],
                                  self.group_selected_items[group_name],
                                  self.history_items[group_name],
                                  self.group_item_all[group_name])
            self.group_main_widget[group_name].setLayout(self.group_selection_vlayout[group_name])
            self.group_areas[group_name].setWidget(self.group_main_widget[group_name])

        self.choose_button = QPushButton("Get Last Export Choose", self)
        self.choose_button.clicked.connect(self.onButtonChooseCliked)

        self.main_vlayout.addWidget(self.choose_button)

        self.label4 = QLabel("Window time", self)
        self.label4.setAlignment(Qt.AlignCenter)

        # self.main_vlayout.addWidget(self.label4)

        self.window = QLineEdit(self)
        # self.main_vlayout.addWidget(self.window)
        self.window.setText("3")

        self.main_vlayout.addRow(self.label4, self.window)

        # self.label5 = QLabel("Load CSV", self)
        # self.label5.setAlignment(Qt.AlignCenter)
        #
        # self.main_vlayout.addWidget(self.label5)

        # self.label6 = QLabel("output", self)
        # self.label6.setAlignment(Qt.AlignCenter)
        #
        # self.main_vlayout.addWidget(self.label6)

        self.save_button = QPushButton("Save CSV...", self)
        self.save_button.clicked.connect(self.onSaveClicked)

        self.submit_button = QPushButton("Sumbit", self)
        self.submit_button.clicked.connect(self.onSumbitClicked)

        # self.main_vlayout.addWidget(self.save_button)

        self.main_vlayout.addRow(self.save_button, self.client_answers1)

        self.main_vlayout.addWidget(self.submit_button)

        self.show()

    def MakeCheckBoxList(self, selection_vlayout, selected, topics_Keys, item_all):
        item_all.stateChanged.connect(lambda x: self.updateList(x, item_all, None))
        selection_vlayout.addWidget(item_all)
        topic_data_list = topics_Keys
        topic_data_list.sort()
        for topic in topic_data_list:
            self.addCheckBox(topic, selection_vlayout, selected)

    def addCheckBox(self, topic, selection_vlayout, selected_list):
        item = MyQCheckBox(topic, self, selection_vlayout, selected_list)
        item.stateChanged.connect(lambda x: self.updateList(x, item, topic))
        self.items_list.append(item)
        selection_vlayout.addWidget(item)

    def updateList(self, state, item_clicked, topic=None, force_update_state=False):
        if type(item_clicked) is str:
            item_clicked = self.get_item_by_name(item_clicked)
            if item_clicked is None:
                return
        if topic is None:  # The "All" checkbox was checked / unchecked
            # print "if topic is None"
            if state == Qt.Checked:
                # self.item_all.setTristate(False)
                for item in self.items_list:
                    if item.checkState() == Qt.Unchecked and \
                                    item.selection_vlayout == item_clicked.selection_vlayout:
                        item.setCheckState(Qt.Checked)
            elif state == Qt.Unchecked:
                # self.item_all.setTristate(False)
                for item in self.items_list:
                    if item.checkState() == Qt.Checked and \
                                    item.selection_vlayout == item_clicked.selection_vlayout:
                        item.setCheckState(Qt.Unchecked)
        else:
            if state == Qt.Checked:
                item_clicked.selected_list.append(topic)
                # print item_clicked.selected_list
            else:
                item_clicked.selected_list.remove(topic)
                # if self.item_all.checkState() == Qt.Checked:
                #    self.item_all.setCheckState(Qt.PartiallyChecked)

    def onButtonClicked(self):
        fd = QFileDialog(self)
        wc = "Csv files {.csv} (*.csv)"
        self.files = []
        tmp_pathes, filter = fd.getOpenFileNamesAndFilter(filter=wc, initialFilter=('*.csv'))
        for path in tmp_pathes:
            self.files.append(path.encode("utf-8"))
        print self.files
        if len(self.files) != 0:
            self.client_answers.setText(self.files[0])
        else:
            self.client_answers.setText("")

    def check_int(self, number, condition, title, message_body):
        try:
            val = int(number)
            if val < condition:
                QMessageBox.about(self, title, "The number should >= %s" % condition)
                return False
        except ValueError:
            QMessageBox.about(self, title, message_body)
            return False
        return True

    def check_choose(self):
        flag = True
        temp = []
        for item in self.group_selected_items.values():
            if item:
                for i in item:
                    temp.append(i)
        if len(temp) == 0:
            QMessageBox.about(self, "Features", "One feature at least should be chosen")
            flag = False
        return flag

    def check_files_amount(self):
        flag = True
        if len(self.files) <= 0:
            QMessageBox.about(self, "Load CSV", "One file at least should be chosen")
            flag = False
        return flag

    def check_validation(self):
        flag = self.check_files_amount()
        flag = flag & self.check_choose()
        flag = flag & self.check_int(self.window.text(), 3, "Error in Window Time", "That's not a number!")
        #TODO selected topic not empty
        return flag

    def onSumbitClicked(self):
        pass

    def onSaveClicked(self):
        print self.files
        print self.window.text()
        print self.group_selected_items.values()
        if not self.check_validation():
            return

        print self.group_selected_items

        to_save_filename = QFileDialog.getSaveFileName(self, self.tr('csv File'), '.', self.tr('csv (*.csv)'))
        if to_save_filename[0] != '':
            for item in self.group_selected_items.values():
                if item:
                    for i in item:
                        self.selected_topics.append(i)
            # Defined Logging
            handler = logging.FileHandler('/var/tmp/logger_history.log', mode='w')
            logger_topic.addHandler(handler)
            topics = self.selected_topics
            for topic in topics:
                logger_topic.info(topic)
            self.createTimeSeriesFeatures(self.files, to_save_filename,int(self.window.text()), self.group_selected_items)

    def createTimeSeriesFeatures(self, files, to_save_filename, window, group_selected_items):
        import TimeSeriesFeatures as TS
        to_save_filename = to_save_filename[0].encode('utf-8')
        for i in range(0, len(files)):
            if len(files) == 1:
                i = -1
            input_path = files[i]
            output_path = get_corrent_file_name(to_save_filename, ".csv", i)
            print "in = %s out = %s " % (input_path, output_path)
            self.client_answers1.setText(output_path)
            ts = TS.TimeSeries(input_path, output_path, window, group_selected_items)
            ts.generate_time_series_features()
            if len(files) == 1:
                i = 0
        QMessageBox.about(self, "csv save", "csv was saved successfuly")

    def onButtonChooseCliked(self):
        for checkbox in self.items_list:
            checkbox.setCheckState(Qt.Unchecked)
        with open("/var/tmp/logger_history.log", 'r') as f:
            topics = f.read().splitlines()
        for checkbox in self.items_list:
            if checkbox.text() in topics:
                checkbox.setCheckState(Qt.Checked)

def get_corrent_file_name(filename, suffix, i = -1):
    file_suffix = filename[:-len(suffix)]
    if file_suffix == suffix and i >= 0:
        ret = "%s_%s%s" % (filename[:-len(suffix)], i, suffix)
    elif file_suffix != suffix and i>= 0:
        ret = "%s_%s%s" % (filename, i, suffix)
    elif file_suffix != suffix and i< 0:
        ret = "%s%s" % (filename, suffix)
    else:
        ret = filename
    return ret
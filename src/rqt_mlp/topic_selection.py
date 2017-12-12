import rosgraph
import RunScenarios as S
import TimeSeriesFeatures as TS
from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtWidgets import QHBoxLayout, QFormLayout, QRadioButton, QButtonGroup, QLabel, QWidget, QVBoxLayout, \
    QCheckBox, QScrollArea, QPushButton
from .node_selection import NodeSelection
from MyQCheckBox import MyQCheckBox
from functools import partial
from .input_dialog import inputDialog
import logging

# save choose of user
# logger_topic = logging.getLogger("logger_topic")

class TopicSelection(QWidget):
    recordSettingsSelected = Signal(bool, list, dict)

    def __init__(self):
        super(TopicSelection, self).__init__()
        master = rosgraph.Master('rqt_bag_recorder')
        self.setWindowTitle("Record a Simulation")
        self.resize(650, 720)

        pre = TS.get_time_series_pre_feature_options()
        glob = TS.get_global_time_series_features_options()
        # print pre
        # print glob
        all_topics = S.get_topics_options()
        keys = all_topics.keys()
        # print all_topics.keys()[0]

        self.group_selected_items = dict()
        self.group_areas = dict()
        self.group_main_widget = dict()
        self.group_selection_vlayout = dict()
        self.group_item_all = dict()
        # self.main_vlayout = QVBoxLayout(self)
        self.main_vlayout = QVBoxLayout(self)

        self.group_label = dict()

        self.selected_topics = []
        self.items_list = []

        for group_name in all_topics:
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
                                  all_topics[group_name],
                                  self.group_item_all[group_name])
            self.group_main_widget[group_name].setLayout(self.group_selection_vlayout[group_name])
            self.group_areas[group_name].setWidget(self.group_main_widget[group_name])

        self.label1 = QLabel("Scenarios", self)
        self.label1.setAlignment(Qt.AlignCenter)

        self.main_vlayout.addWidget(self.label1)

        scanarios = S.get_scenarios_options()
        self.scanarios_answer = scanarios
        self.map_answer = dict()
        # print scanarios
        keys1 = scanarios.keys()

        # print keys1[0]
        # print scanarios[keys1[0]]["name"]
        # print scanarios[keys1[0]]["params"]
        #
        # for item in scanarios[keys1[0]]["params"]:
        #     print item

        self.radio_items = dict()
        self.number_group = QButtonGroup(self)

        for id_radio in scanarios:
            self.radio_items[id_radio] = QRadioButton(scanarios[id_radio]["name"])
            self.number_group.addButton(self.radio_items[id_radio])
            self.main_vlayout.addWidget(self.radio_items[id_radio])
            self.radio_items[id_radio].setChecked(False)
            self.radio_items[id_radio].clicked.connect(
                partial(self.callConsult, scanarios[id_radio]["params"], id_radio))

        # self.label = QLabel("live Topics", self)
        # self.label.setAlignment(Qt.AlignCenter)
        #
        # self.main_vlayout.addWidget(self.label)

        # self.area = QScrollArea(self)
        # self.main_widget = QWidget(self.area)

        self.ok_button = QPushButton("Record", self)
        self.ok_button.clicked.connect(self.onButtonClicked)
        self.ok_button.setEnabled(False)
        self.choose_button = QPushButton("Get Last Export Choose", self)
        self.choose_button.clicked.connect(self.onButtonChooseCliked)

        self.clear_button = QPushButton("Clear Selection", self)
        self.clear_button.clicked.connect(self.onClearClicked)

        self.choose_clear_buttons = QHBoxLayout(self)

        self.choose_clear_buttons.addWidget(self.choose_button)

        self.choose_clear_buttons.addWidget(self.clear_button)

        self.main_vlayout.addLayout(self.choose_clear_buttons)

        # self.main_vlayout.addRow(self.choose_button, self.clear_button)

        # self.from_nodes_button = QPushButton("From Nodes", self)
        # self.from_nodes_button.clicked.connect(self.onFromNodesButtonClicked)

        # self.main_vlayout.addWidget(self.area)
        # self.main_vlayout.addWidget(self.choose_button)
        self.main_vlayout.addWidget(self.ok_button)
        # self.main_vlayout.addWidget(self.from_nodes_button)
        self.setLayout(self.main_vlayout)

        self.selection_vlayout = QVBoxLayout(self)

        # self.item_all = MyQCheckBox("All", self, self.selection_vlayout, None)
        # self.item_all.stateChanged.connect(lambda x: self.updateList(x, self.item_all, None))
        # self.selection_vlayout.addWidget(self.item_all)
        # topic_data_list4 = map(lambda l: l[0], master.getPublishedTopics(''))
        # topic_data_list4.sort()
        # for topic in topic_data_list4:
        #     self.addCheckBox(topic, self.selection_vlayout, self.selected_topics)

        # self.main_widget.setLayout(self.selection_vlayout)

        # self.area.setWidget(self.main_widget)

        # print S.get_scenarios_options()

        self.show()

    def onClearClicked(self):
        self.clearTopicCheckState()

    def clearTopicCheckState(self):
        for item in self.items_list:
            item.setCheckState(False)
        for item in self.group_item_all.values():
            item.setCheckState(False)

    def onButtonChooseCliked(self):
        for checkbox in self.items_list:
            checkbox.setCheckState(Qt.Unchecked)
        with open(get_path() + "logger_topic.log", 'r') as f:
            topics = f.read().splitlines()
        for checkbox in self.items_list:
            if checkbox.text() in topics:
                checkbox.setCheckState(Qt.Checked)

    def callConsult(self, params, id_radio):
        self.input_dialog = inputDialog(params, id_radio)
        self.input_dialog.ParamsSelected.connect(self.params_answer)
        # item, ok = QInputDialog.getItem(self, "select parameter",
        #                                 "list of parameters", params, 0, False)
        # if ok and item:
        #     self.gettext(item)

    def params_answer(self, params, label_items, id_radio):
        if id_radio == 0:
            print "------" + str(id_radio)
            self.number_group.setExclusive(False)
            # print self.radio_items
            for item in self.radio_items:
                # print self.radio_items[item]
                self.radio_items[item].setChecked(False)
            self.number_group.setExclusive(True)
            self.enable_record()
        else:
            print id_radio
            print params
            # print params.values()
            a = {}
            print label_items
            for item, name in zip(params, label_items):
                print item
                print name
                # value = params[item].text()
                a[name] = item.text().encode("utf-8")
            print a
            self.map_answer = {"id": id_radio, "params": a}
            print self.map_answer
            self.enable_record()
            return self.map_answer


        # for item in params.values():
        #     print item.text()
        # pass

    # def gettext(self, item):
    #     text, ok = QInputDialog.getText(self, 'Text Input Dialog', item)
    #
    #     if ok:
    #        print self.multipleReplace(str(text), item)
    #
    #
    # def multipleReplace(self, text, item):
    #     for key in self.scanarios_answer:
    #         #
    #         print self.scanarios_answer[key]["params"]
    #         for id, par in enumerate(self.scanarios_answer[key]["params"]):
    #             if par == item:
    #                 self.scanarios_answer[key]["params"][id] = text
    #     for id_radio in self.radio_items.keys():
    #         if self.radio_items[id_radio].isChecked():
    #             self.map_answer = {"id" : id_radio, "params" : self.scanarios_answer[id_radio]["params"]}
    #     return self.map_answer

    def MakeCheckBoxList(self, selection_vlayout, selected, topics_Keys, item_all):
        item_all.stateChanged.connect(lambda x: self.updateList(x, item_all, None))
        selection_vlayout.addWidget(item_all)
        topic_data_list = topics_Keys
        topic_data_list.sort()
        for topic in topic_data_list:
            self.addCheckBox(topic, selection_vlayout, selected)

    def get_item_by_name(self, item_name):
        for item in self.items_linputDialogist:
            if item.text() == item_name:
                return item
        return None

    def addCheckBox(self, topic, selection_vlayout, selected_list):
        item = MyQCheckBox(topic, self, selection_vlayout, selected_list)
        item.stateChanged.connect(lambda x: self.updateList(x, item, topic))
        self.items_list.append(item)
        selection_vlayout.addWidget(item)

    def changeTopicCheckState(self, topic, state):
        for item in self.items_list:
            if item.text() == topic:
                item.setCheckState(state)
                return

    # def updateList(self, state, topic=None, force_update_state=False):
    #     if topic is None:  # The "All" checkbox was checked / unchecked
    #         if state == Qt.Checked:
    #             self.item_all.setTristate(False)
    #             for item in self.items_list:
    #                 if item.checkState() == Qt.Unchecked:
    #                     item.setCheckState(Qt.Checked)
    #         elif state == Qt.Unchecked:
    #             self.item_all.setTristate(False)
    #             for item in self.items_list:
    #                 if item.checkState() == Qt.Checked:
    #                     item.setCheckState(Qt.Unchecked)
    #     else:
    #         if state == Qt.Checked:
    #             self.selected_topics.append(topic)
    #         else:
    #             self.selected_topics.remove(topic)
    #             if self.item_all.checkState() == Qt.Checked:
    #                 self.item_all.setCheckState(Qt.PartiallyChecked)
    #
    #     if self.selected_topics == []:
    #         self.ok_button.setEnabled(False)
    #     else:
    #         self.ok_button.setEnabled(True)

    def updateList(self, state, item_clicked, topic=None, force_update_state=False):
        if type(item_clicked) is str:
            item_clicked = self.get_item_by_name(item_clicked)
            if item_clicked is None:
                return
        if topic is None:  # The "All" checkbox was checked / unchecked
            #print "if topic is None"
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
                #print item_clicked.selected_list
            else:
                item_clicked.selected_list.remove(topic)
        #if self.item_all.checkState() == Qt.Checked:
        #    self.item_all.setCheckState(Qt.PartiallyChecked)

        self.enable_record()

    def enable_record(self):
        temp_selected_topics = []
        for item in self.group_selected_items.values():
            if item:
                for i in item:
                    temp_selected_topics.append(i)

        flag = False
        for item in self.radio_items:
            if self.radio_items[item].isChecked():
                flag = True
                break
            print self.radio_items[item]

        # flag = reduce(lambda acc, curr: acc or , self.radio_items, False)
        print "------+++++-" + str(flag)
        if len(temp_selected_topics) > 0 or len(self.selected_topics) > 0:
            self.ok_button.setEnabled(flag)
        else:
            self.ok_button.setEnabled(False)

    def onButtonClicked(self):
        # print self.group_selected_items
        for item in self.group_selected_items.values():
            if item:
                for i in item:
                    self.selected_topics.append(i)
        print "-----" + str(self.selected_topics)

        # Defined Logging
        # handler = logging.FileHandler('/var/tmp/logger_topic.log', mode='a')
        # logger_topic.addHandler(handler)
        topics = self.selected_topics
        with open(get_path() + 'logger_topic.log', "w") as f:
            for topic in topics:
                f.write(topic + "\n")
        self.close()
        self.recordSettingsSelected.emit(False, self.selected_topics, self.map_answer)

    def onFromNodesButtonClicked(self):
        self.node_selection = NodeSelection(self)

    def getTopicsByName(self, name):
        arr = S.get_topics_options()
        return arr[name]


def get_path():
  import inspect, os
  import logging
  logging_file = os.path.dirname(
      os.path.abspath(inspect.getfile(inspect.currentframe()))) + "/Scenarios/History_Choosing/"
  return logging_file
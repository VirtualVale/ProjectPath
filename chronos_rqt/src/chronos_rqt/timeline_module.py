import os
import rospy
import rospkg
import actionlib
import chronos.msg

from datetime import datetime
from std_msgs.msg import Time
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QPushButton, QDoubleSpinBox
from PyQt5.QtCore import QDateTime, Qt

class MyQWidget(QWidget):
    def __init__(self):
        super(MyQWidget, self).__init__()
        # all gui elements are now accessed through self.ui
        self._x = 0.0
        self._y = 0.0

    def mousePressEvent(self, event):
        print("new goal coordinates")
        self._x = (event.x()-466)*0.05
        self._y = (event.y()-356)*-0.05
        print((event.x()-466)*0.05)
        print(((event.y()-356)*-1)*0.05)
        print(event.pos())


class ChronosRqt(Plugin):

    def __init__(self, context):
        super(ChronosRqt, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ChronosRqt')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._x = 0.0
        self._y = 0.0
        self._widget = MyQWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('chronos_rqt'), 'resource', 'ChronosRqt.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        #anchor for the time calculation // time point of system start in unix format (msecs)
        self.origin_time = QDateTime.currentMSecsSinceEpoch()-rospy.get_rostime().to_sec()*1000

        self._widget.startTime_dateTimeEdit.setDateTime(QDateTime.currentDateTime())
        self._widget.input_dateTimeEdit.setDateTime(QDateTime.currentDateTime())
        self._widget.display_dateTimeEdit.setDateTime(QDateTime.currentDateTime())
        self._widget.job_comboBox.addItems(["create Job", "Delete Job", "Read Job", "Create Path"])
        
        self._publisher = rospy.Publisher("timer", Time, queue_size=100)

        self._widget.currentTime_button.clicked.connect(self._on_currentTime_button_clicked)
        self._widget.time_slider.valueChanged.connect(self._on_time_slider_changed)
        self._widget.execute_pushButton.clicked.connect(self._on_execute_button_clicked)
        self._widget.display_dateTimeEdit.dateTimeChanged.connect(self._on_display_dateTimeEdit_changed)
        self._widget.input_dateTimeEdit.dateTimeChanged.connect(self._on_time_slider_changed)
        self._widget.refresh_pushButton.clicked.connect(self._on_refresh_button_clicked)
        self._widget.job_comboBox.currentIndexChanged.connect(self._on_combobox_changed)
        self.sub = rospy.Subscriber("plan", chronos.msg.plan, self.callback)
        self._widget.plan1_listWidget.itemDoubleClicked.connect(self._on_plan1_listWidget_doubleClicked)

    def _on_plan1_listWidget_doubleClicked(self):
        print(self._widget.plan1_listWidget.currentItem())

    def callback(self, plan):
        self._widget.plan1_listWidget.clear()
        self._widget.plan2_listWidget.clear()
        self._widget.plan3_listWidget.clear()
        for path in plan.plan_1:
            self._widget.plan1_listWidget.addItem(
                "start time: " + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[0].header.stamp.to_sec()).day) + "." + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[0].header.stamp.to_sec()).month) + "." + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[0].header.stamp.to_sec()).year) + " " + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[0].header.stamp.to_sec()).hour) + ":"  + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[0].header.stamp.to_sec()).minute) + ":"  + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[0].header.stamp.to_sec()).second) + " start position x: " + str(round(path.poses[0].pose.position.x, 2)) + " y: " + str(round(path.poses[0].pose.position.y, 2)) + 
                "\nend time:   " + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[-1].header.stamp.to_sec()).day) + "." + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[-1].header.stamp.to_sec()).month) + "." +  str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[-1].header.stamp.to_sec()).year) + " " + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[-1].header.stamp.to_sec()).hour) + ":"  + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[-1].header.stamp.to_sec()).minute) + ":"  + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[-1].header.stamp.to_sec()).second) + " goal position  x: " + str(path.poses[-1].pose.position.x) + " y: " + str(path.poses[-1].pose.position.y))
        for path in plan.plan_2:
            self._widget.plan2_listWidget.addItem(
                "start time: " + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[0].header.stamp.to_sec()).day) + "." + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[0].header.stamp.to_sec()).month) + "." + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[0].header.stamp.to_sec()).year) + " " + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[0].header.stamp.to_sec()).hour) + ":"  + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[0].header.stamp.to_sec()).minute) + ":"  + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[0].header.stamp.to_sec()).second) + " start position x: " + str(round(path.poses[0].pose.position.x, 2)) + " y: " + str(round(path.poses[0].pose.position.y, 2)) + 
                "\nend time:   " + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[-1].header.stamp.to_sec()).day) + "." + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[-1].header.stamp.to_sec()).month) + "." +  str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[-1].header.stamp.to_sec()).year) + " " + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[-1].header.stamp.to_sec()).hour) + ":"  + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[-1].header.stamp.to_sec()).minute) + ":"  + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[-1].header.stamp.to_sec()).second) + " goal position  x: " + str(path.poses[-1].pose.position.x) + " y: " + str(path.poses[-1].pose.position.y))
        for path in plan.plan_3:
            self._widget.plan3_listWidget.addItem(
                "start time: " + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[0].header.stamp.to_sec()).day) + "." + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[0].header.stamp.to_sec()).month) + "." + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[0].header.stamp.to_sec()).year) + " " + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[0].header.stamp.to_sec()).hour) + ":"  + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[0].header.stamp.to_sec()).minute) + ":"  + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[0].header.stamp.to_sec()).second) + " start position x: " + str(round(path.poses[0].pose.position.x, 2)) + " y: " + str(round(path.poses[0].pose.position.y, 2)) + 
                "\nend time:   " + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[-1].header.stamp.to_sec()).day) + "." + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[-1].header.stamp.to_sec()).month) + "." +  str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[-1].header.stamp.to_sec()).year) + " " + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[-1].header.stamp.to_sec()).hour) + ":"  + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[-1].header.stamp.to_sec()).minute) + ":"  + str(datetime.fromtimestamp(self.origin_time*0.001+path.poses[-1].header.stamp.to_sec()).second) + " goal position  x: " + str(path.poses[-1].pose.position.x) + " y: " + str(path.poses[-1].pose.position.y))

    def _on_combobox_changed(self):
        print(self._widget.job_comboBox.currentIndex())

    def _on_refresh_button_clicked(self):
        self._widget.position_x_doubleSpinBox.setValue(self._widget._x)
        self._widget.position_y_doubleSpinBox.setValue(self._widget._y)

    def _on_display_dateTimeEdit_changed(self):
        ROSTimeInMsecs = self._widget.display_dateTimeEdit.dateTime().toMSecsSinceEpoch() - (QDateTime.currentMSecsSinceEpoch()-rospy.get_rostime().to_sec()*1000)
        self._send_time(ROSTimeInMsecs*0.001)

    def _on_time_slider_changed(self):
        self._widget.display_dateTimeEdit.setDateTime(QDateTime.fromMSecsSinceEpoch(self._widget.input_dateTimeEdit.dateTime().toMSecsSinceEpoch() + self._widget.time_slider.value()*1000))
        #self._send_time()

    def _on_currentTime_button_clicked(self):
        self._widget.input_dateTimeEdit.setDateTime(QDateTime.currentDateTime())
        self._widget.display_dateTimeEdit.setDateTime(QDateTime.currentDateTime())
        print(rospy.get_rostime().to_sec())
        #self._widget.time_slider.setValue(rospy.get_rostime())

    
    def _on_execute_button_clicked(self):
        # client for pathCreation with actionserver
        client = actionlib.SimpleActionClient('planning_actionserver', chronos.msg.planningAction)
        goal = chronos.msg.planningGoal()
        goal.task = self._widget.job_comboBox.currentIndex()
        goal.resource_number = self._widget.resource_spinBox.value()
        ROSTimeInMsecs = self._widget.startTime_dateTimeEdit.dateTime().toMSecsSinceEpoch() - (QDateTime.currentMSecsSinceEpoch()-rospy.get_rostime().to_sec()*1000)
        if(ROSTimeInMsecs > 0):
            goal.start_time.data = rospy.Time(ROSTimeInMsecs*0.001)
            goal.goal.pose.position.x = self._widget.position_x_doubleSpinBox.value()
            goal.goal.pose.position.y = self._widget.position_y_doubleSpinBox.value()
            # Sends the goal to the action server.
            client.send_goal(goal)

    def on_parameter_changed(self):
        self._send_time(self._widget.time_slider.value())
    
    def _send_time(self, time):
        if time<0:
            time=rospy.Time(0)
        else:
            time = rospy.Time(time)

        self._publisher.publish(time)
        


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
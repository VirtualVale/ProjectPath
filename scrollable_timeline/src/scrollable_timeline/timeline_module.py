import os
import rospy
import rospkg
import actionlib
import chronos.msg

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


class ScrollableTimeline(Plugin):

    def __init__(self, context):
        super(ScrollableTimeline, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ScrollableTimeline')

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
        ui_file = os.path.join(rospkg.RosPack().get_path('scrollable_timeline'), 'resource', 'ScrollableTimeline.ui')
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

        self._widget.startTime_dateTimeEdit.setDateTime(QDateTime.currentDateTime())
        self._widget.input_dateTimeEdit.setDateTime(QDateTime.currentDateTime())
        self._widget.display_dateTimeEdit.setDateTime(QDateTime.currentDateTime())
        #self._widget.time_slider.setSliderPosition(QDateTime.currentMSecsSinceEpoch())
        
        self._publisher = rospy.Publisher("timer", Time, queue_size=100)

        self._widget.currentTime_button.clicked.connect(self._on_currentTime_button_clicked)
        self._widget.time_slider.valueChanged.connect(self._on_time_slider_changed)
        self._widget.goal_pushButton.clicked.connect(self._on_goal_button_clicked)


    def _on_time_slider_changed(self):
        self._widget.display_dateTimeEdit.setDateTime(QDateTime.fromMSecsSinceEpoch(QDateTime.currentMSecsSinceEpoch() + self._widget.time_slider.value()*1000))
        #self.on_parameter_changed()

    def _on_currentTime_button_clicked(self):
        self._widget.position_x_doubleSpinBox.setValue(self._widget._x)
        self._widget.position_y_doubleSpinBox.setValue(self._widget._y)
        self._widget.input_dateTimeEdit.setDateTime(QDateTime.currentDateTime())
        self._widget.display_dateTimeEdit.setDateTime(QDateTime.currentDateTime())
        print(QDateTime.currentMSecsSinceEpoch() + 2592000000)
        print(QDateTime.currentMSecsSinceEpoch() - 2592000000)
        #self._widget.time_slider.setValue(rospy.get_rostime())

    
    def _on_goal_button_clicked(self):
        # client for pathCreation with actionserver
        client = actionlib.SimpleActionClient('PTS', chronos.msg.PTSAction)
        # client.wait_for_server()
        goal = chronos.msg.PTSGoal()
        goal.goal.pose.position.x = self._widget.position_x_doubleSpinBox.value()
        goal.goal.pose.position.y = self._widget.position_y_doubleSpinBox.value()
        goal.start_time.data = rospy.get_rostime() + rospy.Duration(0.001 *(QDateTime.currentMSecsSinceEpoch() - self._widget.startTime_dateTimeEdit.dateTime().toMSecsSinceEpoch()))
        goal.resource_number = self._widget.resource_spinBox.value()
        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        # client.wait_for_result()

        # Prints out the result of executing the action
        return client.get_result()  # A FibonacciResult

    def on_parameter_changed(self):
        self._send_time(self._widget.time_slider.value())
    
    def _send_time(self, time):
        time = rospy.Time(time*0.1)
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
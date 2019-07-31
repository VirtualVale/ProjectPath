import os
import rospy
import rospkg

from std_msgs.msg import Time
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QPushButton, QDoubleSpinBox



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
        self._widget = QWidget()

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

        self._publisher = rospy.Publisher("timer", Time, queue_size=100)

        self._widget.pushButton.clicked.connect(self._on_button_clicked)
        self._widget.horizontalSlider.valueChanged.connect(self._on_slider_changed)
        self._widget.doubleSpinBox.valueChanged.connect(self._on_spinbox_changed)

    def _on_spinbox_changed(self):
        self._send_time(self._widget.doubleSpinBox.value())

    def _on_slider_changed(self):
        self._widget.doubleSpinBox.setValue(self._widget.horizontalSlider.value()*0.1)
        self.on_parameter_changed()

    def _on_button_clicked(self):
        self._widget.horizontalSlider.setValue(901)
        self._send_time(0)

    def on_parameter_changed(self):
        self._send_time(self._widget.horizontalSlider.value())
    
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
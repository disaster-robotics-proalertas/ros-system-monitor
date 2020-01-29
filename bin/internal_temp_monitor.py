#!/usr/bin/env python

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rospy
import rostopic
import socket
import subprocess
from sensor_msgs.msg import Temperature

class Monitor:
    def __init__(self, vehicle_name, **kwargs):
        self.vehicle_name = vehicle_name
    
        # Initialize node
        rospy.init_node("internal_temp_monitor_%s" % vehicle_name, anonymous=True)

        # Create system status publisher
        self.status_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)

        # Get parameters
        self.temp_level_warn = rospy.get_param('~temp_level_warn')
        self.temp_level_error = rospy.get_param('~temp_level_error')
        self.topic = rospy.get_param('~temp_level_topic', default='/temperature')

        # Create subscriber for temperature
        rospy.Subscriber(self.topic, Temperature, callback=self.temp_callback)

        # Temperature and diagnostics message
        self.temperature = Temperature()

        # Rate
        self.rate = rospy.Rate(1)

        # Time for last received message
        self.last_update = 0

    def temp_callback(self, msg):
        self.temperature = msg
        self.last_update = rospy.Time().now().to_sec()

    def run(self):
        # Run while ROS is active
        while not rospy.is_shutdown():
            # Diagnostics message
            temp_diag = DiagnosticArray()
            temp_diag.header.stamp = rospy.Time().now()
            temp_diag.status.append(DiagnosticStatus())
            temp_diag.status[0].name = 'Internal Temperature'
            temp_diag.status[0].hardware_id = self.vehicle_name

            # Compare internal temperature with threshold levels
            temp_diag.status[0].level = DiagnosticStatus.OK
            temp_diag.status[0].message = 'OK'
            temp_diag.status[0].values.insert(0, KeyValue(key='Update Status', value='OK'))
            if self.temperature.temperature >= self.temp_level_warn:
                temp_diag.status[0].level = DiagnosticStatus.WARN
                temp_diag.status[0].message = 'Warning'
                temp_diag.status[0].values.insert(0, KeyValue(key = 'Update Status', value = 'Warning'))
            elif self.temperature.temperature >= self.temp_level_error:
                temp_diag.status[0].level = DiagnosticStatus.ERROR
                temp_diag.status[0].message = 'Error'
                temp_diag.status[0].values.insert(0, KeyValue(key = 'Update Status', value = 'Error'))
            temp_diag.status[0].values.insert(1, KeyValue(key='Temperature', value=str(self.temperature.temperature)))
            
            # Check if message is stale (older than 35 seconds)
            elapsed = rospy.Time().now().to_sec() - self.last_update
            if elapsed > 35:
                temp_diag.status[0].level = DiagnosticStatus.STALE
                temp_diag.status[0].message = 'Stale'
                temp_diag.status[0].values.insert(0, KeyValue(key = 'Update Status', value = 'Stale'))
                temp_diag.status[0].values.insert(1, KeyValue(key = 'Time Since Update', value = str(elapsed)))

            # Publish diagnostics message
            self.status_pub.publish(temp_diag)

            # Sleep for some time
            self.rate.sleep()
            
if __name__ == '__main__':
    temp_monitor = Monitor(socket.gethostname())
    temp_monitor.run()
#!/usr/bin/env python

import rospy
import socket
import rostopic
from math import sqrt
from mavros_msgs.msg import RCIn
from sensor_msgs.msg import NavSatFix
from diagnostic_msgs.msg import DiagnosticArray
from system_monitor_msgs.msg import VehicleStatus

'''
This node verifies health for all modules, according to each module's monitors, and reports overall vehicle status
Statuses are:
0 - Error       (when one of the peripheral sensor modules fails for some reason)
1 - Boot        (initial state, vehicle waits for GPS fix and for sensor modules to report OK/Warning status)
2 - Service     (Vehicle is operational)
3 - Recording   (special state, only activated through an RC command)
'''

class FSM:
    # FSM states
    ERROR=0
    BOOT=1
    SERVICE=2
    RECORDING=3

    def __init__(self, **kwargs):
        # Initial FSM state
        self.state = self.BOOT

        # ROS rate (5 Hz)
        self.rate = rospy.Rate(5)

        # Parameters
        self.sensor_modules = rospy.get_param("asv_description/sensor_modules")
        self.vehicle_name = rospy.get_param("asv_description/vehicle_name")
        self.rec_topic = rospy.get_param("asv_description/record_command_topic")
        self.gps_topic = rospy.get_param("asv_description/fix_topic", default='/gps/fix')
        self.rec_cmd_channel = rospy.get_param('asv_description/record_command_channel', default=5)
        self.rec_cmd_threshold = rospy.get_param('asv_description/record_command_threshold', default=20)

        # Publishers and subscribers
        self.status_pub = rospy.Publisher('%s/status' % self.vehicle_name, VehicleStatus, queue_size=10)
        self.gps_sub = rospy.Subscriber(self.gps_topic, NavSatFix, callback=self.gps_callback)
        self.rec_cmd_sub = rospy.Subscriber(self.rec_topic, RCIn, callback=self.rec_callback)
        
        # Messages
        self.rec_msg = RCIn()
        self.rec_nominal = RCIn()
        self.gps_msg = NavSatFix()
        self.vehicle_status = VehicleStatus()
        self.vehicle_status.name = self.vehicle_name

    # Subscriber callbacks
    def rec_callback(self, msg):
        self.rec_msg = msg

    def gps_callback(self, msg):
        self.gps_msg = msg

    # Helper functions
    def check_modules_health(self):
        # Check health for each module 
        system_healthy = True
        for modname in self.sensor_modules:
            try:
                # 2 seconds timeout for diagnostics message
                diag_msg = rospy.wait_for_message("%s/status" % modname, DiagnosticArray, timeout=2.0)
                # If at least one module is in error (codes 0 - OK, 1 - Warning, 2 - Error), overall system is not healthy
                if diag_msg.status.level > 1:
                    system_healthy = False
            # This exception is thrown if wait_for_message has timed out, in which case the module is unresponsive
            except rospy.ROSException:
                system_healthy = False
        return system_healthy

    def check_rec_cmd(self):
        # Check if RC PWM channel is within a certain threshold
        nominal_pwm = self.rec_nominal.channels[self.rec_cmd_channel]
        current_pwm = self.rec_msg.channels[self.rec_cmd_channel]
        if sqrt((current_pwm - nominal_pwm)**2) <= self.rec_cmd_threshold:
            return True
        else:
            return False
    
    # Main FSM function
    def run(self):
        if self.state == self.ERROR:
            # If all modules are back online
            if self.check_modules_health() == 0:
                self.state = self.BOOT
        elif self.state == self.BOOT:
            # If modules are healthy and GPS is fix
            if self.check_modules_health() and self.gps_msg.status >= 0:
                self.state = self.SERVICE
            # Get nominal RC PWM channel values
            self.rec_nominal = self.rec_msg
        elif self.state == self.SERVICE:
            # Check GPS fix was lost
            if self.gps_msg.status < 0:
                self.state = self.BOOT
            # Check if modules are unhealthy
            if not self.check_modules_health():
                self.state = self.ERROR
            # Check if record command on RC is enabled
            if self.check_rec_cmd():
                self.state = self.RECORDING
        elif self.state == self.RECORDING:
            if not self.check_rec_cmd():
                self.state = self.SERVICE
        
        # Publish current vehicle status
        self.vehicle_status.header.stamp = rospy.Time.now()
        self.status_pub.publish(self.vehicle_status)         

        # Sleep for some time
        self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('vehicle_monitor', anonymous=True)
    fsm = FSM()
    # Run FSM while ros node is active
    while not rospy.is_shutdown():
        fsm.run()
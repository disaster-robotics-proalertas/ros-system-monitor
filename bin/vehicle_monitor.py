#!/usr/bin/env python

import rospy
import socket
import rostopic
from math import sqrt
from mavros_msgs.msg import RCIn
from sensor_msgs.msg import NavSatFix
from diagnostic_msgs.msg import DiagnosticArray
from system_monitor.msg import VehicleState

'''
This node verifies health for all modules, according to each module's monitors, and reports overall vehicle status
Statuses are:
0 - Error       (when one of the peripheral sensor modules fails for some reason)
1 - Boot        (initial state, vehicle waits for GPS fix and for sensor modules to report OK/Warning status)
2 - Service     (Vehicle is operational)
3 - Recording   (special state, only activated through an RC command)
'''

class FSM:
    def __init__(self, **kwargs):
        # Initial FSM state
        self.state = VehicleState.BOOT
        self.statedesc = 'Waiting for GPS fix'
        self.statename = 'BOOT'

        # ROS rate (1 Hz)
        self.rate = rospy.Rate(1)

        # Parameters
        self.vehicle_name = socket.gethostname()
        self.sensor_modules = rospy.get_param("/asv_description/%s/sensor_modules" % self.vehicle_name)
        self.rec_topic = rospy.get_param("/asv_description/%s/record_command_topic" % self.vehicle_name)
        self.gps_topic = rospy.get_param("/asv_description/%s/fix_topic" % self.vehicle_name, default='/gps/fix')
        self.rec_cmd_channel = rospy.get_param("/asv_description/%s/record_command_channel" % self.vehicle_name, default=5)
        self.rec_cmd_threshold = rospy.get_param("/asv_description/%s/record_command_threshold" % self.vehicle_name, default=20)

        # Publishers and subscribers
        self.status_pub = rospy.Publisher('/system_monitor/%s/vehicle/state' % self.vehicle_name, VehicleState, queue_size=10)
        self.gps_sub = rospy.Subscriber(self.gps_topic, NavSatFix, callback=self.gps_callback)
        self.rec_cmd_sub = rospy.Subscriber(self.rec_topic, RCIn, callback=self.rec_callback)
        
        # Messages
        self.rec_msg = RCIn()
        self.rec_nominal = RCIn()
        self.gps_msg = NavSatFix()
        self.vehicle_state = VehicleState()

    # Subscriber callbacks
    def rec_callback(self, msg):
        self.rec_msg = msg

    def gps_callback(self, msg):
        self.gps_msg = msg

    # Helper functions
    def check_modules_health(self):
        # Check health for each module 
        system_healthy = True
        faulty_mod = ''
        for modname in self.sensor_modules:
            try:
                # 2 seconds timeout for diagnostics message
                diag_msg = rospy.wait_for_message("%s/status" % modname, DiagnosticArray, timeout=2.0)
                # If at least one module is in error (codes 0 - OK, 1 - Warning, 2 - Error), overall system is not healthy
                if diag_msg.status[0].level > 1:
                    system_healthy = False
                    faulty_mod = '%s:%s' % (diag_msg.hardware_id, diag_msg.name)
            # This exception is thrown if wait_for_message has timed out, in which case the module is unresponsive
            except rospy.ROSException:
                system_healthy = False
        return system_healthy, faulty_mod

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
        if self.state == VehicleState.ERROR:
            # If all modules are back online
            if self.check_modules_health() == 0:
                self.state = VehicleState.BOOT
                self.statename = 'BOOT'
                self.statedesc = 'Waiting for GPS fix'
        elif self.state == VehicleState.BOOT:
            # If modules are healthy and GPS is fix
            if self.check_modules_health() and self.gps_msg.status >= 0:
                self.state = VehicleState.SERVICE
                self.statename = 'SERVICE'
                self.statedesc = 'Vehicle operational'
            # Get nominal RC PWM channel values
            self.rec_nominal = self.rec_msg
        elif self.state == VehicleState.SERVICE:
            # Check GPS fix was lost
            if self.gps_msg.status < 0:
                self.state = VehicleState.BOOT
                self.statename = 'BOOT'
                self.statedesc = 'Waiting for GPS fix'
            # Check if modules are unhealthy
            healthy, fault = self.check_modules_health()
            if not healthy:
                self.state = VehicleState.ERROR
                self.statename = 'ERROR'
                self.statedesc = 'Error in %s' % fault
            # Check if record command on RC is enabled
            if self.check_rec_cmd():
                self.state = VehicleState.RECORDING
                self.statename = 'RECORDING'
                self.statedesc = 'Vehicle logging data'
        elif self.state == VehicleState.RECORDING:
            if not self.check_rec_cmd():
                self.state = VehicleState.SERVICE
                self.statename = 'SERVICE'
                self.statedesc = 'Waiting for GPS fix'
        
        # Publish current vehicle status
        self.vehicle_state.header.stamp = rospy.Time.now()
        self.vehicle_state.name = self.statename
        self.vehicle_state.id = self.state
        self.vehicle_state.description = self.statedesc
        self.status_pub.publish(self.vehicle_state)         

        # Sleep for some time
        self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('vehicle_monitor', anonymous=True)
    fsm = FSM()
    # Run FSM while ros node is active
    while not rospy.is_shutdown():
        fsm.run()

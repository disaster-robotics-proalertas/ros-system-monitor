#!/usr/bin/env python

import rospy
import socket
from math import sqrt
from mavros_msgs.msg import RCIn
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
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
        self.rec_topic = rospy.get_param("/asv_description/%s/record_command_topic" % self.vehicle_name, default='/mavros/RC/in')
        self.rec_cmd_channel = rospy.get_param("/asv_description/%s/record_command_channel" % self.vehicle_name, default=5)
        self.rec_cmd_threshold = rospy.get_param("/asv_description/%s/record_command_threshold" % self.vehicle_name, default=20)

        # Publishers and subscribers
        self.status_pub = rospy.Publisher('%s/vehicle/state' % socket.gethostname(), VehicleState, queue_size=10)
        self.diag_agg_sub = rospy.Subscriber('/diagnostics_agg', DiagnosticArray, callback=self.diag_callback)
        self.rec_cmd_sub = rospy.Subscriber(self.rec_topic, RCIn, callback=self.rec_callback)
        
        # Messages
        self.rec_msg = RCIn()
        self.rec_nominal = RCIn()
        self.diag_agg_msg = DiagnosticArray()
        self.diag_toplevel_msg = DiagnosticStatus()
        self.vehicle_state = VehicleState()

    # Subscriber callbacks
    def rec_callback(self, msg):
        self.rec_msg = msg

    def diag_callback(self, msg):
        self.diag_agg_msg = msg

    def check_rec_cmd(self):
        # Check if RC PWM channel is within a certain threshold
        try:
            nominal_pwm = self.rec_nominal.channels[self.rec_cmd_channel]
            current_pwm = self.rec_msg.channels[self.rec_cmd_channel]
            if sqrt((current_pwm - nominal_pwm)**2) <= self.rec_cmd_threshold:
                return True
            else:
                return False
        except IndexError:
            return False
    
    # Main FSM function
    def run(self):
        # Get GPS diagnostics level in diagnostics aggregator 
        # From the way we structure the aggregator node, it is the second-to-last aggregated diagnostic
        gps_diag_level = self.diag_agg_msg.status[-2].level
        if self.state == VehicleState.ERROR:
            # If all modules are back online (toplevel status is warning or OK)
            if self.diag_toplevel_msg.level <= 1:
                self.state = VehicleState.BOOT
                self.statename = 'BOOT'
                self.statedesc = 'Waiting for GPS fix'
        elif self.state == VehicleState.BOOT:
            # If modules are healthy and GPS is fix
            if self.diag_toplevel_msg.level <= 1 and gps_diag_level <= 1:
                self.state = VehicleState.SERVICE
                self.statename = 'SERVICE'
                self.statedesc = 'Vehicle operational'
            # Get nominal RC PWM channel values
            self.rec_nominal = self.rec_msg
        elif self.state == VehicleState.SERVICE:
            # Check GPS diag level is error or stale
            if self.diag_toplevel_msg.level > 1:
                self.state = VehicleState.BOOT
                self.statename = 'BOOT'
                self.statedesc = 'Waiting for GPS fix'
            # Check if modules are unhealthy
            if self.diag_toplevel_msg.level > 1:
                self.state = VehicleState.ERROR
                self.statename = 'ERROR'
                self.statedesc = 'Toplevel %d' % self.diag_toplevel_msg.level
            # Check if record command on RC is enabled
            if self.check_rec_cmd():
                self.state = VehicleState.RECORDING
                self.statename = 'RECORDING'
                self.statedesc = 'Vehicle logging data'
        elif self.state == VehicleState.RECORDING:
            if not self.check_rec_cmd():
                self.state = VehicleState.SERVICE
                self.statename = 'SERVICE'
                self.statedesc = 'Vehicle operational'
        
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

    # Wait for messages to arrive in diagnostics topic
    rospy.wait_for_message('/diagnostics', DiagnosticArray)

    # Run FSM while ros node is active
    while not rospy.is_shutdown():
        fsm.run()

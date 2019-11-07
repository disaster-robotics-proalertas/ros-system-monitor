#!/usr/bin/env python

from diagnostic_msgs.msg import DiagnosticArray
import rospy
import rostopic
import socket
from system_monitor_msgs.msg import VehicleStatus

'''
This node verifies health for all modules, according to each module's monitors, and reports overall vehicle status
Statuses are:
0 - Error
1 - Boot
2 - Service
3 - Recording   (special state only activated through RC topic)
'''

def node():
    # Initialize node
    rospy.init_node("vehicle_monitor", anonymous=True)

    # Get active modules from YAML file parameters
    vehicle_name = socket.gethostname().replace('-', '_')
    modules = dict((el,0) for el in rospy.get_param('~/modules'))

    # Create system status publisher
    pub = rospy.Publisher('%s/status' % vehicle_name, VehicleStatus, queue_size=10)

    # ROS rate (10 Hz)
    rate = rospy.Rate(10)

    # Run while not shutdown
    vehicle = VehicleStatus()
    while not rospy.is_shutdown():
        vehicle.header.stamp = rospy.Time.now()
        
        # Check status for each module
        for modname in modules:
            diag_msg = rospy.wait_for_message("%s/status" % modname, DiagnosticArray)
            modules[modname] = diag_msg.status.level
        
        pub.publish(vehicle)

        rate.sleep()

if __name__ == '__main__':
    node()
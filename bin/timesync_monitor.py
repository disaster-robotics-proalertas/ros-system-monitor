#!/usr/bin/env python

from diagnostic_msgs.msg import DiagnosticArray
import rospy
import rostopic
import socket
import subprocess

'''
This node verifies all system monitor statuses and reports the overall system health
'''

def node():
    # Initialize node
    rospy.init_node("timesync_monitor", anonymous=True)

    # Get module name
    module_name = socket.gethostname().replace('-', '_')

    # Create system status publisher
    status_pub = rospy.Publisher('%s/diagnostics/timesync' % module_name, DiagnosticArray, queue_size=10)

    # ROS rate (10 Hz)
    rate = rospy.Rate(10)

    # Run while not shutdown
    sys_status = DiagnosticArray()
    while not rospy.is_shutdown():
        sys_status.header.stamp = rospy.Time.now()

        # Get timesyncd service status
        proc = subprocess.Popen('/bin/systemctl status timesyncd.service', shell=True, stdout=subprocess.PIPE)
        stdout = proc.communicate()[0].split('\n')
        daemon_status = stdout[3].split('status')[1].split('/')[1].strip(')')
        
        # Publish synchronization diagnostics accordingly
        if daemon_status == 'SUCCESS':
            sys_status.status.level = 0
        else:
            sys_status.status.level = 2

        status_pub.publish(sys_status)

        rate.sleep()

if __name__ == '__main__':
    node()
#!/usr/bin/env python

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rospy
import rostopic
import socket
import subprocess

def node():
    system_name = socket.gethostname()

    # Initialize node
    rospy.init_node("timesync_monitor_%s" % system_name, anonymous=True)

    # Create system status publisher
    status_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)

    # ROS rate (1 Hz)
    rate = rospy.Rate(1.0)

    # Define initial diagnostics message
    timesync = DiagnosticArray()
    timesync.status.append(DiagnosticStatus())
    timesync.status[0].level = 2
    timesync.status[0].name = 'timesyncd (%s)' % system_name
    timesync.status[0].hardware_id = system_name

    # Run while not shutdown
    while not rospy.is_shutdown():
        timesync.header.stamp = rospy.Time.now()

        # Get timesyncd service status
        proc = subprocess.Popen('/bin/systemctl status timesyncd.service', shell=True, stdout=subprocess.PIPE)
        try:
            stdout = proc.communicate()[0].split('\n')
            daemon_status = stdout[3].split('status')[1].split('/')[1].strip(')')
            
            # Publish synchronization diagnostics accordingly
            if daemon_status == 'SUCCESS':
                timesync.status[0].level = 0
            else:
                timesync.status[0].level = 2
            
            timesync.status[0].values = [ KeyValue(key = 'Time synchronization', value = daemon_status) ]
            status_pub.publish(timesync)
        except IndexError:
            break

        rate.sleep()

if __name__ == '__main__':
    node()
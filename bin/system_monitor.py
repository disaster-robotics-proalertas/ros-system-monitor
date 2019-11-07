#!/usr/bin/env python

from diagnostic_msgs.msg import DiagnosticArray
import rospy
import rostopic
import socket

'''
This node verifies all system monitor statuses and reports the overall system health
'''

def node():
    # Initialize node
    rospy.init_node("system_monitor", anonymous=True)

    # Get all topics in module's namespace with DiagnosticArray type
    module_name = socket.gethostname().replace('-', '_')
    diag_topics = []
    for topic in rostopic.find_by_type('diagnostic_msgs/DiagnosticArray'):
        if module_name in topic:
            diag_topics.append(topic)

    # Create system status publisher
    status_pub = rospy.Publisher('%s/status' % module_name, DiagnosticArray, queue_size=10)

    # ROS rate (10 Hz)
    rate = rospy.Rate(10)

    # Run while not shutdown
    sys_status = DiagnosticArray()
    while not rospy.is_shutdown():
        sys_status.header.stamp = rospy.Time.now()
        # Check status for all monitors running on the system
        # If all are OK, report overall status as ok
        for topic in diag_topics:
            topic_msg = rospy.wait_for_message(topic, DiagnosticArray)
            if topic_msg.status.level == 1:
                sys_status.status.level = 1
            elif topic_msg.status.level == 2:
                sys_status.status.level = 2
            else:
                sys_status.status.level = 0
        
        status_pub.publish(sys_status)

        rate.sleep()

if __name__ == '__main__':
    node()
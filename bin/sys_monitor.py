#!/usr/bin/env python

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
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
    module_name = rospy.get_param("/asv_description/system_name")
    diag_topics = []
    for topic in rostopic.find_by_type('diagnostic_msgs/DiagnosticArray'):
        if module_name in topic:
            diag_topics.append(topic)

    # Create system status publisher
    status_pub = rospy.Publisher('%s/status' % module_name, DiagnosticArray, queue_size=10)

    # ROS rate (1 Hz)
    rate = rospy.Rate(1)

    # We use a diagnostic array because DiagnosticStatus does not include header or timestamp information.......
    sys_status = DiagnosticArray()
    sys_status.status.append(DiagnosticStatus())

    # Run while not shutdown
    while not rospy.is_shutdown():
        sys_status.header.stamp = rospy.Time.now()
        # Check status for all monitors running on the system
        # If all are OK, report overall status as ok
        for topic in diag_topics:
            try:
                # 2 seconds timeout for diagnostics message
                topic_msg = rospy.wait_for_message(topic, DiagnosticArray, timeout=2.0)
                # Each monitor publishes a list of statuses
                # For example, CPU monitor publishes statuses on CPU temperature and usage 
                # If at least one module is in error (codes 0 - OK, 1 - Warning, 2 - Error), overall system is not healthy
                for comp in topic_msg.status:
                    if comp.level == 1:
                        sys_status.status[0].level = 1
                    elif comp.level == 2:
                        sys_status.status[0].level = 2
                    else:
                        sys_status.status[0].level = 0
            # This exception is thrown if wait_for_message has timed out, in which case the module is unresponsive
            except rospy.ROSException:
                sys_status.status[0].level = 2
        
        status_pub.publish(sys_status)

        rate.sleep()

if __name__ == '__main__':
    node()

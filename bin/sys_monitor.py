#!/usr/bin/env python

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import rospy
import socket

'''
This node verifies all system monitor statuses and reports the overall system health
'''

def node():
    # Initialize node
    rospy.init_node("system_monitor", anonymous=True)

    # Create system status publisher
    module_name = socket.gethostname()
    status_pub = rospy.Publisher('/system_monitor/%s/status' % module_name, DiagnosticArray, queue_size=10)

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
        try:
            # 5 seconds timeout for diagnostics message
            topic_msg = rospy.wait_for_message('/system_monitor/%s/diagnostics' % module_name, DiagnosticArray, timeout=5.0)
            # Each monitor publishes a list of statuses
            # For example, CPU monitor publishes statuses on CPU temperature and usage (components)
            # If at least one component is in error (codes 0 - OK, 1 - Warning, 2 - Error), overall system is not healthy
            for comp in topic_msg.status:
                if comp.level == 1:
                    sys_status.status[0].level = 1
                elif comp.level == 2:
                    sys_status.status[0].level = 2
                else:
                    sys_status.status[0].level = 0
        # This exception is thrown if wait_for_message has timed out, in which case the module is unresponsive
        except rospy.ROSException:
            rospy.logwarn("[sys_monitor] No response from /system_monitor/%s/diagnostics topic" % module_name)
            sys_status.status[0].level = 2
        
        status_pub.publish(sys_status)

        rate.sleep()

if __name__ == '__main__':
    node()

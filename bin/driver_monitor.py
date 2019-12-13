#!/usr/bin/env python

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rosnode
import rospy
import socket

class Monitor:
    def __init__(self, vehicle_name, **kwargs):
        self.vehicle_name = vehicle_name
        rospy.init_node("%s/driver_monitor" % self.vehicle_name, anonymous=True)
        self.diag_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)
        self.drivers = rospy.get_param("/asv_description/%s/sensor_drivers" % self.vehicle_name)
        self.rate = rospy.Rate(1)
        self.last_update = 0

    def run(self):
        nodes = rosnode.get_node_names()
        for driver in self.drivers:
            diag_msg = DiagnosticArray()
            diag_msg.status.append(DiagnosticStatus())
            diag_msg.status[0].hardware_id = driver.split('/')[1]

            # Check if message is stale (older than 35 seconds)
            elapsed = rospy.Time().now().to_sec() - self.last_update
            if elapsed > 35:
                diag_msg.status[0].level = DiagnosticStatus.STALE
                diag_msg.status[0].values.insert(0, KeyValue(key = '%s Status' % driver.split('/')[2], value = 'Stale'))
                diag_msg.status[0].values.insert(1, KeyValue(key = 'Time Since Update', value = str(elapsed)))
            else:
                if driver in nodes:
                    diag_msg.status[0].level = DiagnosticStatus.OK
                    diag_msg.status[0].values.insert(0, KeyValue(key = '%s Status' % driver.split('/')[2], value = 'Up'))
                    self.last_update = rospy.Time.now().to_sec()
                else:
                    diag_msg.status[0].level = DiagnosticStatus.ERROR
                    diag_msg.status[0].values.insert(0, KeyValue(key = '%s Status' % driver.split('/')[2], value = 'Down'))
                    self.last_update = rospy.Time.now().to_sec()  

            diag_msg.header.stamp = rospy.Time.now()
            self.diag_pub.publish(diag_msg)
        
        self.rate.sleep()

if __name__ == '__main__':
    monitor = Monitor(socket.gethostname())
    while not rospy.is_shutdown():
        monitor.run()

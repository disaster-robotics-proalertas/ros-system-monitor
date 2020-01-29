#!/usr/bin/env python

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rosnode
import rospy
import socket

class Monitor:
    def __init__(self, vehicle_name, **kwargs):
        self.vehicle_name = vehicle_name
        rospy.init_node("driver_monitor_%s" % self.vehicle_name, anonymous=True)
        self.diag_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)
        self.drivers = rospy.get_param("/asv_description/%s/sensor_drivers" % self.vehicle_name)
        self.rate = rospy.Rate(1)
        self.last_updates = {}

    def run(self):
        nodes = rosnode.get_node_names()
        status_list = []
        for driver in self.drivers:
            status = DiagnosticStatus()
            status.name = "%s Module Driver" % driver.split('/')[1]
            status.hardware_id = driver.split('/')[1]

            if driver in nodes:
                status.level = DiagnosticStatus.OK
                status.message = 'OK'
                status.values.insert(0, KeyValue(key = 'Driver Status', value = 'Up'))
                self.last_updates[status.hardware_id] = rospy.Time.now().to_sec()
            else:
                status.level = DiagnosticStatus.ERROR
                status.message = 'Module node not running'
                status.values.insert(0, KeyValue(key = 'Driver Status', value = 'Down'))
                self.last_updates[status.hardware_id] = rospy.Time.now().to_sec()

            # Check if message is stale (older than 35 seconds)
            try:
                elapsed = rospy.Time.now().to_sec() - self.last_updates[status.hardware_id]
            except KeyError:
                elapsed = 0
            if elapsed > 35:
                status.level = DiagnosticStatus.STALE
                status.message = 'Stale'
                status.values.insert(0, KeyValue(key = 'Update Status', value = 'Stale'))
                status.values.insert(1, KeyValue(key = 'Time Since Update', value = str(elapsed)))
            
            status_list.append(status)

        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = rospy.Time.now()
        for st in status_list:
            diag_msg.status.append(st)
        self.diag_pub.publish(diag_msg)
        
        self.rate.sleep()

if __name__ == '__main__':
    monitor = Monitor(socket.gethostname())
    while not rospy.is_shutdown():
        monitor.run()

#!/usr/bin/env python

import rospy

import traceback
import sys
import subprocess
import string
import re

import socket

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

net_level_warn = 0.95
net_capacity = 128

stat_dict = {0: 'OK', 1: 'Warning', 2: 'Error'}

def update_status_stale(stat, last_update_time):
  time_since_update = rospy.get_time() - last_update_time
  stale_status = 'OK'
  if time_since_update > 20 and time_since_update <= 35:
    stale_status = 'Lagging'
    if stat.level == DiagnosticStatus.OK:
      stat.message = stale_status
    elif stat.message.find(stale_status) < 0:
      stat.message = ', '.join([stat.message, stale_status])
    stat.level = max(stat.level, DiagnosticStatus.WARN)
  if time_since_update > 35:
    stale_status = 'Stale'
    if stat.level == DiagnosticStatus.OK:
      stat.message = stale_status
    elif stat.message.find(stale_status) < 0:
      stat.message = ', '.join([stat.message, stale_status])
    stat.level = max(stat.level, DiagnosticStatus.ERROR)
  stat.values.pop(0)
  stat.values.pop(0)
  stat.values.insert(0, KeyValue(key = 'Update Status', value = stale_status))
  stat.values.insert(1, KeyValue(key = 'Time Since Update',
    value = str(time_since_update)))

def get_sys_net_stat(iface, sys):
  cmd = 'cat /sys/class/net/%s/statistics/%s' %(iface, sys)
  p = subprocess.Popen(cmd,
                       stdout = subprocess.PIPE,
                       stderr = subprocess.PIPE, shell = True)
  stdout, _ = p.communicate()
  return (p.returncode, stdout.strip())

def get_sys_net(iface, sys):
  cmd = 'cat /sys/class/net/%s/%s' %(iface, sys)
  p = subprocess.Popen(cmd,
                       stdout = subprocess.PIPE,
                       stderr = subprocess.PIPE, shell = True)
  stdout, _ = p.communicate()
  return (p.returncode, stdout.strip())

class NetMonitor():
  def __init__(self, hostname, diag_hostname):
    self._diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size = 100)
    self._net_level_warn = rospy.get_param('~net_level_warn', net_level_warn)
    self._net_capacity = rospy.get_param('~net_capacity', net_capacity)
    self._usage_timer = None
    self._usage_stat = DiagnosticStatus()
    self._usage_stat.name = 'Network Usage (%s)' % diag_hostname
    self._usage_stat.level = 1
    self._usage_stat.hardware_id = hostname
    self._usage_stat.message = 'No Data'
    self._usage_stat.values = [KeyValue(key = 'Update Status',
                               value = 'No Data' ),
                               KeyValue(key = 'Time Since Last Update',
                               value = 'N/A') ]
    self._last_usage_time = 0
    self._last_publish_time = 0

  def check_network(self):
    values = []
    net_dict = {0: 'OK', 1: 'High Network Usage', 2: 'Network Down', 3: 'Call Error'}
    try:
      p = subprocess.Popen('ifstat -q -S 1 1',
                           stdout = subprocess.PIPE,
                           stderr = subprocess.PIPE, shell = True)
      stdout, _ = p.communicate()
      retcode = p.returncode
      if retcode != 0:
        values.append(KeyValue(key = "\"ifstat -q -S 1 1\" Call Error",
          value = str(retcode)))
        return DiagnosticStatus.ERROR, net_dict[3], values
      rows = stdout.split('\n')
      data = rows[0].split()
      ifaces = []
      for i in range(0, len(data)):
        ifaces.append(data[i])
      data = rows[2].split()
      kb_in = []
      kb_out = []
      for i in range(0, len(data), 2):
        kb_in.append(data[i])
        kb_out.append(data[i + 1])
      level = DiagnosticStatus.OK
      for i in range(0, len(ifaces)):
        values.append(KeyValue(key = 'Interface Name',
          value = ifaces[i]))
        (retcode, cmd_out) = get_sys_net(ifaces[i], 'operstate')
        if retcode == 0:
          values.append(KeyValue(key = 'State', value = cmd_out))
          ifacematch = re.match('eth[0-9]+', ifaces[i])
          if ifacematch and (cmd_out == 'down' or cmd_out == 'dormant'):
            level = DiagnosticStatus.ERROR
        values.append(KeyValue(key = 'Input Traffic',
          value = str(float(kb_in[i]) / 1024) + " (MB/s)"))
        values.append(KeyValue(key = 'Output Traffic',
          value = str(float(kb_out[i]) / 1024) + " (MB/s)"))
        net_usage_in = float(kb_in[i]) / 1024 / self._net_capacity
        net_usage_out = float(kb_out[i]) / 1024 / self._net_capacity
        if net_usage_in > self._net_level_warn or\
          net_usage_out > self._net_level_warn:
          level = DiagnosticStatus.WARN
        (retcode, cmd_out) = get_sys_net(ifaces[i], 'mtu')
        if retcode == 0:
          values.append(KeyValue(key = 'MTU', value = cmd_out))
        (retcode, cmd_out) = get_sys_net_stat(ifaces[i], 'rx_bytes')
        if retcode == 0:
          values.append(KeyValue(key = 'Total received MB',
            value = str(float(cmd_out) / 1024 / 1024)))
        (retcode, cmd_out) = get_sys_net_stat(ifaces[i], 'tx_bytes')
        if retcode == 0:
          values.append(KeyValue(key = 'Total transmitted MB',
            value = str(float(cmd_out) / 1024 / 1024)))
        (retcode, cmd_out) = get_sys_net_stat(ifaces[i], 'collisions')
        if retcode == 0:
          values.append(KeyValue(key = 'Collisions', value = cmd_out))
        (retcode, cmd_out) = get_sys_net_stat(ifaces[i], 'rx_errors')
        if retcode == 0:
          values.append(KeyValue(key = 'Rx Errors', value = cmd_out))
        (retcode, cmd_out) = get_sys_net_stat(ifaces[i], 'tx_errors')
        if retcode == 0:
          values.append(KeyValue(key = 'Tx Errors', value = cmd_out))
    except Exception, e:
      rospy.logerr(traceback.format_exc())
      msg = 'Network Usage Check Error'
      values.append(KeyValue(key = msg, value = str(e)))
      level = DiagnosticStatus.ERROR
    return level, net_dict[level], values

  def check_usage(self):
    diag_level = 0
    diag_vals = [KeyValue(key = 'Update Status', value = 'OK'),
                 KeyValue(key = 'Time Since Last Update', value = 0)]
    diag_msgs = []
    net_level, net_msg, net_vals = self.check_network()
    diag_vals.extend(net_vals)
    if net_level > 0:
      diag_msgs.append(net_msg)
    diag_level = max(diag_level, net_level)
    if diag_msgs and diag_level > 0:
      usage_msg = ', '.join(set(diag_msgs))
    else:
      usage_msg = stat_dict[diag_level]
    
    self._last_usage_time = rospy.get_time()
    self._usage_stat.level = diag_level
    self._usage_stat.values = diag_vals
    self._usage_stat.message = usage_msg

  def publish_stats(self):
    update_status_stale(self._usage_stat, self._last_usage_time)
    msg = DiagnosticArray()
    msg.header.stamp = rospy.get_rostime()
    msg.status.append(self._usage_stat)
    self._diag_pub.publish(msg)

if __name__ == '__main__':
  hostname = socket.gethostname()
  hostname = hostname.replace('-', '_')

  import optparse
  parser =\
    optparse.OptionParser(
    usage="usage: net_monitor.py [--diag-hostname=cX]")
  parser.add_option("--diag-hostname", dest="diag_hostname",
                    help="Computer name in diagnostics output (ex: 'c1')",
                    metavar="DIAG_HOSTNAME",
                    action="store", default = hostname)
  options, args = parser.parse_args(rospy.myargv())
  try:
    rospy.init_node('net_monitor_%s' % hostname)
  except rospy.exceptions.ROSInitException:
    print >> sys.stderr,\
      'Network monitor is unable to initialize node. Master may not be running.'
    sys.exit(0)
  net_node = NetMonitor(hostname, options.diag_hostname)
  rate = rospy.Rate(1.0)
  try:
    while not rospy.is_shutdown():
      rate.sleep()
      net_node.check_usage()
      net_node.publish_stats()
  except KeyboardInterrupt:
    pass
  except Exception, e:
    traceback.print_exc()
    rospy.logerr(traceback.format_exc())
  sys.exit(0)

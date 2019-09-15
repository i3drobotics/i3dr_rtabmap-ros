#!/usr/bin/env python

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from rtabmap_ros.srv import GetMap, GetMapRequest, GetMapResponse
import rospy

class RtabmapTransfer:
    def __init__(self):
        self.setupNode()

    def setupNode(self):
        self.rtabmap_cloud = None
    
        rospy.init_node("i3dr_rtabmap_transfer", anonymous=True, disable_signals=True)
        rospy.Service('i3dr_scan_send_map', Empty, self.handle_i3dr_scan_send_map)
        rospy.Service('i3dr_scan_pause', Empty, self.handle_i3dr_scan_pause)
        rospy.Service('i3dr_scan_resume', Empty, self.handle_i3dr_scan_resume)
        rospy.Service('i3dr_scan_new_map', Empty, self.handle_i3dr_scan_new_map)

        self.rtabmap_namespace = rospy.get_param('~rtabmap_namespace', "rtabmap")

        self.sub_rtabmap_cloud = rospy.Subscriber('rtabmap/cloud_map', PointCloud2, self.callback_rtabmap_cloud)

        self.pub_i3dr_scan_map = rospy.Publisher('i3dr_scan_map', PointCloud2, queue_size=20)

    def spin(self,rate_hz):
        rate = rospy.Rate(rate_hz)
        try:
            while not rospy.is_shutdown():
                rate.sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

    def callback_rtabmap_cloud(self,data):
        rospy.loginfo("rtabmap cloud received")
        self.rtabmap_cloud = data
        self.rtabmap_cloud_received = True

    def rtabmap_get_map(self):
        try:
            rospy.loginfo("running rtabmap get_map...")
            rospy.wait_for_service('/'+self.rtabmap_namespace+'/get_map_data', timeout=2)
            srv_get_map = rospy.ServiceProxy('/'+self.rtabmap_namespace+'/get_map_data', GetMap)
            req = GetMapRequest(True,True,False)
            # request map and wait for response
            resp = srv_get_map(req)
            rospy.loginfo("rtabmap get_map sucessful")
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s",e)
        except rospy.ROSException, e:
            rospy.logerr("Service call failed: %s",e)
        except:
            rospy.logerr("Service call failed: Unknown error")

    def rtabmap_pause(self):
        print "Pausing rtabmap with service: /"+self.rtabmap_namespace+"/pause"
        try:
            rospy.wait_for_service(
                '/'+self.rtabmap_namespace+'/pause', timeout=2)
            srv_pause = rospy.ServiceProxy(
                '/'+self.rtabmap_namespace+'/pause', Empty)
            srv_pause()
            self.rtabmap_is_paused = True
            print "Rtabmap service complete: /"+self.rtabmap_namespace+"/pause"
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
        except rospy.ROSException, e:
            rospy.logerr("Service call failed: %s", e)

    def rtabmap_pause_odom(self):
        print "Pausing rtabmap odometry with service: /" + \
            self.rtabmap_namespace+"/pause_odom"
        try:
            rospy.wait_for_service(
                '/'+self.rtabmap_namespace+'/pause_odom', timeout=2)
            srv_pause_odom = rospy.ServiceProxy(
                '/'+self.rtabmap_namespace+'/pause_odom', Empty)
            srv_pause_odom()
            self.rtabmap_is_paused = True
            print "Rtabmap service complete: /"+self.rtabmap_namespace+"/pause_odom"
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
        except rospy.ROSException, e:
            rospy.logerr("Service call failed: %s", e)

    def rtabmap_resume(self):
        try:
            rospy.wait_for_service(
                '/'+self.rtabmap_namespace+'/resume', timeout=2)
            srv_resume = rospy.ServiceProxy(
                '/'+self.rtabmap_namespace+'/resume', Empty)
            srv_resume()
            self.rtabmap_is_paused = False
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
        except rospy.ROSException, e:
            rospy.logerr("Service call failed: %s", e)

    def rtabmap_resume_odom(self):
        try:
            rospy.wait_for_service(
                '/'+self.rtabmap_namespace+'/resume_odom', timeout=2)
            srv_reset = rospy.ServiceProxy(
                '/'+self.rtabmap_namespace+'/resume_odom', Empty)
            srv_reset()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
        except rospy.ROSException, e:
            rospy.logerr("Service call failed: %s", e)

    def rtabmap_reset_odom(self):
      try:
          rospy.wait_for_service(
                '/'+self.rtabmap_namespace+'/reset_odom', timeout=2)
          srv_reset_odom = rospy.ServiceProxy(
                '/'+self.rtabmap_namespace+'/reset_odom', Empty)
          srv_reset_odom()
      except rospy.ServiceException, e:
          rospy.logerr("Service call failed: %s", e)
      except rospy.ROSException, e:
          rospy.logerr("Service call failed: %s", e)

    def rtabmap_new_map(self):
        try:
            rospy.wait_for_service(
                '/'+self.rtabmap_namespace+'/trigger_new_map', timeout=2)
            srv_new_map = rospy.ServiceProxy(
                '/'+self.rtabmap_namespace+'/trigger_new_map', Empty)
            srv_new_map()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
        except rospy.ROSException, e:
            rospy.logerr("Service call failed: %s", e)

    def rtabmap_reset(self):
        try:
            rospy.wait_for_service(
                '/'+self.rtabmap_namespace+'/reset', timeout=2)
            srv_reset = rospy.ServiceProxy(
                '/'+self.rtabmap_namespace+'/reset', Empty)
            srv_reset()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
        except rospy.ROSException, e:
            rospy.logerr("Service call failed: %s", e)

    def handle_i3dr_scan_send_map(self,req):
        print("request to send latest map")
        self.rtabmap_get_map()
        if (self.rtabmap_cloud):
            self.pub_i3dr_scan_map.publish(self.rtabmap_cloud)
        else:
            rospy.logerr("Unable to send map. No data found in map!")
        return EmptyResponse()

    def handle_i3dr_scan_pause(self, req):
          self.rtabmap_pause()
          self.rtabmap_pause_odom()
          return EmptyResponse()

    def handle_i3dr_scan_resume(self, req):
          self.rtabmap_resume()
          self.rtabmap_resume_odom()
          return EmptyResponse()

    def handle_i3dr_scan_new_map(self, req):
          self.rtabmap_new_map()
          self.rtabmap_reset()
          self.rtabmap_reset_odom()
          self.rtabmap_get_map()
          return EmptyResponse()

if __name__ == '__main__':
    rt = RtabmapTransfer()
    rt.spin(100)


#! /usr/bin/env python2.7
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg         import PoseStamped, Pose
from porszilo_telepresence.srv import ClickedPoint
from sensor_msgs.msg           import Image
from nav_msgs.msg              import OccupancyGrid, Odometry
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class Telepresence():
    ## global variables:
        # color
        # depth
        # grid
        # odom
        # goal
    
    
    ## subscriber callbacks
    def cbColor(self, data):
        self.color = data
        self.cv_color = CvBridge().imgmsg_to_cv2(data, desired_encoding="passthrough")

        #  self.color = data
        #  cv_color = CvBridge().imgmsg_to_cv2(color, desired_encoding="passthrough")
        #  cv2.imshow('image', cv_color)
        #  cv2.waitKey(1)
    
    def cbDepth(self, data):
        self.depth = data
        self.cv_depth = CvBridge().imgmsg_to_cv2(data, desired_encoding="passthrough")
    
    def cbGrid(self, data):
        self.grid = data
    
    def cbOdom(self, data):
        self.odom = data
    
    
    ## loop
    def run(self):
        # 2D pic with the clickable areas marked
        # published to the browser

        self.getClickablePic()# fills self.clickable_pic

        self.imagePub.publish(CvBridge().cv2_to_imgmsg(self.clickable_pic, encoding="passthrough"))
        pass
    
    #!!!!!!!!!!!!!!!!!! már csak ez nincs meg
    def getClickablePic(self):
        # calculate 2D pic with the clickable areas marked
        self.clickable_pic = self.cv_depth
        #  rospy.loginfo("OccupancyGrid origin: \n{}".format(self.grid.info.origin))
        pose = PoseStamped()
        pose.header.frame_id = self.grid.header.frame_id
        pose.header.stamp = rospy.Time.now()
        #  rospy.loginfo("Grid frame id: {}".format(pose.header.frame_id))
        pose.pose = self.grid.info.origin
        self.orig_pub.publish(pose)

        #  self.grid_image = Image()
        #  self.grid_image.header = self.grid.header
        #  self.grid_image.height = self.grid.info.height
        #  self.grid_image.width  = self.grid.info.width
        #  self.grid_image.data   = self.grid.data
        #  self.cv_grid_image = CvBridge().imgmsg_to_cv2(self.grid_image, desired_encoding="passthrough")



    #!!!!!!!!!!!!!!!!!! meg ez
    def clickedTo3D(self):

        #########################################################################
        ####################      ezzel szarakodtunk ma      ####################
        #########################################################################

        #  rospy.loginfo("Actual size: {}".format(len(self.depth.data)))
        #  rospy.loginfo("Should-be size: {}".format(self.depth.width*self.depth.height))

        #  rospy.loginfo("First ten: {}".format(self.depth.data[:10]))
        #  rospy.loginfo("Array shape:".format(np.array(self.depth.data).shape))
        #  shape_ = np.reshape(np.array(self.depth.data), (self.depth.width, self.depth.height, 4))
        #  rospy.loginfo("Shape: {}".format(shape_))
        #  self.clickedDepth =[self.point_x, self.point_y, 0]

        rospy.loginfo("Clicked point: {}, {}".format(self.point_x, self.point_y))
        #  rospy.loginfo("Color width: {}, height: {}".format(self.color.width, self.color.height))
        #  rospy.loginfo("Depth width: {}, height: {}".format(self.depth.width, self.depth.height))
        #  rospy.loginfo("Shape: {}".format(self.cv_depth.shape))
        #  rospy.loginfo("Clicked depth: {}".format(self.cv_depth[self.point_y, self.point_x]))

        #  neigh = 5
        #  rospy.loginfo("Neighborhood of clicked point: \n{}".format(self.cv_depth[self.point_y-neigh:self.point_y+neigh, self.point_x-neigh:self.point_x+neigh]))

        #  cv2.imshow("anyad", self.cv_depth == np.nan)
        #  tmp = np.array(self.cv_depth)
        #  cv2.waitKey(0)
        #  cv2.destroyAllWindows()

        #########################################################################
        ####################      ezzel szarakodtunk ma      ####################
        #########################################################################

        img_width  = 700
        img_height = 500 # az index.html-ből

        actual_clicked_width  = int((self.point_y - 1) * self.depth.width / img_width)
        actual_clicked_height = int(self.point_x * self.depth.height / img_height)

        rospy.loginfo("actual_clicked: {}, {}".format(actual_clicked_width, actual_clicked_height))
        ## még ez sem működik

        idx = actual_clicked_width * self.depth.width + actual_clicked_height
        clicked_data = self.depth.data[idx]
        rospy.loginfo("Got depth: {}".format(clicked_data))
        rospy.loginfo("Type of got depth: {}".format(type(clicked_data)))

        self.goal.header.stamp = rospy.Time.now()
        #  self.goal.pose.position.x = ...
        #  self.goal.pose.position.y = ...
        #  self.goal.pose.position.z = ...

        
    def cbClickedPoint(self, req):
        # ebbol megvan a pixel helye: req.x, req.y
        # ha click-elnek, itt dolgozzuk fel es kuldjuk el
        # (pixel -> 3D trafo, aztan publishelni)
        # innen nem kell beszelni a web klienssel
        self.point_x = req.x
        self.point_y = req.y

        self.clickedTo3D()# fills self.goal
        self.goalPub.publish(self.goal)
    
    def main(self):
        rospy.init_node('telepresence')
    
        self.freq = rospy.get_param("~freq", 4) # looped at 4 Hz by default
        self.view_angle_v = rospy.get_param("~view_angle_v")
        self.view_angle_h = rospy.get_param("~view_angle_h")
    
        rospy.loginfo('Waiting for messages to arrive...')
        try:
            # elsőre megvárjuk hogy minden legalább egyszer megérkezzen, így minden
            # define-olva lesz és mindenben lesz adat
            self.cbColor(rospy.wait_for_message("/camera/color/image_raw", Image))
            self.cbDepth(rospy.wait_for_message("/camera/depth/image_raw", Image))
            self.cbGrid(rospy.wait_for_message("/move_base/global_costmap/costmap", OccupancyGrid))
            self.cbOdom(rospy.wait_for_message("/odometry/filtered", Odometry))

            self.goal = PoseStamped()
        except rospy.ROSInterruptException:
            rospy.loginfo("Telepresence node is being shut down")
            quit()

        rospy.Subscriber("/camera/color/image_raw", Image, self.cbColor)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.cbDepth)
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.cbGrid)
        rospy.Subscriber("/odometry/filtered", Odometry, self.cbOdom)
    
        self.orig_pub = rospy.Publisher("origin", PoseStamped, queue_size=1)
        rospy.loginfo('All messages have arrived at least once, starting publishers')
        self.goalPub  = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=5)
        self.imagePub = rospy.Publisher("/telepresence/image", Image, queue_size=5)

        # think of this as a remote function call, called by the  JS code in the browser
        self.srv_clicked_point = rospy.Service("/telepresence/clicked_point", ClickedPoint, self.cbClickedPoint)
    
        rate = rospy.Rate(self.freq)
        try:
            while not rospy.is_shutdown():
                self.run()# <- this is being looped
                rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("Telepresence node is being shut down")
            quit()


if __name__ == "__main__":

    tp = Telepresence()
    tp.main()

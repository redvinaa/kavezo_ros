#! /usr/bin/env python2.7
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg         import PoseStamped
from porszilo_telepresence.srv import ClickedPoint
from sensor_msgs.msg           import Image
from nav_msgs.msg              import OccupancyGrid, Odometry
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import matplotlib.pyplot as plt

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

        #  self.imagePub.publish(CvBridge().cv2_to_imgmsg(self.clickable_pic, encoding="passthrough"))
        self.imagePub.publish(CvBridge().cv2_to_imgmsg(self.clickable_pic, encoding="passthrough"))
        
    
    #!!!!!!!!!!!!!!!!!! már csak ez nincs meg
    def getClickablePic(self):
        # calculate 2D pic with the clickable areas marked
	
	
	# az általam rajzolt térképen 1 pixel 1 cm a valóságban

	pos_y = int(round(self.odom.pose.pose.position.y/self.grid.info.resolution + self.grid.info.height/2)) # pozíciónk a térképen pixelben
	pos_x = int(round(self.odom.pose.pose.position.x/self.grid.info.resolution + self.grid.info.width/2))
	h = int(round(self.odom.pose.pose.position.z/self.grid.info.resolution))+5 # a kamer optikai tengelyének magassága a földtől pixel egységgé alakítva
	view_angle_v = self.view_angle_v  # kamera vertikális látószöge fokban
	view_angle_h = self.view_angle_h   # kamera horizontális látószöge fokban

	qx=self.odom.pose.pose.orientation.x
	qy=self.odom.pose.pose.orientation.y
	qz=self.odom.pose.pose.orientation.z
	qw=self.odom.pose.pose.orientation.w
	siny_cosp = 2 * (qw * qz + qx * qy)
	cosy_sinp = 1 - 2 * (qy * qy + qz * qz)
	yaw = math.atan2(siny_cosp, cosy_sinp)
	
	deg = yaw * 180 / math.pi + 90 # hány fokkal kell elforgatnunk a térépet hogy a robot nézési iránya függőlegesen legyen
	rospy.loginfo(deg)

	orig_map = np.array(self.grid.data)  # beolvassuk a térképet és a látott képet
	orig_map = np.where(orig_map==-1, 0, 255)
	
	orig_map = np.reshape(orig_map,(self.grid.info.height, self.grid.info.width)).astype('uint8')
	orig_map = cv2.flip(orig_map, 0)

	
	cam = self.cv_color

	self.clickable_pic = orig_map
	#cv2.imshow("a", orig_map)
	#cv2.waitKey()
	
	#plt.imshow(orig_map)
	#plt.show()


	cam_gray = cv2.cvtColor(cam, cv2.COLOR_BGR2GRAY)

	# cv2.imshow('map', map)
	# cv2.waitKey()

	map_cols = orig_map.shape[1]
	map_rows = orig_map.shape[0]

	cam_cols = cam.shape[1]
	cam_rows = cam.shape[0]

	map_diag = int(round(1.2 * (math.sqrt(math.pow(map_cols, 2) + math.pow(map_rows, 2)))))  # egy nagy térkép amibe beletéve a kicsit szabadon el tudjuk forgatni
	big_map = np.zeros((map_diag, map_diag), 'uint8')

	shift_cols = int(round((map_diag - map_cols) / 2))  # beletesszük a közepére a térképet
	shift_rows = int(round((map_diag - map_rows) / 2))
	big_map[shift_rows:(map_rows + shift_rows), shift_cols:(map_cols + shift_cols)] = orig_map

	pos_y_big = shift_rows + pos_y  # átszámítva a koordinátkat a nagy térképre
	pos_x_big = shift_cols + pos_x

	# big_map[pos_y_big,pos_x_big]=0
	# cv2.imshow('big_map', big_map)
	# cv2.waitKey()


	M_rot = cv2.getRotationMatrix2D((map_diag / 2, map_diag / 2), deg, 1)  # elforgatjuk hogy függőlegesen fölfelé legyen a robot nézési iránya
	map_rot = cv2.warpAffine(big_map, M_rot, (map_diag, map_diag))

	# Hol vagyunk a forgatás után, a nagy térképen?
	big_map_half = int(round(map_diag / 2))
	rad = deg * math.pi / 180
	pos_x_rot = int((pos_x_big - big_map_half) * math.cos(rad) + (pos_y_big - big_map_half) * math.sin(rad) + big_map_half)
	pos_y_rot = int(-(pos_x_big - big_map_half) * math.sin(rad) + (pos_y_big - big_map_half) * math.cos(rad) + big_map_half)

	# map_rot[pos_y_rot, pos_x_rot] = 0
	# cv2.imshow('map_rot', map_rot)
	# cv2.waitKey()


	# keressük meg a perspektívikusan torzítandó képrészlet 4 sarkát
	# ez itt puszta koordinátageometria kiegyszerűsítve, átláthatatlanul

	# print(pos_y_rot,pos_x_rot)

	y3 = pos_y_rot - int(round(h * math.tan((90 - view_angle_v / 2) * math.pi / 180)))  # 324
	y4 = np.where(big_map == 255)[0][0]  # 14

	rospy.loginfo("view_angle_h = {}".format(view_angle_h))
	assert(view_angle_h != 180)
	m = math.tan((90 - view_angle_h / 2) * math.pi / 180)  # 1.28
	b1 = pos_y_rot + m * pos_x_rot  # 711.6
	b2 = pos_y_rot - m * pos_x_rot  # 20.4

	assert(m)
	p1x = int(round((y3 - b2) / m))
	p1y = int(round(m * p1x + b2))

	p2x = int(round((y4 - b2) / m))
	p2y = int(round(m * p2x + b2))

	p3x = int(round((-y4 + b1) / m))
	p3y = int(round(-m * p3x + b1))

	p4x = int(round((-y3 + b1) / m))
	p4y = int(round(-m * p4x + b1))

	# map_rot[p1y, p1x] = 0
	# map_rot[p4y, p4x] = 0
	# map_rot[p2y, p2x] = 255
	# map_rot[p3y, p3x] = 255
	# cv2.imshow('map_rot2', map_rot)
	# cv2.waitKey()

	# mennyire kell összenyomni a képet
	beta = math.atan((pos_y_rot - y4) / h) * 180 / math.pi - 90 + view_angle_v / 2
	ratio = beta / view_angle_v
	print(ratio)

	output_rows = int(round(cam_rows * ratio))
	output_cols = int(round(cam_cols))

	pts1 = np.float32([[p2x, p2y], [p3x, p3y], [p1x, p1y], [p4x, p4y]])
	pts2 = np.float32([[0, 0], [output_cols, 0], [0, output_rows], [output_cols, output_rows]])
	M_persp = cv2.getPerspectiveTransform(pts1, pts2)
	output = cv2.warpPerspective(map_rot, M_persp, (output_cols, output_rows))
	output = output

	output2 = np.array([cam_gray], dtype=int)
	output2[0, cam_rows - output_rows - 1:-1, :] = output
	output2 = np.where(output2 == 0, cam_gray, output2)

	final2 = np.zeros((cam_rows, cam_cols, 3))
	final2[:, :, 0] = cam_gray
	final2[:, :, 1] = output2
	final2[:, :, 2] = cam_gray


	#plt.matshow(final2)
	#  cv2.imshow("lofasz", final2 / 255.0)
	#  cv2.waitKey(1)
	#cv2.imwrite("D:\\BME\\MSZO\\hostess robot\\2020\\kep2_res_rgb.jpg", final2)
	self.clickable_pic = final2


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
        rospy.loginfo("Shape: {}".format(self.cv_depth.shape))
        rospy.loginfo("Clicked depth: {}".format(self.cv_depth[self.point_y, self.point_x]))

        #  cv2.imshow("anyad", self.cv_depth == np.nan)
        tmp = np.array(self.cv_depth)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        #########################################################################
        ####################      ezzel szarakodtunk ma      ####################
        #########################################################################


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
    
        freq = rospy.get_param("frequency", 10) # default 10 Hz
	self.view_angle_v = rospy.get_param("view_angle_v", 50)
	self.view_angle_h = rospy.get_param("view_angle_h", 76)
    
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
    
        rospy.loginfo('All messages have arrived at least once, starting publishers')
        self.goalPub  = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=5)
        self.imagePub = rospy.Publisher("/telepresence/image", Image, queue_size=5)

        # think of this as a remote function call, called by the  JS code in the browser
        self.srv_clicked_point = rospy.Service("/telepresence/clicked_point", ClickedPoint, self.cbClickedPoint)
    
        rate = rospy.Rate(freq)
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

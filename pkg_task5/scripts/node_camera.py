#!/usr/bin/env python

import rospy
from cv2 import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospkg

import time
import datetime

from RepMeth import IotRosBridgeActionClient

import numpy as np

import yaml


class Camera1:
    '''
    A class to represent camera module
    '''

    def __init__(self):
        '''
        Construts parameters for Camera1 Object
        '''

        rospy.logwarn('Start Camera')
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber(
            "/eyrc/vb/camera_1/image_raw", Image, self.callback, queue_size=1)

        self.pkg_color = []
        self.pkg_color_2d = []

        self.pkg_det = {}
        self.pkg_det['models'] = {}

    def get_dominant_colour(self, arg_img):
        '''
        Function to get Dominant color from image

        :param arg_img: Image array
        :type arg_img: list
        '''

        # setting values for base colors
        b = arg_img[:, :, :1]
        g = arg_img[:, :, 1:2]
        r = arg_img[:, :, 2:]

        # computing the mean
        b_mean = int(np.mean(b))
        g_mean = int(np.mean(g))
        r_mean = int(np.mean(r))

        # print(b_mean, g_mean, r_mean)

        if(b_mean == g_mean == r_mean):
            self.pkg_color.append('Empty')
        elif (g_mean == r_mean):
            self.pkg_color.append('Yellow')
        elif (g_mean > r_mean and g_mean > b_mean):
            self.pkg_color.append('Green')
        elif(r_mean > g_mean and r_mean > b_mean):
            self.pkg_color.append('Red')
        else:
            self.pkg_color.append('Something else')

    def split_img(self, arg_img):
        '''
        Divides Image into cells

        :param arg_img: Image Array
        :type arg_img: list
        '''

        sizeX = arg_img.shape[0]
        sizeY = arg_img.shape[1]
        cv2.waitKey()

        x = int(sizeX * 0.065)
        y = int(sizeY * 0.42)

        h = sizeY
        w = sizeX

        img = arg_img[y:sizeX-int(0.68 * y), x:sizeY-x]
        (length, width, size) = img.shape

        x = length/5
        y = width/3

        image_grid = []

        for i in range(5):
            for j in range(3):
                image_grid.append(img[i*x:(i+1) * x:, j*y:(j+1) * y])

        for i in range(len(image_grid)):

            self.get_dominant_colour(image_grid[i])

        self.pkg_color_2d = [self.pkg_color[i: i+3]
                             for i in range(0, len(self.pkg_color), 3)]

        for i in range(4):
            for j in range(3):
                name = 'packagen' + str(i) + str(j)
                self.pkg_det['models'][name] = {
                    'color': self.pkg_color_2d[i][j]
                }
        rospy.loginfo(self.pkg_det)

        rospy.set_param('models', self.pkg_det['models'])

        # rospy.logwarn(self.pkg_color_2d)
        self.image_sub.unregister()

    def callback(self, data):
        '''
        Callback function for 2D Camera topic

        :param data: Image Array
        :type data: list
        '''
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        (rows, cols, channels) = cv_image.shape

        image = cv_image

        # Resize a 720x1280 image to 360x640 to fit it on the screen
        resized_image = cv2.resize(image, (720/2, 1280/2))

        # cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)

        self.split_img(resized_image)

        cv2.waitKey(0)


class InventoryClient:
    '''
    A class to for Inventory Client
    '''

    def __init__(self):
        '''
        Constructs all parameets for InventoryClient class
        '''
        self._team_id = 'VB#0393'
        self._unique_id = 'jgHsFpUv'
        self._spreadsheet_id_1 = 'AKfycbxZ0aU9jx8KPq52NhaIducZK257E0E9ecnwRmfvT-wmlTX5KeTbbiHQ'

        self.param_models = ''

        self.pkg_list = ['packagen00', 'packagen01', 'packagen02', 'packagen10', 'packagen11', 'packagen12',
                         'packagen20', 'packagen21', 'packagen22', 'packagen30', 'packagen31', 'packagen32']

        self.pos_R = ''
        self.pos_C = ''
        self.code = ''
        self.item = ''
        self.cost = ''
        self.priority = ''
        self.quan = 1
        self.SKU = ''
        self.sto_no = ''

        self.goal_id = 0

        self.action_client = IotRosBridgeActionClient()

    def get_time_str(self):
        '''
        Generates Time String

        :return: Time String
        :rtype: str
        '''

        timestamp = int(time.time())
        value = datetime.datetime.fromtimestamp(timestamp)
        str_time = value.strftime('%Y-%m-%d %H:%M:%S')

        return str_time

    def push(self):
        '''
        Function to publish all data to mqtt topic
        '''

        for i in self.pkg_list:

            color = self.param_models[i]['color']
            rospy.logwarn(color)

            self.pos_R, self.pos_C = i[-2], i[-1]
            self.sto_no = 'R' + self.pos_R + ' C' + self.pos_C

            str_time = self.get_time_str()

            month = str_time[5:7]
            year = str_time[2:4]

            if color == 'Red':
                self.priority = 'HP'
                self.item = 'Medicine'
                self.code = 'R'
                self.cost = '350'
            elif color == 'Yellow':
                self.priority = 'MP'
                self.item = 'Food'
                self.code = 'Y'
                self.cost = '250'
            elif color == 'Green':
                self.priority = 'LP'
                self.item = 'Clothes'
                self.code = 'G'
                self.cost = '150'

            self.SKU = self.code + self.pos_R + self.pos_C + month + year
            self.goal_id += 1

            self.pub_mqtt()

    def pub_mqtt(self):
        '''
        Sends a goal to Action Server
        '''

        msg = '{} {} {} {} {}'.format(
            self.SKU, self.item, self.sto_no, self.quan, 'Inventory')

        goal_handle = self.action_client.send_goal(
            "mqtt", "pub", self.action_client._config_mqtt_pub_topic, msg)

        self.action_client._goal_handles[str(self.goal_id)] = goal_handle

        rospy.loginfo('MQTT Goal Sent')

        rospy.sleep(2)


def main():
    '''
    Object Creation main execution of node
    '''

    rospy.logwarn('Camera')

    rospy.sleep(25)

    rospy.init_node('node_camera', anonymous=True)

    ic = Camera1()

    rospy.sleep(5)

    inv_client = InventoryClient()
    while True:
        try:
            inv_client.param_models = rospy.get_param('models')
            break
        except:
            pass

    inv_client.push()


if __name__ == '__main__':
    '''
    Main Function
    '''
    try:
        main()
    except rospy.ROSInterruptException:
        pass

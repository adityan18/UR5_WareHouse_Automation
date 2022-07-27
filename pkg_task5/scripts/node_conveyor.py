#! /usr/bin/env python

import rospy
import sys
import copy
import threading
import yaml
import os
import time
import rospkg
import math

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib


from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse

from pkg_vb_sim.srv import ConveyorBeltControl
from pkg_vb_sim.srv import ConveyorBeltControlRequest
from pkg_vb_sim.srv import ConveyorBeltControlResponse

from pkg_vb_sim.srv import vacuumGripperResponse
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripper

from pkg_vb_sim.msg import LogicalCameraImage

from std_srvs.srv import Empty


class Conveyor:

    '''
    A class for handling the Conveyor belt
    '''

    # Constructor
    def __init__(self):
        '''
        Constructs all parameters for Conveyor object
        '''

        rospy.init_node('node_conveyor', anonymous=True)

        self._conveyor_belt_power = '/eyrc/vb/conveyor/set_power'
        self._logical_camera_topic = '/eyrc/vb/logical_camera_2'
        self._vacuum_gripper = '/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2'

        self._power = 90
        self._stop = 0

        self._belt_state = 1
        self.pkg_list = ['packagen00', 'packagen01', 'packagen02', 'packagen10', 'packagen11', 'packagen12',
                         'packagen20', 'packagen21', 'packagen22', 'packagen30', 'packagen31', 'packagen32']

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

    def conveyor_manager(self):
        '''
        Function that is used to stop the conveyor belt when the package reaches the desired postition
        '''

        rospy.wait_for_service(self._conveyor_belt_power)

        try:
            s2 = rospy.ServiceProxy(
                self._conveyor_belt_power, conveyorBeltPowerMsg)
        except rospy.ServiceException:
            rospy.logerr('Service Call Failed')

        rospy.logwarn('**************Stoping Belt  UR_2**************')
        rospy.loginfo(s2(self._stop).result)
        self._belt_state = 0

    def logical_cam_sub_callback(self, msg):
        '''
        Logical Camera Subscription Callback Function

        :param msg: message from topic
        :type msg: dict
        '''

        no_of_models = len(msg.models)

        if(no_of_models == 0 and self._belt_state == 0):

            rospy.wait_for_service(self._conveyor_belt_power)

            try:
                s2 = rospy.ServiceProxy(
                    self._conveyor_belt_power, conveyorBeltPowerMsg)
            except rospy.ServiceException:
                rospy.logerr('Service Call Failed')

            rospy.logwarn('**************Restarting Belt**************')
            rospy.loginfo(s2(self._power).result)
            self._belt_state = 1

        for i in range(0, no_of_models):
            self._model_name = msg.models[i].type
            # rospy.loginfo(self._model_name)
            if(self._model_name in self.pkg_list):
                pose = msg.models[i].pose
                self.pose_y = pose.position.y
                self.pose_x = pose.position.x
                self.pose_z = pose.position.z

                if(self.pose_y < 0 and self.pose_x > 1):
                    print(self.pose_x, self.pose_y, self.pose_z)
                    self.conveyor_manager()

    def __del__(self):
        '''
        Destroy all parameters for CConveyor object
        '''
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Conveyor Deleted." + '\033[0m')


def main(argv):
    '''
    Object Creation and main execution of node

    :param argv: sys argv
    :type argv: list
    '''
    rospy.sleep(25)
    # node_camera.main()

    ur5 = Conveyor()
    # Starting Conveyor Belt

    rospy.Subscriber(ur5._logical_camera_topic,
                     LogicalCameraImage, ur5.logical_cam_sub_callback, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    '''
    Main Function
    '''
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass

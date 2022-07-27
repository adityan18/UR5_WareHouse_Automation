#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import rospkg

import yaml
import os
import time

from std_srvs.srv import Empty

from pkg_vb_sim.srv import vacuumGripperResponse
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripper

from pkg_ros_iot_bridge.msg import msgRosIotAction
# Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotGoal
# Message Class that is used for Result Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult
# Message Class for MQTT Subscription Messages
from pkg_ros_iot_bridge.msg import msgMqttSub

from RepMeth import IotRosBridgeActionClient


import paho.mqtt.client as mqtt
import time
import datetime


class UR5_1:
    '''
    A class for controlling UR5_1 Arm
    '''

    def __init__(self, arg_robot_name):
        '''
        Constructs all parameters for UR5_1 Object

        :param arg_robot_name: Robot Arm Name
        :type arg_robot_name: str
        '''

        rospy.init_node('node_ur5_1', anonymous=True)

        self._robot_ns = '/' + arg_robot_name
        self._planning_group = "manipulator"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(
            robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(
            ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(
            self._planning_group, robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher(
            self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        self.home = [0.13697517276931226, -2.3896570101413683, -0.5927255764061465, -
                     1.7288313443534982, 1.5700304686775333, 0.13642580642830016]

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo("Package Path: {}".format(self._file_path))

        self._logical_camera_topic = '/eyrc/vb/logical_camera_1'
        self._vacuum_gripper = '/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1'
        self._group.set_planning_time(99)

        param_config_iot = rospy.get_param('config_iot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']

        self.red = []
        self.yellow = []
        self.green = []

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> UR5_1 init done." + '\033[0m')

    def attach_box(self, i, j, timeout=4):
        '''
        Function for attaching box in RVIZ

        :param i: Row number of box
        :type i: int
        :param j: Column number of box
        :type j: int
        :param timeout: timeout, defaults to 4
        :type timeout: int, optional
        '''

        _box_name = str(i) + str(j)
        rospy.logwarn('Attaching Box in Planning Scene')

        grasping_group = self._group_names[0]
        touch_links = self._robot.get_link_names(group=grasping_group)

        self._scene.attach_box(self._eef_link, _box_name,
                               touch_links=touch_links)

        rospy.logwarn('Attach Successful')

    def detach_box(self, i, j, timeout=4):
        '''
        Function for detaching box from EE in RVIZ

        :param i: Row number of box
        :type i: int
        :param j: Column number of box
        :type j: int
        :param timeout: timeout, defaults to 4
        :type timeout: int, optional
        '''

        _box_name = str(i) + str(j)
        rospy.logwarn('Detaching Box from Planning Scene')

        self._scene.remove_attached_object(self._eef_link, name=_box_name)

        rospy.logwarn('Detach Successful')

    def remove_box(self, i, j, timeout=4):
        '''
        Function for removing box from EE in RVIZ

        :param i: Row number of box
        :type i: int
        :param j: Column number of box
        :type j: int
        :param timeout: timeout, defaults to 4
        :type timeout: int, optional
        '''

        _box_name = str(i) + str(j)
        rospy.logwarn('Removing Box from Planning Scene')

        self._scene.remove_world_object(_box_name)

        rospy.logwarn('Removal Successful')

    def clear_octomap(self):
        '''
        Octomap

        :return: clear _octomap service proxy
        :rtype: function
        '''

        clear_octomap_service_proxy = rospy.ServiceProxy(
            self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        '''
        Hard play saved trajectories from file

        :param arg_file_path: path to yaml file
        :type arg_file_path: str
        :param arg_file_name: file name
        :type arg_file_name: str
        :param arg_max_attempts: maximum number of attempts
        :type arg_max_attempts: int

        :return: plan
        :rtype: plan
        '''
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open, Loader=yaml.Loader)

        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        '''
        Hard play saved trajectories from file

        :param arg_file_path: path to yaml file
        :type arg_file_path: str
        :param arg_file_name: file name
        :type arg_file_name: str
        :param arg_max_attempts: maximum number of attempts
        :type arg_max_attempts: int

        :return: Success or Fail
        :rtype: boolean
        '''

        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(
                arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts))
            # # self.clear_octomap()

        return True

    def vacuum_gripper(self, state):
        '''
        Function to activate or deactivate vaccum gripper

        :param state:  state of gripper to be executed
        :type state: boolean
        '''
        rospy.logwarn('Vacuum Gripper')
        rospy.wait_for_service(self._vacuum_gripper)
        cmd = False

        try:
            rospy.logwarn('Vacuum Gripper')
            s1 = rospy.ServiceProxy(self._vacuum_gripper,
                                    vacuumGripper)

            if state == True:
                while True:
                    cmd = s1(state).result
                    if cmd == True:
                        break
                rospy.logwarn(
                    '**************Activate Vacuum Gripper UR5_1**************')
                rospy.logwarn(cmd)

            if state == False:
                rospy.loginfo(
                    '**************Deactivating Vacuum Gripper UR5_1**************')
                while True:
                    cmd = s1(state).result
                    if cmd == False:
                        break
                rospy.logwarn(cmd)
        except:
            pass

    def attach_drop(self, i, j):
        '''
        Function to attach box to arm and drop box on belt

        :param i: Row number of box
        :type i: int
        :param j: Column number of box
        :type j: int
        '''

        rospy.logwarn('Box Found Activating Gripper:UR5_1')
        self.vacuum_gripper(True)

        self.attach_box(i, j)
        print(str(i)+str(j)+'_to_home'+'.yaml')

        if (i, j) == (2, 0) or (i, j) == (3, 2):
            self.moveit_hard_play_planned_path_from_file(
                self._file_path, str(i)+str(j)+'_to_1'+'.yaml', 3)

            self.moveit_hard_play_planned_path_from_file(
                self._file_path, str(i)+str(j)+'_1_to_home'+'.yaml', 3)

        else:
            self.moveit_hard_play_planned_path_from_file(
                self._file_path, str(i)+str(j)+'_to_home'+'.yaml', 3)

        rospy.logwarn('Dropping Box on Belt:UR5_1')
        self.vacuum_gripper(False)

        self.detach_box(i, j)
        self.remove_box(i, j)

    def pkg_pos(self):
        '''
        Function to get the postions of different items on shelf
        '''

        self.param_models = rospy.get_param('models')
        print(self.param_models)

        for key in self.param_models:

            color = self.param_models[key]['color']
            if color == 'Red':
                self.red.append(key[-2:])
            elif color == 'Yellow':
                self.yellow.append(key[-2:])
            elif color == 'Green':
                self.green.append(key[-2:])
            else:
                continue

        self.red = sorted(self.red)
        self.yellow = sorted(self.yellow)
        self.green = sorted(self.green)

        rospy.logwarn('Red:{}'.format(self.red))
        rospy.logwarn('Yellow:{}'.format(self.yellow))
        rospy.logwarn('Green:{}'.format(self.green))

        # Destructor

    def __del__(self):
        '''
        Destory all parameters of UR5_1 Object
        '''
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class UR5_1 Deleted." + '\033[0m')


class Params:
    '''
    A class to handle order parameter
    '''

    def __init__(self):
        '''
        Constructs all parameters for Params object
        '''
        self.sort_order = ['Medicine', 'Food', 'Clothes']
        self.res = []
        self.param_orders = []

    def update_params_orders(self):
        '''
        Function to update /orders parameters
        '''

        self.param_orders = rospy.get_param('orders')
        rospy.loginfo(self.param_orders)

        self.res = [
            dict for i in self.sort_order for dict in self.param_orders if dict['item'] == i]

        rospy.loginfo('res:{}'.format(self.res))


def get_time_str():
    '''
    Function to get Current Time

    :return: Time String
    :rtype: str
    '''

    timestamp = int(time.time())
    value = datetime.datetime.fromtimestamp(timestamp)
    str_time = value.strftime('%Y-%m-%d %H:%M:%S')

    return str_time


def pub_mqtt(msg, goal_id):
    '''
    Sends a goal to Action Server

    :param msg: message to be pushed to spreadsheet
    :type msg: str
    :param goal_id: goal id
    :type goal_id: int
    '''

    action_client = IotRosBridgeActionClient()

    goal_handle = action_client.send_goal(
        "mqtt", "pub", action_client._config_mqtt_pub_topic, msg)

    action_client._goal_handles[str(goal_id)] = goal_handle

    rospy.loginfo('MQTT Goal Sent')


def main():
    '''
    Object Creation main execution of node

    :param argv: sys argv
    :type argv: list
    '''
    rospy.sleep(30)
    disp_order = []

    ur5 = UR5_1(sys.argv[1])
    params = Params()
    while True:
        try:
            ur5.pkg_pos()
            break
        except:
            pass

    ur5.moveit_hard_play_planned_path_from_file(
        ur5._file_path, 'zero_to_home.yaml', 3)

    while True:
        try:
            params.update_params_orders()
            break
        except:
            pass

    goal_id = 0

    while True:
        try:
            params.update_params_orders()

            disp = params.res[0]
            item = params.res[0]['item']
            rospy.logwarn(item)

            if item == 'Medicine':
                r, c = ur5.red[0][0], ur5.red[0][1]
                ur5.red.remove(r+c)
                rospy.logwarn(ur5.red)

            if item == 'Food':
                r, c = ur5.yellow[0][0], ur5.yellow[0][1]
                ur5.yellow.remove(r+c)

                rospy.logwarn(ur5.yellow)

            if item == 'Clothes':
                r, c = ur5.green[0][0], ur5.green[0][1]
                ur5.green.remove(r+c)

                rospy.logwarn(ur5.green)

            rospy.logwarn('UR5_1:' + 'home_to_' + r + c)

            ur5.moveit_hard_play_planned_path_from_file(
                ur5._file_path, 'home_to_'+r+c+'.yaml', 3)

            ur5.attach_drop(int(r), int(c))

            params.update_params_orders()
            params.res.remove(disp)
            rospy.logwarn(params.res)

            msg = '{} {} {} {} {} {} {}'.format(
                disp['order_id'], disp['city'], disp['item'], disp['qty'], 'Yes', get_time_str(), 'OrdersDispatched')

            rospy.logwarn('Sending Goal: Dispatch')
            pub_mqtt(msg, goal_id)
            goal_id += 1

            rospy.logwarn(
                '*****************DISPATCHED***************\n{}'.format(disp))
            disp_order.append(disp)
            rospy.set_param('disp_order', disp_order)
            print('Dispatch Order UR1:{}'.format(disp))

            if len(params.res) == 0:
                rospy.delete_param('orders')
            else:
                rospy.set_param('orders', params.res)
                params.update_params_orders()
        except:
            pass

    del ur5


if __name__ == '__main__':
    '''
    Main Function
    '''
    try:
        main()
    except rospy.ROSInterruptException:
        pass

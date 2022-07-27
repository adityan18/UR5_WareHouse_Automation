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
import datetime
import time


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

# Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotAction
# Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotGoal
# Message Class that is used for Result Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult
# Message Class for MQTT Subscription Messages
from pkg_ros_iot_bridge.msg import msgMqttSub

from RepMeth import IotRosBridgeActionClient


from std_srvs.srv import Empty


class UR5_2:
    '''
    A class for controlling UR5_2 Arm
    '''

    def __init__(self, arg_robot_name):
        '''
        Constructs all parameters for UR5_2 Object

        :param arg_robot_name: Robot Arm Name
        :type arg_robot_name: str
        '''

        rospy.init_node('node_ur5_2', anonymous=True)

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

        # Attribute to store computed trajectory by the planner
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        self._conveyor_belt_power = '/eyrc/vb/conveyor/set_power'
        self._logical_camera_topic = '/eyrc/vb/logical_camera_2'
        self._vacuum_gripper = '/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2'

        self.box_length = 0.15               # Length of the Package
        self.vacuum_gripper_width = 0.115    # Vacuum Gripper Width
        self.delta = self.vacuum_gripper_width + (self.box_length/2)  # 0.19

        self.pose_x, self.pose_y, self.pose_z = 0, 0, 0

        self._power = 100
        self._stop = 0

        self._belt_state = 1

        self._box_name = ''
        self._model_name = ''

        self.pick = False
        self.pkg_drop = False
        self.entry = False
        self.drop = True

        self.disp_order = 0

        self.reach_home = True
        self.pkg_list = ['packagen00', 'packagen01', 'packagen02', 'packagen10', 'packagen11', 'packagen12',
                         'packagen20', 'packagen21', 'packagen22', 'packagen30', 'packagen31', 'packagen32']

        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo("Package Path: {}".format(self._file_path))

        self.param_models = rospy.get_param('models')

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

    def clear_octomap(self):
        '''
        Octomap

        :return: clear _octomap service proxy
        :rtype: function
        '''
        clear_octomap_service_proxy = rospy.ServiceProxy(
            self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        '''
        Function to move arm along a cartestian path

        :param trans_x: tranlation x
        :type trans_x: float
        :param trans_y: translation y
        :type trans_y: float
        :param trans_z: translation z
        :type trans_z: float
        :return: boolean
        :rtype: path computed state
        '''

        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)
        wpose.position.y = waypoints[0].position.y + (trans_y)
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5

        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))

        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.05,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        try:
            self._group.execute(plan)
            return True
        except:
            return False

    def set_joint_angles(self, arg_list_joint_angles):
        '''
        Function to move arm by hard setting joint angles

        :param arg_list_joint_angles: joint angles
        :type arg_list_joint_angles: list
        '''

        list_joint_values = self._group.get_current_joint_values()

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()

        pose_values = self._group.get_current_pose().pose

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        '''
        Function to move arm by hard setting joint angles

        :param arg_list_joint_angles: joint angles
        :type arg_list_joint_angles: list
        :param arg_max_attempts: maximum number of attempts
        :type arg_max_attempts: int
        '''

        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))
            # self.clear_octomap()

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
        rospy.wait_for_service(self._vacuum_gripper)
        cmd = False

        try:
            s1 = rospy.ServiceProxy(self._vacuum_gripper,
                                    vacuumGripper)

            if state == True:
                while True:
                    cmd = s1(state).result
                    if cmd == True:
                        break
                rospy.logwarn(
                    '**************Activate Vacuum Gripper  UR5_2**************')
                rospy.logwarn(cmd)
                # self.attach_box()

            if state == False:
                rospy.loginfo(
                    '**************Deactivating Vacuum Gripper  UR5_2**************')
                while True:
                    cmd = s1(state).result
                    if cmd == False:
                        break
                rospy.logwarn(cmd)
                # self.detach_box()
        except:
            pass

    def package_drop(self, color):
        '''
        Function that manages the arm to go to bin position and drop package

        :param color: color of box
        :type color: str

        :return: Completion
        :rtype: boolean
        '''

        file_name_2 = color + '_to_home.yaml'

        # Function that manges the arm to go to bin position and drop package
        rospy.wait_for_service(self._conveyor_belt_power)
        try:
            s2 = rospy.ServiceProxy(
                self._conveyor_belt_power, conveyorBeltPowerMsg)
        except rospy.ServiceException:
            rospy.logerr('Service Call Failed')

        rospy.logwarn('Box Found Activating Gripper  UR5_2')
        self.vacuum_gripper(True)

        self.pick = True
        rospy.logwarn('Adjusting Position  UR5_2')
        if(self.pose_z < -0.05):
            self.ee_cartesian_translation(0, -self.pose_z, 0.1)
        else:
            self.ee_cartesian_translation(0, 0, 0.1)
        # rospy.sleep(0.05)

        rospy.logwarn('**************Restarting Belt  UR5_2**************')
        rospy.loginfo(s2(self._power).result)

        self.reach_home = False
        self._belt_state = 1

        rospy.logwarn('Home to ' + color)
        # self.moveit_hard_play_planned_path_from_file(
        #     self._file_path, file_name_1, 5)
        if(color == 'red'):
            angles = [-1.2761559001004503, -1.6459839018638345, -1.5471282885304385, -
                      1.5186658107818394, 1.57172425479252, -1.2767270241274087]
        if(color == 'green'):
            angles = [1.6730040694469261, -1.7141379427586179, -1.471585021668382, -
                      1.5270529121882133, 1.5695388712934983, 1.6724301125079686]

        if(color == 'yellow'):
            angles = [math.radians(-169.344815413),
                      math.radians(-127.088431959),
                      math.radians(-49.7360204238),
                      math.radians(-93.1171577966),
                      math.radians(90.0271906165),
                      math.radians(-169.289929133)]

        self.hard_set_joint_angles(angles, 2)

        self.pick = False

        rospy.loginfo('Bin Reached Deactivating Gripper UR5_2')
        self.vacuum_gripper(False)

        self.ship_order()

        rospy.logwarn(color+' to Home')
        self.moveit_hard_play_planned_path_from_file(
            self._file_path, file_name_2, 3)

        self.reach_home = True

        rospy.logwarn(self.entry)
        rospy.logwarn('pkg Entry: ' + str(self.entry))

        rospy.logwarn('Reached Home:' + str(self.reach_home) + ' UR5_2')

        return True

    def ship_order(self):
        '''
        Generate message to send goal to push to sheet
        '''

        ship_order = rospy.get_param('disp_order')
        ship = ship_order[self.disp_order]
        print('Dispatch Order UR2:{}'.format(ship))
        rospy.logwarn(
            '*****************SHIPPED***************\n{}'.format(ship))

        if(ship['item'] == 'Clothes'):
            x = 5
        elif (ship['item'] == 'Food'):
            x = 3
        elif (ship['item'] == 'Medicine'):
            x = 1

        time, etd = get_time_str(x)
        msg = '{} {} {} {} {} {} {} {}'.format(ship['order_id'], ship['city'],
                                               ship['item'], ship['qty'], 'Yes', time, etd, 'OrdersShipped')

        pub_mqtt(msg, self.disp_order)

        self.disp_order += 1

    def conveyor_manager(self):
        '''
        Function that is used to stop the conveyor belt when the package reaches the desired postition

        '''

        rospy.wait_for_service(self._conveyor_belt_power)

        if(self.pick == False):

            try:
                s2 = rospy.ServiceProxy(
                    self._conveyor_belt_power, conveyorBeltPowerMsg)
            except rospy.ServiceException:
                rospy.logerr('Service Call Failed')

            rospy.logwarn('**************Stoping Belt  UR5_2**************')
            rospy.loginfo(s2(self._stop).result)
            self._belt_state = 0

    def arm_manager(self):
        '''
        Function that aligns the end effector with the package

        '''
        try:
            if self.reach_home == True:
                self.entry = True

                rospy.logwarn(self._model_name)

                color = self.param_models[self._model_name]['color']
                rospy.logwarn(color)
                rospy.logwarn('Entered ' + color + ' UR5_2')

                # self.ee_cartesian_translation(
                #     self.pose_z, self.pose_y, 0)

                self.package_drop(color.lower())
                self.entry = False
                # rospy.sleep(0.5)
                rospy.logwarn('xxxxxxxxBreakingxxxxxxxxxxx')

                self.pkg_list.remove(self._model_name)
                self.drop = True

        except:
            pass

    def logical_cam_sub_callback(self, msg):
        '''
        Logical Camera Subscription Callback Function

        :param msg: message from topic
        :type msg: dict
        '''
        no_of_models = len(msg.models)

        for i in range(0, no_of_models):
            self._model_name = msg.models[i].type
            if(self._model_name in self.pkg_list):
                pose = msg.models[i].pose
                self.pose_y = pose.position.y
                self.pose_x = pose.position.x
                self.pose_z = pose.position.z

                if(self.pose_y < 0 and self.pose_x > 1 and self.drop == True):
                    self.drop = False
                    rospy.logwarn('Entry:' + str(self.entry))
                    rospy.logwarn(self._model_name)
                    self.arm_manager()

    def __del__(self):
        '''
        Destory all parameters of UR_1 Object
        '''
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class UR_2 Deleted." + '\033[0m')


def get_time_str(day):
    '''
    Function to get Current Time

    :param day: estimated time of delivery
    :type day: int

    :return: Time String
    :rtype: str
    '''
    timestamp = int(time.time())
    value = datetime.datetime.fromtimestamp(timestamp)
    str_time = value.strftime('%Y-%m-%d %H:%M:%S')

    etd = value + datetime.timedelta(days=day)
    str_time_1 = value.strftime('%Y-%m-%d %H:%M:%S')

    str_time_2 = etd.strftime('%Y-%m-%d %H:%M:%S')

    return str_time_1, str_time_2


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

    rospy.logwarn('Start Ur5_2')

    ur5 = UR5_2(sys.argv[1])

    rospy.wait_for_service(ur5._conveyor_belt_power)
    try:
        s2 = rospy.ServiceProxy(
            ur5._conveyor_belt_power, conveyorBeltPowerMsg)
    except rospy.ServiceException:
        rospy.logerr('Service Call Failed')
    rospy.logwarn('**************Starting Belt Ur5_2**************')

    rospy.loginfo(s2(ur5._power).result)

    rospy.logwarn('Zero to Home UR5_2')

    ur5.moveit_hard_play_planned_path_from_file(
        ur5._file_path, 'zero_to_home_2.yaml', 3)

    # Starting Conveyor Belt

    rospy.Subscriber(ur5._logical_camera_topic,
                     LogicalCameraImage, ur5.logical_cam_sub_callback, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

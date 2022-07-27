#!/usr/bin/env python

# ROS Node - Action Server - IoT ROS Bridge

import rospy
import actionlib
import threading

# Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotAction
# Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotGoal
# Message Class that is used for Result Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult
# Message Class that is used for Feedback Messages
from pkg_ros_iot_bridge.msg import msgRosIotFeedback

# Message Class for MQTT Subscription Messages
from pkg_ros_iot_bridge.msg import msgMqttSub

# Custom Python Module to perfrom MQTT Tasks
from pyiot import iot


class IotRosBridgeActionServer:
    """
    A class for Action Server
    """

    # Constructor
    def __init__(self):
        '''
        Constructs all necessary parameters for IotRosBridgeActionServer object
        '''

        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_ros_iot',
                                          msgRosIotAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)

        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_iot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        print(param_config_iot)

        # Initialize ROS Topic Publication
        # Incoming message from MQTT Subscription will be published on a ROS Topic (/ros_iot_bridge/mqtt/sub).
        # ROS Nodes can subscribe to this ROS Topic (/ros_iot_bridge/mqtt/sub) to get messages from MQTT Subscription.
        self._handle_ros_pub = rospy.Publisher(
            self._config_mqtt_sub_cb_ros_topic, msgMqttSub, queue_size=10)

        # Subscribe to MQTT Topic (eyrc/jgHsFpUv/iot_to_ros) which is defined in 'config_pyiot.yaml'.
        # self.mqtt_sub_callback() function will be called when there is a message from MQTT Subscription.
        ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                              self._config_mqtt_server_url,
                                              self._config_mqtt_server_port,
                                              self._config_mqtt_sub_topic,
                                              self._config_mqtt_qos)
        if(ret == 0):
            rospy.loginfo('MQTT Subscribe Thread Started')
        else:
            rospy.logerr('Failed to start MQTT Subscribe Thread')

        # Start the Action Server
        self._as.start()

        rospy.loginfo('Started ROS-IoT Bridge Action Server.')

        # self.spreadsheet_id_1 = rospy.get_param(
        #     'config_iot')['google_apps']['spread_sheet_id']

        self.pos_R = ''
        self.pos_C = ''
        self.code = ''
        self.item = ''
        self.cost = ''
        self.priority = ''
        self.quan = 1
        self.SKU = ''
        self.sto_no_r = ''
        self.sto_no_c = ''

        self.qty = 1
        self.order_day = ''
        self.order_time = ''
        self.city = ''
        self.lat = ''
        self.lon = ''
        self.lat_dir = ''
        self.lon_dir = ''
        self.order_id = ''
        self.etd_day = ''
        self.etd_time = ''

        self.goal_id = 0

        self._team_id = 'VB#0393'
        self._unique_id = 'jgHsFpUv'

        self.id = ''
        self.disp_stat = ''
        self.disp_day = ''
        self.disp_time = ''
        self.disp_qty = ''

    def mqtt_sub_callback(self, client, userdata, message):
        '''
        Callback Function for MQTT subscription

        :param client: client
        :type client: str
        :param userdata: userdata
        :type userdata: str
        :param flags: flags
        :type flags: int
        '''
        payload = str(message.payload.decode('utf-8'))

        print('[MQTT SUB CB] Message: ', payload)
        print('[MQTT SUB CB] Topic: ', message.topic)

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload

        self._handle_ros_pub.publish(msg_mqtt_sub)

    def on_goal(self, goal_handle):
        '''
        This function will be called when Action Server receives a Goal

        :param goal_handle: goal handle
        :type goal_handle: int

        '''
        goal = goal_handle.get_goal()

        rospy.loginfo('Received new goal from Client')
        rospy.loginfo(goal)

        # Validate incoming goal parameters
        if(goal.protocol == 'mqtt'):

            if((goal.mode == 'pub') or (goal.mode == 'sub')):
                goal_handle.set_accepted()

                # Start a new thread to process new goal from the client (For Asynchronous Processing of Goals)
                # 'self.process_goal' - is the function pointer which points to a function that will process incoming Goals
                thread = threading.Thread(name='worker',
                                          target=self.process_goal,
                                          args=(goal_handle,))
                thread.start()

            else:
                goal_handle.set_rejected()
                return

        else:
            goal_handle.set_rejected()
            return

    def default(self, item):
        '''
        Returns default parameters for item

        :param item: item
        :type item: str
        :return: priority, cost, etd
        :rtype: tuple
        '''
        if self.item == 'Clothes':
            priority = 'LP'
            cost = 1000
            etd = '5'
        if self.item == 'Food':
            priority = 'MP'
            cost = 2000
            etd = '3'
        if self.item == 'Medicine':
            priority = 'HP'
            cost = 3000
            etd = '1'

        return (priority, cost, etd)

    def process_goal(self, goal_handle):
        '''
        This function is called is a separate thread to process Goal.

        :param goal_handle: goal handle
        :type goal_handle: int
        '''
        flag_success = False
        params = {}
        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo('Processing goal : ' + str(goal_id.id))

        goal = goal_handle.get_goal()

        # Goal Processing
        if goal.protocol == 'mqtt':
            rospy.logwarn('MQTT')

            if goal.mode == 'pub':
                rospy.logwarn('MQTT PUB Goal ID: ' + str(goal_id.id))

                sheet = goal.message.split()
                print(sheet)

                if sheet[-1] == 'Inventory':

                    self.SKU, self.item, self.sto_no_r, self.sto_no_c, self.quan, self.id = goal.message.split()

                    self.priority, self.cost, self.etd = self.default(
                        self.item)

                    params = {
                        'Team Id': self._team_id,
                        'Unique Id': self._unique_id,
                        'SKU': self.SKU,
                        'Item': self.item,
                        'Priority': self.priority,
                        'Storage Number': self.sto_no_r + ' ' + self.sto_no_c,
                        'Cost': self.cost,
                        'Quantity': self.quan,
                        'id': 'Inventory'
                    }

                elif sheet[-1] == 'IncomingOrders':

                    self.order_id, self.order_day, self.order_time, self.item, self.qty, self.city, self.lat, self.lat_dir, self.lon, self.lon_dir, self.id = goal.message.split()

                    self.priority, self.cost, self.etd = self.default(
                        self.item)

                    params = {
                        'Team Id': self._team_id,
                        'Unique Id': self._unique_id,
                        'Order ID': self.order_id,
                        'City': self.city,
                        'Item': self.item,
                        'Priority': self.priority,
                        'Order Quantity': self.qty,
                        'Cost': self.cost,
                        'Order Date and Time': self.order_day + ' ' + self.order_time,
                        'Longitude': self.lon+' '+self.lon_dir,
                        'Latitude': self.lat + ' ' + self.lat_dir,
                        'id': 'IncomingOrders'
                    }

                elif(sheet[-1] == 'OrdersDispatched'):

                    self.order_id, self.city, self.item, self.disp_qty, self.disp_stat, self.disp_day, self.disp_time, self.id = goal.message.split()
                    self.priority, self.cost, self.etd = self.default(
                        self.item)

                    params = {
                        'Team Id': self._team_id,
                        'Unique Id': self._unique_id,
                        'Order ID': self.order_id,
                        'City': self.city,
                        'Item': self.item,
                        'Priority': self.priority,
                        'Dispatch Quantity': self.disp_qty,
                        'Cost': self.cost,
                        'Dispatch Status': self.disp_stat,
                        'Dispatch Date and Time': self.disp_day + ' ' + self.disp_time,
                        'id': 'OrdersDispatched'
                    }

                elif(sheet[-1] == 'OrdersShipped'):

                    self.order_id, self.city, self.item, self.ship_qty, self.ship_stat, self.ship_day, self.ship_time, self.etd_day, self.etd_time, self.id = goal.message.split()
                    self.priority, self.cost, self.etd = self.default(
                        self.item)

                    params = {
                        'Team Id': self._team_id,
                        'Unique Id': self._unique_id,
                        'Order ID': self.order_id,
                        'City': self.city,
                        'Item': self.item,
                        'Priority': self.priority,
                        'Shipped Quantity': self.ship_qty,
                        'Cost': self.cost,
                        'Shipped Status': self.ship_stat,
                        'Shipped Date and Time': self.ship_day + ' ' + self.ship_time,
                        'Estimated Time of Delivery': self.etd_day + ' ' + self.etd_time,
                        'id': 'OrdersShipped'
                    }

                # Push to spreadsheet
                iot.push_ss(params)

            elif goal.mode == 'sub':
                rospy.logwarn('MQTT SUB Goal ID: ' + str(goal_id.id))
                rospy.logwarn(goal.topic)
                ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                                      self._config_mqtt_server_url,
                                                      self._config_mqtt_server_port,
                                                      goal.topic,
                                                      self._config_mqtt_qos)
                if(ret == 0):
                    rospy.loginfo('MQTT Subscribe Thread Started')
                    result.flag_success = True
                else:
                    rospy.logerr('Failed to start MQTT Subscribe Thread')
                    result.flag_success = False

        if (result.flag_success == True):
            rospy.loginfo('Succeeded')
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo('Goal Failed. Aborting.')
            goal_handle.set_aborted(result)

        rospy.loginfo('Goal ID: ' + str(goal_id.id) + ' Goal Processing Done.')

    def on_cancel(self, goal_handle):
        '''
        This function will be called when Goal Cancel request is send to the Action Server

        :param goal_handle: goal handle
        :type goal_handle: int

        '''
        rospy.loginfo('Received cancel request.')
        goal_id = goal_handle.get_goal_id()


def main():
    '''
    Function that inits the node and created IotRosBridgeActionServer object
    '''
    rospy.init_node('node_action_server_ros_iot_bridge', anonymous=True)

    action_server = IotRosBridgeActionServer()

    rospy.spin()


if __name__ == '__main__':
    """
    Main function for node
    """
    try:
        main()
    except rospy.ROSInterruptException:
        pass

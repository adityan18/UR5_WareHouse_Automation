#!/usr/bin/env python

import paho.mqtt.client as mqtt
import time
import rospy


import actionlib

from RepMeth import IotRosBridgeActionClient





goal_id = 0


def on_connect(client, userdata, flags, rc):
    '''
    On Connect Function Callback for MQTT Subscription

    :param client: client
    :type client: str
    :param userdata: userdata
    :type userdata: str
    :param flags: flags
    :type flags: int
    :param rc: result code
    :type rc: int
    '''

    print("[INFO] Connected With Result Code: " + str(rc))


def on_message(client, userdata, message):
    '''
    On message Function Callback for MQTT Subscription

    :param client: client
    :type client: str
    :param userdata: userdata
    :type userdata: str
    :param flags: flags
    :type flags: int
    '''

    global goal_id

    try:
        ls = rospy.get_param('orders')
    except:
        ls = []

    payload = eval(message.payload)
    # print(type(x))
    rospy.loginfo(payload)

    msg = '{} {} {} {} {} {} {} {}'.format(
        payload['order_id'], payload['order_time'], payload['item'], payload['qty'], payload['city'], payload['lat'], payload['lon'], 'IncomingOrders')
    pub_mqtt(msg)

    ls.append(payload)

    rospy.set_param('orders', ls)

    rospy.logwarn(ls)
    goal_id += 1


def pub_mqtt(msg):
    '''
    Function to send goals to Action Server

    :param msg: message
    :type msg: str
    '''


    action_client = IotRosBridgeActionClient()
    goal_handle = action_client.send_goal(
        "mqtt", "pub", action_client._config_mqtt_pub_topic, msg)

    action_client._goal_handles[str(goal_id)] = goal_handle

    rospy.loginfo('MQTT Goal Sent')

def main():
    '''
    Object and Node Creation
    '''

    broker_url = "broker.mqttdashboard.com"
    broker_port = 1883

    ls = []

    # ROS Node
    rospy.init_node('node_sub_mqtt_client', anonymous=True)

    sub_client = mqtt.Client()
    sub_client.on_connect = on_connect
    sub_client.on_message = on_message
    sub_client.connect(broker_url, broker_port)

    sub_client.subscribe("/eyrc/vb/jgHsFpUv/orders", qos=0)
    # sub_client.subscribe("/eyrc/vb/time", qos=0)

    sub_client.loop_forever()

    print("Out of Loop. Exiting..")


if __name__ == '__main__':
    '''
    Main Function
    '''
    try:
        main()
    except rospy.ROSInterruptException:
        pass
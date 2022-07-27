#!/usr/bin/env python
from multiprocessing.dummy import Pool
import time
import requests
import rospy

import sys
import paho.mqtt.client as mqtt
import time


class print_colour:
    """
    Different Colors to be shown on terminal
    """

    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


# -----------------  MQTT SUB -------------------
def iot_func_callback_sub(client, userdata, message):
    '''
    Callback Function for MQTT Subscription

    :param client: client
    :type client: str
    :param userdata: userdata
    :type userdata: str
    :param flags: flags
    :type flags: int
    '''
    print("message received ", str(message.payload.decode("utf-8")))
    print("message topic=", message.topic)
    print("message qos=", message.qos)
    print("message retain flag=", message.retain)


def mqtt_subscribe_thread_start(arg_callback_func, arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_qos):
    '''
    subscriber thread for MQTT Topic

    :param arg_callback_func: callback function
    :type arg_callback_func: function
    :param arg_broker_url: broker url
    :type arg_broker_url: str
    :param arg_broker_port: broker port
    :type arg_broker_port: int
    :param arg_mqtt_topic: mqtt topic
    :type arg_mqtt_topic: str
    :param arg_mqtt_qos: mqtt qos
    :type arg_mqtt_qos: int
    :return: 0 or -1
    :rtype: int
    '''
    try:
        mqtt_client = mqtt.Client()
        mqtt_client.on_message = arg_callback_func
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.subscribe(arg_mqtt_topic, arg_mqtt_qos)
        time.sleep(1)  # wait
        # mqtt_client.loop_forever() # starts a blocking infinite loop
        mqtt_client.loop_start()    # starts a new thread
        return 0
    except:
        return -1


# -----------------  MQTT PUB -------------------
def mqtt_publish(arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos):
    """
    MQTT Publisher

    :param arg_callback_message: message
    :type arg_callback_func: str
    :param arg_broker_url: broker url
    :type arg_broker_url: str
    :param arg_broker_port: broker port
    :type arg_broker_port: int
    :param arg_mqtt_topic: mqtt topic
    :type arg_mqtt_topic: str
    :param arg_mqtt_qos: mqtt qos
    :type arg_mqtt_qos: int
    :return: 0 or -1
    :rtype: int
    """
    try:
        mqtt_client = mqtt.Client("mqtt_pub")
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.loop_start()

        print("Publishing message to topic", arg_mqtt_topic)
        mqtt_client.publish(arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos)
        time.sleep(0.1)  # wait

        mqtt_client.loop_stop()  # stop the loop
        return 0
    except:
        return -1


def push_ss(parameters):
    """
    Used to push data to Google spreadsheet

   :param parameters: parameters for get request
    :type arg_mqtt_qos: dict
    """
    spreadsheet_id_1 = rospy.get_param(
        'config_iot')['google_apps']['spread_sheet_id']
    spreadsheet_id_2 = 'AKfycbw5xylppoda-8HPjt2Tzq4ShU_Xef-Ik-hEtBPcPk0gdGw8095j4RZ7'

    print(parameters)

    # response_1 = requests.get("https://script.google.com/macros/s/"+spreadsheet_id_1+"/exec?",
    #                           params=parameters)
    # rospy.logwarn("Response Our Sheet {}: {}".format(
    #     parameters['id'], response_1))

    # response_2 = requests.get("https://script.google.com/macros/s/"+spreadsheet_id_2+"/exec?",
    #                           params=parameters)
    # rospy.logwarn(
    #     "Response E-Yantra Sheet {}: {}".format(parameters['id'], response_2))

    rospy.logwarn('Spreadsheet Push Successfull')

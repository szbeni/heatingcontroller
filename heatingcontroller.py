#!/usr/bin/python3

import paho.mqtt.client as mqtt
from time import sleep
from heatingcontroller_settings import HeatingControllerSettings as Settings

class HeatingController():
    def __init__(self):
        self.temperature = 0
        self.state = none
        self.mode = "manual"
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.username_pw_set(Settings.mqtt_server['username'], Settings.mqtt_server['password'])


    # The callback for when the client receives a CONNACK response from the server.
    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))

        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        client.subscribe(Settings.main_topic)

    # The callback for when a PUBLISH message is received from the server.
    def on_message(self, client, userdata, msg):
        #print(msg.topic+" "+str(msg.payload))
        if (msg.topic == Settings.temperature_topic):
            self.temperature = float(msg.payload)
            print("New temperature: ", self.temperature)

        if (msg.topic == Settings.input_topic):


    def start(self):
        self.client.connect(Settings.mqtt_server['host'], Settings.mqtt_server['port'])
        self.client.loop_start()
        while True:
            sleep(1)
        self.client.loop_stop()


if __name__ == "__main__":
    heatingController = HeatingController()
    heatingController.start()
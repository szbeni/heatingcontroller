#!/usr/bin/python3

import re
import paho.mqtt.client as mqtt
from time import sleep
from heatingcontroller_settings import HeatingControllerSettings as Settings
from heatingcontroller_scheduler import HeatingControllerScheduler

from influxdb import InfluxDBClient
from typing import NamedTuple


class SensorData(NamedTuple):
    location: str
    measurement: str
    value: float

    def getAsJSON(self):
        return [
            {
                'measurement': self.measurement,
                'tags': {
                    'location': self.location
                },
                'fields': {
                    'value': self.value
                }
            }
        ]


class HeatingController():
    def __init__(self):

        # InfluxDB client
        self.influxdb_client = InfluxDBClient(  Settings.influxdb['host'], Settings.influxdb['port'], Settings.influxdb['username'], Settings.influxdb['password'], Settings.influxdb['database'])

        # MQTT client
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.username_pw_set(Settings.mqtt_server['username'], Settings.mqtt_server['password'])

        # Control states
        self.INPUT_TO_STATE = ['00000', '10000', '01000','00100','00010','00001']
        self.STATES = ['Off', 'Gas Auto Fan', 'Gas Slow Fan', 'Fan', 'Elec Auto Fan', 'Elec Slow Fan']
        self.STATE_GAS_AUTO = 'Gas Auto Fan'
        self.STATE_ELEC_AUTO = 'Elec Auto Fan'
        self.STATE_ERROR = 'Error'
        self.STATE_OFF = 'Off'
        self.STATE_ON_LOW = 'Elec Slow Fan'
        self.STATE_ON_HIGH = 'Elec Auto Fan'
        self.STATE_ON = self.STATE_ON_HIGH
        self.state = self.STATE_OFF
        self.desired_state = self.STATE_OFF

        # Temperature control paramters
        self.hysteresis = 0.3
        self.set_temperature = 19.8
        self.temperature = 20

        # Mode, manual or auto
        self.MODE_AUTO = 'auto'
        self.MODE_MANUAL = 'manual'
        self.MODES = [self.MODE_AUTO, self.MODE_MANUAL]
        self.mode = self.MODE_MANUAL

        # Intensity low or high
        self.INTENSITY_LOW = 'low'
        self.INTENSITY_HIGH= 'high'
        self.INTENSITY = [self.INTENSITY_LOW, self.INTENSITY_HIGH]
        self.intensity = self.INTENSITY_LOW

        # Scheduler
        self._scheduler = HeatingControllerScheduler(self.schedulerEvent)
        self._scheduler.start()

    def publishControlStatus(self, controlName, value):
        if hasattr(self,'client'):
            self.client.publish(Settings.control_topic.replace("#","status/" + str(controlName)), value, retain=True)

    @property
    def set_temperature(self):
        return self._set_temperature

    @set_temperature.setter
    def set_temperature(self, x):
        if x > 30:
            x = 30
        elif x<5:
            x = 5
        self._set_temperature = x
        self.publishControlStatus('set_temperature', self.set_temperature)

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, mode):
        if mode in self.MODES:
            self._mode = mode
            self.publishControlStatus('mode', self.mode)

    @property
    def intensity(self):
        return self._intensity

    @intensity.setter
    def intensity(self, v):
        if v in self.INTENSITY:
            self._intensity = v
            if self.intensity == self.INTENSITY_LOW:
                self.STATE_ON = self.STATE_ON_LOW
            else:
                self.STATE_ON = self.STATE_ON_HIGH
            self.publishControlStatus('intensity', self.intensity)

    @property
    def scheduler(self):
        if self._scheduler.running:
            return 'ON'
        else:
            return 'OFF'

    @scheduler.setter
    def scheduler(self, v):
        if v == True:
            self._scheduler.start()
        else:
            self._scheduler.stop()

        self.publishControlStatus('scheduler', self.scheduler)

    def schedulerEvent(self, temp=None,intensity=None,mode=None):
        if temp:
            self.set_temperature = temp
        if intensity:
            self.intensity = intensity
        if mode:
            self.mode = mode
        self.publishControlStatus('scheduler_next', self._scheduler.next_run())

    # The callback for when the client receives a CONNACK response from the server.
    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))

        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        client.subscribe(Settings.main_topic)

    # These messages coming from the ESP8266
    def process_sensor_data(self, msg):
        match = re.match(Settings.sensordata_regex, msg.topic)
        if match:
            measurement = match.group(1)
            data = SensorData('caravan', measurement, float(msg.payload))
            try:
                self.influxdb_client.write_points(data.getAsJSON())
            except:
                print("Error writing data to influxdb")

            if (measurement == 'temperature'):
                self.temperature = float(msg.payload)
                #print("Current: {0},  Set: {1}".format(self.temperature, self.set_temperature))
                #print("New temperature: ", self.temperature)

    # These messages coming from the ESP8266
    def process_input_data(self, msg):
        if (msg.topic == Settings.input_topic):
            inp = msg.payload.decode('utf-8')
            if inp in self.INPUT_TO_STATE:
                self.state = self.STATES[self.INPUT_TO_STATE.index(inp)]
            else:
                self.state = self.STATE_ERROR


    # These messages coming from the UI
    def process_control_message(self, msg):
        match = re.match(Settings.control_regex, msg.topic)
        if match:
            command = match.group(1)

            if command == 'mode':
                self.mode = msg.payload.decode('utf-8')

            elif command == 'intensity':
                self.intensity = msg.payload.decode('utf-8')

            elif command == 'set_temperature':
                self.set_temperature = float(msg.payload)

            elif command == 'scheduler':
                self.scheduler = True if msg.payload.decode('utf-8') == 'ON' else False



    def get_state_distance(self, state1, state2):
        idx1 = self.STATES.index(state1)
        idx2 = self.STATES.index(state2)
        distance = idx2 - idx1
        if distance < 0:
            distance = distance + len(self.STATES)-1
        
        return distance
    def press_off(self, delay = 0):
        self.client.publish(Settings.command_topic,'1')
        sleep(delay)

    def press_on(self, times = 1, delay = 1):
        for i in range(0,times):
            self.client.publish(Settings.command_topic,'0')
            print("Press ON {0}".format(i+1))
            sleep(delay)

    def switch_to_state(self, state):
        if self.state != state:
            if state == self.STATE_OFF:
                self.press_off()
            else:
                distance = self.get_state_distance(self.state, state)
                self.press_on(distance)

        
    # The callback for when a PUBLISH message is received from the server.
    def on_message(self, client, userdata, msg):
        #print(msg.topic+" "+str(msg.payload))
        self.process_sensor_data(msg)
        self.process_input_data(msg)
        self.process_control_message(msg)

    def start(self):
        self.client.connect(Settings.mqtt_server['host'], Settings.mqtt_server['port'])
        self.client.loop_start()
        sleep(5)
        while True:
            self._scheduler.tick()
            self.publishControlStatus('scheduler_next', self._scheduler.next_run())
            sleep(10)

            # Change to auto mode if state is set to Elec_AUTO
            if self.mode == self.MODE_MANUAL:
                if self.state == self.STATE_ELEC_AUTO:
                    self.mode = self.MODE_AUTO

            # Change to manual mode if state is set to Gas_AUTO
            if self.mode == self.MODE_AUTO:
                if self.state == self.STATE_GAS_AUTO:
                    self.mode = self.MODE_MANUAL

            if self.mode == self.MODE_AUTO:
                if self.temperature < (self.set_temperature - self.hysteresis):
                    self.switch_to_state(self.STATE_ON)
                elif self.temperature > (self.set_temperature + self.hysteresis):
                    self.switch_to_state(self.STATE_OFF)

        self.client.loop_stop()
        
if __name__ == "__main__":
    heatingController = HeatingController()
    heatingController.start()
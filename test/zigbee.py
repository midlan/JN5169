import pytest
import time
import paho.mqtt.client as mqtt


class ZigbeeNetwork():
    def __init__(self, options):
        self.topic = options.getini('mqtt_topic')
        self.message_received = None

        self.client = mqtt.Client()
        self.client.on_message = self.on_message_received
        self.client.connect(options.getini('mqtt_server'), int(options.getini('mqtt_port')))


    def on_message_received(self, client, userdata, msg):
        print(f"Received message on topic {msg.topic}: {msg.payload.decode()}")
        self.message_received = msg.payload


    def get_topic(self, subtopic):
        if subtopic:
            return self.topic + '/' + subtopic
        
        return self.topic


    def subscribe(self, subtopic = ""):
        self.client.subscribe(self.get_topic(subtopic))
        self.message_received = None


    def publish(self, subtopic, message):
        topic = self.get_topic(subtopic)
        print(f"Sending message to '{topic}': {message}")
        self.client.publish(topic, message)


    def wait_msg(self, timeout=5):
        start_time = time.time()

        while self.message_received is None:
            self.client.loop(timeout=1)

            elapsed_time = time.time() - start_time
            if elapsed_time > timeout:
                raise TimeoutError
        
        payload = self.message_received.decode()
        self.message_received = None
        return payload
            
            
    def disconnect(self):
        self.client.disconnect()


@pytest.fixture(scope="session")
def zigbee(pytestconfig):
    net = ZigbeeNetwork(pytestconfig)
    yield net
    net.disconnect()
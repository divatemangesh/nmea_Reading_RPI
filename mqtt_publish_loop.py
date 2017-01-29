import paho.mqtt.client as paho
import time
 
def on_publish(client, userdata, mid):
    print("mid: "+str(mid))
 
client = paho.Client()
client.on_publish = on_publish
client.connect('192.168.137.43', 1883)
client.loop_start()
c=0.000
while True:
    c+=0.001
    temperature = c#ser.readline()
    (rc, mid) = client.publish('en', str(temperature))
    (rc, mid) = client.publish('vn', "seccond topic")#copy,paste and edit  inorder to publish multiple message to diffrent topic(<topic>,message), confirm sending strings only
    time.sleep(1)

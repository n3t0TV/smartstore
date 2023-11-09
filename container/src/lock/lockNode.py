import json
import rospy
import serial

from time import sleep
from std_msgs.msg import Bool, String, Int8
from container.msg import mqtt_publishers_msg

class Lock:

    STATUS = "1951431614212021\n"
    STATUS_BYTE = bytes(STATUS, 'ascii')

    OPEN = "01121608160013161901\n"
    OPEN_BYTE = bytes(OPEN, 'ascii')

    def __init__(self):
        self.battery = 0
        self.busy = False
        self.status = False
        self.ser = serial.Serial("/dev/ttyS0",115200)
        
        self.lidSensorPub = rospy.Publisher('/sensor_topic/lid',Bool,queue_size=1)
        self.batterySensorPub = rospy.Publisher('/sensor_topic/battery',Int8,queue_size=1)

        rospy.Subscriber('/lock_open_topic', Bool, self.openLock, queue_size=1)
    
    def openLock(self,data):
        if data.data:
            while self.busy:
                sleep(0.1)
            self.busy = True
            self.ser.write(self.OPEN_BYTE)
            self.busy = False
    
    def getFeedback(self):
        if not self.busy:
            self.busy = True
            self.ser.write(self.STATUS_BYTE)
            data = self.ser.readline()
            self.busy = False
            data = str(data, 'ascii')
            data = data.strip("\r\n")
            temp = data.split("&")
            """ Battery """
            bat = int(temp[0])
            if (self.battery != bat):
                self.battery = bat
                # j = {"type":"battery","values":self.battery}
                self.batterySensorPub.publish(data=self.battery)
            """ Lid status """
            stat = True if temp[1] == '1' else False
            if (self.status != stat):
                self.status = stat
                # j = {"type":"status","values":self.status}
                self.lidSensorPub.publish(data=self.status)
    
    def close(self):
        self.ser.close()

def main():
    rospy.init_node('lockNode')
    r = rospy.Rate(0.5)
    lock = Lock()

    while not rospy.is_shutdown():
        lock.getFeedback()
        r.sleep()
    
    lock.close()
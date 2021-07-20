#!/usr/bin/env python3

import rospy
import serial
import json
from transmit_wifi.msg import Transmission

connections = {}

def on_transmit_requested(data):
    ser = connections[data.connection]
    if ser is None:
        rospy.logwarn("[transceive_radio] No such connection: %s", data.connection)
        return
    rospy.loginfo("[transceive_radio] Transmitting %d bytes of data", data.length)
    ser.write(bytes(data.data))

def transmit():
    rospy.init_node('transceive_radio')

    with open("../json/radio.json") as file:
        j = json.load(file)
        for c in j:
            rospy.loginfo("[transceive_radio] Connecting to '%s' with serial %s (baudrate: %d)", c['name'], c['device'], c['baud_rate'])
            if c['name'] in connections.keys():
                rospy.logwarn("[transceive_radio] Connection named %s already exists", c['name'])

            ser_open = None
            for s in connections.values():
                if s.port == c['device']:
                    ser_open = s

            if ser_open:
                connections[c['name']] = s
            else:
                connections[c['name']] = serial.Serial(c['device'], c['baud_rate'])

    rospy.Subscriber("radio_out", Transmission, on_transmit_requested)
    pub = rospy.Publisher('radio_in', Transmission, queue_size=10)

    rate = rospy.rate(10000)

    while not rospy.is_shutdown():
        for (name, ser) in connections.items():
            if ser.inWaiting():
                header = ser.read(9)
                data_length = (header[5] << 24) + (header[6] << 16) + (header[7] << 8) + (header[8])
                rospy.loginfo("[transceive_radio] Incoming data from '%s' of length %d", name, data_length)

                data = bytes([])
                nRead = 0
                while nRead < data_length:
                    if ser.inWaiting():
                        chunk = ser.read(ser.inWaiting())
                        data += chunk
                        nRead += len(chunk)
                        rospy.loginfo("[transceive_radio] Read %d bytes worth of data so far", nRead)

                t = Transmission(data=header+data, length=len(header+data), connection=name)
                pub.publish(t)

        rate.sleep()

    for (name, ser) in connections.items():
        ser.close()


if __name__ == '__main__':
    transmit()
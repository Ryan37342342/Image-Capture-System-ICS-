# !/usr/bin/env
import rospy
import serial
from std_msgs.msg import String
from ublox_gps import UbloxGps


def run_gps():
    pub = rospy.Publisher('gps_coordinates', String, queue_size=10)
    rospy.init_node('gps_pub_node', anonymous=True)
    rospy.loginfo("GPS node started")
    # initalize gps
    port = serial.Serial('/dev/ttyACM0', baudrate=38400, timeout=10)
    gps = UbloxGps(port)

    # while the node is running
    while not rospy.is_shutdown():
        geo = gps.geo_coords()
        # publish them
        coord = "lat:" + str(geo.lat) + " long:" + str(geo.lon)
        pub.publish(coord)
        # print(coord + "\n")


if __name__ == '__main__':
    try:
        run_gps()
    except rospy.ROSException:
        pass

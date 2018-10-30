#!/usr/bin/env python

# Translates from NavSatFix to GPSFix and back

import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from novatel_gps_msgs.msg import Inspva
from novatel_gps_msgs.msg import Inscov



class InsTranslator(object):
    
    def __init__(self):
        self.navsat_pub = rospy.Publisher('navsat_fix_out', NavSatFix, queue_size=10)        
        self.ins_sub = rospy.Subscriber("inspva", Inspva, self.ins_callback)
        self.inscov_sub = rospy.Subscriber("inscov", Inscov, self.inscov_callback)
        self.pos_cov = Inscov()
        self.navsat_msg = NavSatFix()

    def inscov_callback(self,inscov_msg):
        self.pos_cov = inscov_msg
        pass
    
    # Translates from INSpva to NavSatFix.
    def ins_callback(self, ins_msg):
        self.navsat_msg.header = ins_msg.header

        self.navsat_msg.status.status= NavSatStatus.STATUS_SBAS_FIX
        self.navsat_msg.status.service = NavSatStatus.SERVICE_GPS

        self.navsat_msg.latitude=ins_msg.latitude
        self.navsat_msg.longitude=ins_msg.longitude
        self.navsat_msg.altitude=ins_msg.height
        self.navsat_msg.position_covariance= self.pos_cov.position_covariance
        self.navsat_pub.publish(self.navsat_msg)

if __name__ == '__main__':
    rospy.init_node('ins_translator', anonymous=True)
    translator = InsTranslator()
    rospy.spin()

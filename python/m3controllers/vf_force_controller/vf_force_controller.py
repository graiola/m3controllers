#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import yaml
import os 
import m3.toolbox as m3t
import vf_force_controller_pb2 as m3_vf
from m3.component import M3Component
import m3.component_factory as m3f
import m3.rt_proxy as m3p
"""
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    controller.command.contact_force=data.data
"""


class M3Controller(M3Component):
    """Example component"""
    def __init__(self,name,controller_type):
        M3Component.__init__(self,name,controller_type)
        self.status = m3_vf.VfForceControllerStatus()
        self.command = m3_vf.VfForceControllerCommand()
        self.param = m3_vf.VfForceControllerParam()
    def callback(self,data):
        """rospy.loginfo(rospy.get_caller_id() + "Contact force %s", data.data)"""
        self.command.contact_force=data.data
    def listener(self):
        rospy.Subscriber("/optoforce_ros_bridge_node/contact_force", Bool, self.callback)

if __name__ == '__main__':
    rospy.init_node('contact_force_listener', anonymous=True)
    controller=M3Controller('vf_force_controller_test','vf_force_controller')
    proxy = m3p.M3RtProxy()
    proxy.start()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        controller.listener()
        proxy.publish_command(controller)
        proxy.step()
        rate.sleep()



"""
controller=M3Controller('vf_force_controller_test','vf_force_controller')
controller.command.contact_force=True
proxy.publish_command(controller)
proxy.step()
#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)  
def listener():

    rospy.init_node('contact_force_listener', anonymous=True)

    rospy.Subscriber("/optoforce_ros_bridge_node/contact_force", Bool, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
"""

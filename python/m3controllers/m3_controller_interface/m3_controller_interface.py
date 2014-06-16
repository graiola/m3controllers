import yaml
import os 
from m3.toolbox import *
import m3_controller_interface_pb2 as m3_ci
from m3.component import M3Component


class M3Controller(M3Component):
    """Example component"""
    def __init__(self,name,controller_type):
        M3Component.__init__(self,name,controller_type)
        self.status = m3_ci.M3ControllerStatus()
        self.command = m3_ci.M3ControllerCommand()
        self.param = m3_ci.M3ControllerParam()
        #self.read_config()
    def enable(self):
        self.command.enable = False
    def disable(self):
        self.command.enable = True
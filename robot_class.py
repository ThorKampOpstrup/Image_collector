import rtde_control
import rtde_receive

##generate class

class robot:
    def __init__(self, ip):
        self.controller = rtde_control.RTDEControlInterface(ip)
        self.receiver = rtde_receive.RTDEReceiveInterface(ip)
    
    def get_pose(self):
        # print("Current pose: " + str(self.receiver.getActualTCPPose()))
        return self.receiver.getActualTCPPose()
    
    def get_tcp_offset(self):
        return self.controller.getTCPOffset()
    
    def teachMode(self):
        if self.controller.teachMode():
            print("Freedrive mode enabled")
        else:
            print("Freedrive mode not enabled")

    def endTeachMode(self):
        if self.controller.endTeachMode():
            self.controller.endFreedriveMode()
            print("Freedrive mode disabled")
        else:
            print("Freedrive mode not disabled")

    def set_tcp_offset(self, offset=[0,0,0,0,0,0]):
        self.controller.setTcp(offset)

    def move_to(self, pose, speed=0.4, acc=0.5): # pose is a list of 6 floats, returns the obtained pose
        # self.controller.moveL(pose)
        print("hello")
        self.controller.moveJ_IK(pose, speed, acc)
        print("hello")
        return self.get_pose()
    
    def disconnect(self):
        self.controller.disconnect()
        self.receiver.disconnect()
        print("Connection closed")
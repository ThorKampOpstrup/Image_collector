import rtde_control
import rtde_receive

##generate class

class robot:
    def __init__(self, ip):
        self.controller = rtde_control.RTDEControlInterface(ip)
        self.receiver = rtde_receive.RTDEReceiveInterface(ip)
    
    def get_pose(self):
        print("Current pose: " + str(self.receiver.getActualTCPPose()))

        return self.receiver.getActualTCPPose()
    
    def move_to(self, pose, speed=0.4, acc=0.5): # pose is a list of 6 floats, returns the obtained pose
        self.controller.moveL(pose, speed, acc)
        return self.get_pose()
    
    def disconnect(self):
        self.controller.disconnect()
        self.receiver.disconnect()
        print("Connection closed")
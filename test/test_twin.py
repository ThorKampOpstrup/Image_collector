import sys
# caution: path[0] is reserved for script path (or '' in REPL)
sys.path.insert(1, '../')

import setup_class
from twin import *

shutter_param = 5000
ip_addr = "10.42.0.63"


if __name__ == "__main__":
    setup = setup_class.setup(ip=ip_addr, exposure=shutter_param)
    tmp_pose = [0.27387209441031374, 0.3949999912907006, 0.29365682574189694, 1.5165206920780452, -0.4820064807657478, 0.4820069862726179]
    show_robot_pose_freedrive(tmp_pose, setup)
import rospy
from pi_trees_lib.pi_trees_lib import *

class CheckBattery(Task):
    def __init__(self, blackboard, low_battery_threshold):
        super(CheckBattery, self).__init__('CHECK_BATTERY')

        self.blackboard = blackboard
        self.low_battery_threshold = low_battery_threshold

    def run(self):
        if self.black_board.robot_battery_level is None:
            return None
        else:
            if self.black_board.robot_battery_level < self.low_battery_threshold:
                rospy.loginfo("LOW BATTERY - level: " + str(int(self.black_board.robot_battery_level)))
                return False
            else:
                return True
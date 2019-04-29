import rospy
import queue

from drone_control_system.msg import RunLiability, FinalizeLiability
from . import traking_thread
from threading import Timer


class DroneManager:
    def __init__(self):
        rospy.init_node('drone_manager_node')

        self.available_drones = queue.Queue()
        for x in range(3):
            self.available_drones.put(x + 1)

        rospy.Subscriber('run_liability', RunLiability, self.run_liability)
        self.result_queue = queue.Queue()

        self.finalize_pub = rospy.Publisher('finalize_liability', FinalizeLiability, queue_size=10)

        rospy.loginfo('Node is loaded')

    def run_liability(self, msg):
        if not self.available_drones.empty():
            drone = self.available_drones.get()
            t = traking_thread.TrakingThread(self.result_queue, drone, msg.liability, msg.filename)
            t.run()

    def spin(self):
        def finalize_liabilities():
            if not self.result_queue.empty():
                drone, liability = self.result_queue.get()
                msg = FinalizeLiability()
                msg.liability = liability
                msg.status = "Ok"
                self.finalize_pub.publish(msg)
                self.available_drones.put(drone)
            Timer(1, finalize_liabilities).start()

        finalize_liabilities()
        rospy.spin()

from . import drone_manager


def fly_by_local_points_node():
    drone_manager.DroneManager().spin()

from . import fly_by_local_points_service

def fly_by_local_points_node():
    fly_by_local_points_service.FlyByLocalPointsService().spin()


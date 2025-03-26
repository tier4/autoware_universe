#!/usr/bin/python3

"""
This module defines a function that manages the map and global path
"""
import os
from autoware_planning_msgs.msg import LaneletRoute
from geometry_msgs.msg import Pose

import lanelet2
from autoware_lanelet2_extension_python.utility import load_info_from_yaml, MapProjectorInfo
from autoware_lanelet2_extension_python.projection import MGRSProjector


def automatic_find_projector_yaml(map_path):
    """in the same directory with the name projector_info.yaml"""
    # get the directory of the map file path
    map_dir = os.path.dirname(map_path)
    # get the name of the map file without the extension
    projector_info = os.path.join(map_dir, "map_projector_info.yaml")
    if os.path.exists(projector_info):
        return projector_info
    else:
        return None

def get_lanelet2_projector(projector_info: MapProjectorInfo):
    """
    プロジェクタ情報に基づいて、適切なlanelet2のプロジェクタを返します。
    
    引数:
      projector_info: プロジェクタ情報を保持するオブジェクト
      
    戻り値:
      lanelet2のプロジェクタオブジェクト
      
    例外:
      ValueError: サポートされていないプロジェクタタイプが指定された場合
    """
    # LOCAL_CARTESIAN_UTM の場合
    if projector_info.projector_type == "LOCAL_CARTESIAN_UTM":
        position = lanelet2.GPSPoint(
            projector_info.map_origin.latitude,
            projector_info.map_origin.longitude,
            projector_info.map_origin.altitude
        )
        origin = lanelet2.io.Origin(position)
        return lanelet2.projection.UtmProjector(origin)
    
    # MGRS の場合
    elif projector_info.projector_type == "MGRS":
        projector = MGRSProjector(lanelet2.io.Origin(0, 0))
        projector.setMGRSCode(projector_info.mgrs_grid)
        return projector
    
    # TRANSVERSE_MERCATOR の場合
    elif projector_info.projector_type == "TRANSVERSE_MERCATOR":
        position = lanelet2.core.GPSPoint(
            projector_info.map_origin.latitude,
            projector_info.map_origin.longitude,
            projector_info.map_origin.altitude
        )
        origin = lanelet2.Origin(position)
        return TransverseMercatorProjector(origin)
    
    # LOCAL_CARTESIAN の場合
    elif projector_info.projector_type == "LOCAL_CARTESIAN":
        position = lanelet2.core.GPSPoint(
            projector_info.map_origin.latitude,
            projector_info.map_origin.longitude,
            projector_info.map_origin.altitude
        )
        origin = lanelet2.ioOrigin(position)
        return lanelet2.projection.LocalCartesianProjector(origin)

class MapManager:
    def __init__(self, map_path, local_map_range=80.0):
        self.map_path = map_path
        self.projector_yaml = automatic_find_projector_yaml(map_path)
        if self.projector_yaml is None:
            self.projector = MGRSProjector(lanelet2.io.Origin(0.0, 0.0))
        else:
            self.projector = get_lanelet2_projector(load_info_from_yaml(self.projector_yaml))
        self.map_object = lanelet2.io.load(self.map_path, self.projector)

        # ## Also load the connection first
        # traffic_rules = lanelet2.traffic_rules.create(
        #     lanelet2.traffic_rules.Locations.Germany,
        #     lanelet2.traffic_rules.Participants.Vehicle,
        # )
        # self.routing_graph = lanelet2.routing.RoutingGraph(self.map_object, traffic_rules)

        self.global_path = []
        self.local_map_range = local_map_range

    def set_global_path(self, msgs:LaneletRoute):
        self.global_path = [segment.preferred_primitive.id for segment in msgs.segments]

    def fetch_local_map(self, position:Pose):
        """This methods should return the map elements near the position, and also return nearby global routes"""
        x = position.position.x
        y = position.position.y
        z = position.position.z
        # get the map elements near the position
        search_bounding_box = lanelet2.core.BoundingBox2d(
            lanelet2.core.BasicPoint2d(float(x - self.local_map_range), float(y - self.local_map_range)),
            lanelet2.core.BasicPoint2d(float(x + self.local_map_range), float(y + self.local_map_range))
        )
        nearby_lanelets =  self.map_object.laneletLayer.search(search_bounding_box)
        nearby_lanelets_ids = [lanelet.id for lanelet in nearby_lanelets]

        nearby_global_path = []
        for lanelet_id in self.global_path:
            if lanelet_id in nearby_lanelets_ids:
                nearby_global_path.append(lanelet_id)

        return {"map_elements": nearby_lanelets, "routes": nearby_global_path, "nearby_lanelets_ids": nearby_lanelets_ids}

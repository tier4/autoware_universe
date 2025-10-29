import typing
from dataclasses import dataclass
import lanelet2.core  # For type hinting
from shapely.geometry import LineString, MultiLineString
from shapely.validation import make_valid


@dataclass
class AdjacentLanelet:
    lanelet: lanelet2.core.Lanelet
    leftBound: lanelet2.core.LineString3d
    rightBound: lanelet2.core.LineString3d

    @property
    def id(self):
        return self.lanelet.id

@dataclass
class DrivableLaneSegment:
    """
    Holds the drivable area information for a single route lanelet.
    This mimics the C++ 'DrivableLanes' struct, but simplified.
    """
    route_lanelet: lanelet2.core.Lanelet
    # The 'outermost' lanelet is the adjacent lanelet or the lanelet itself
    left_lane: typing.Union[AdjacentLanelet, lanelet2.core.Lanelet]
    right_lane: typing.Union[AdjacentLanelet, lanelet2.core.Lanelet]

    @property
    def left_boundary(self):
        return self.left_lane.leftBound

    @property
    def right_boundary(self):
        return self.right_lane.rightBound


def process_route_to_drivable_area(lanelet_map: lanelet2.core.LaneletMap, route_ids: typing.List[int]):
    """
    Main function to process a list of route IDs.

    Args:
        route_ids: A list of integer IDs for the lanelets on the route.
        lanelet_map: The fully loaded lanelet2.core.LaneletMap object.

    Returns:
        A tuple containing:
        1. all_drivable_ids (Set[int]): A general set of all unique drivable IDs.
        2. outer_left_bounds (List[LineString3d]): A single list of the leftmost boundary linestrings.
        3. outer_right_bounds (List[LineString3d]): A single list of the rightmost boundary linestrings.
    """
    drivable_segments, all_drivable_ids = calculate_drivable_area_one_level(lanelet_map, route_ids)
    
    # Post-process the results
    left_boundary = [seg.left_boundary for seg in drivable_segments]
    right_boundary = [seg.right_boundary for seg in drivable_segments]
    
    return all_drivable_ids, left_boundary, right_boundary


def calculate_drivable_area_one_level(lanelet_map: lanelet2.core.LaneletMap, route_ids: typing.List[int]):
    """
    Iterates through the route and finds the neighbors one level to the
    left and right for each route lanelet.
    """
    drivable_segments = []
    all_drivable_ids = set()

    for lanelet_id in route_ids:
        try:
            route_lanelet = lanelet_map.laneletLayer.get(lanelet_id)
        except RuntimeError: # lanelet2 python binding throws RuntimeError on not found
            print(f"Failed to get lanelet with ID {lanelet_id} from map. Skipping.")
            continue
        
        # Add route lanelet to the set
        all_drivable_ids.add(route_lanelet.id)
        
        # Get the immediate neighbor to the left (returns None if not found)
        left_neighbor = find_geometric_neighbor(lanelet_map, route_lanelet, 'left')

        # Get the immediate neighbor to the right (returns None if not found)
        right_neighbor = find_geometric_neighbor(lanelet_map, route_lanelet, 'right')

        # Add neighbors (if they exist) to the set
        if left_neighbor:
            all_drivable_ids.add(left_neighbor.id)
        if right_neighbor:
            all_drivable_ids.add(right_neighbor.id)

        # Determine the final outer lane.
        # If a neighbor was found, use it. Otherwise, use the route_lanelet itself.
        left_lane = left_neighbor if left_neighbor else route_lanelet
        right_lane = right_neighbor if right_neighbor else route_lanelet

        # Store the result for this segment
        segment = DrivableLaneSegment(
            route_lanelet=route_lanelet,
            left_lane=left_lane,
            right_lane=right_lane
        )
        drivable_segments.append(segment)
        
    return drivable_segments, all_drivable_ids


def find_geometric_neighbor(lanelet_map: lanelet2.core.LaneletMap, lanelet: lanelet2.core.Lanelet, direction: str):
    """
    Finds the single adjacent neighbor using the 'findUsages' (shared linestring) method.
    This handles both same-direction and opposite-direction lanes.
    
    This is the Python equivalent of C++ 'getRightLanelet' + 'getRightOppositeLanelets'
    but without the RoutingGraph.
    """
    if direction == 'left':
        # --- We are looking for a lane to our LEFT ---
        bound_same_dir = lanelet.leftBound        
        bound_opp_dir = lanelet.leftBound.invert()  

        # Check for SAME-direction neighbor
        for candidate in lanelet_map.laneletLayer.findUsages(bound_same_dir):
            if candidate.id != lanelet.id:
                return AdjacentLanelet(lanelet=candidate, 
                                       leftBound=candidate.leftBound, 
                                       rightBound=candidate.rightBound)

        # Check for OPPOSITE-direction neighbor
        for candidate in lanelet_map.laneletLayer.findUsages(bound_opp_dir):
            if candidate.id != lanelet.id:
                return AdjacentLanelet(lanelet=candidate, 
                                       leftBound=candidate.rightBound.invert(), 
                                       rightBound=candidate.leftBound.invert())

    elif direction == 'right':
        # --- We are looking for a lane to our RIGHT ---
        bound_same_dir = lanelet.rightBound       
        bound_opp_dir = lanelet.rightBound.invert()

        # Check for SAME-direction neighbor
        for candidate in lanelet_map.laneletLayer.findUsages(bound_same_dir):
            if candidate.id != lanelet.id:
                return AdjacentLanelet(lanelet=candidate, 
                                       leftBound=candidate.leftBound, 
                                       rightBound=candidate.rightBound)

        # Check for OPPOSITE-direction neighbor 
        for candidate in lanelet_map.laneletLayer.findUsages(bound_opp_dir):
            if candidate.id != lanelet.id:
                return AdjacentLanelet(lanelet=candidate, 
                                       leftBound=candidate.rightBound.invert(), 
                                       rightBound=candidate.leftBound.invert())

    return None  # No neighbor found in this direction

def convert_lanelet_linestrings_to_shapely(linestrings: list, logger) -> typing.Optional[MultiLineString]:
        """
        Converts a list of lanelet2.core.LineString3d objects into a single
        Shapely MultiLineString object for efficient geometric checks.
        
        Args:
            linestrings: A list of lanelet2.core.LineString3d objects.
            
        Returns:
            A MultiLineString object, or None if conversion fails.
        """
        shapely_lines = []
        try:
            for ls in linestrings:
                # Extract 2D points (x, y) from the LineString3d
                points = [(p.x, p.y) for p in ls]
                if len(points) >= 2:
                    shapely_lines.append(LineString(points))
            
            if not shapely_lines:
                logger.warn("No valid LineStrings found to create MultiLineString.")
                return None
                
            multi_line = MultiLineString(shapely_lines)
            
            # Ensure the geometry is valid
            if not multi_line.is_valid:
                logger.warn("Created MultiLineString is invalid, attempting to fix...")
                multi_line = make_valid(multi_line)
                
            return multi_line
            
        except Exception as e:
            logger.error(f"Failed to convert lanelet linestrings to Shapely: {e}")
            return None
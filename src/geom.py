# Algorithm for checking if a point lies in a polygon
# based on https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/ 

import numpy as np
import csv
from collections import namedtuple

Point = namedtuple('Point', ['x', 'y'])

# TODO: Load csv file containing lane ROIs
# class LaneEstimator:
#     def __init__(self, lane_cfg="/tmp/sensible/lane_rois.csv"):
#         # parse lane_cfg, storing each lane coordinates
#         # as an array of 2D/3D coordinates
#         self.lanes = []
#         with open(lane_cfg) as csvfile:
#             lane_rois = csv.reader(csvfile)
#             lane_id = 1
#             for i, row in enumerate(lane_rois):
#                 if i == 0:
#                     continue
#                 if int(row[0]) > lane_id:
#                     lane_id += 1
#                 if len(self.lanes) < lane_id:
#                     self.lanes.append([])
#                 self.lanes[lane_id-1].append(Point(float(row[1]), float(row[2])))

def on_segment(p, q, r):
    """
    Given three collinear points p,q,r, checks if
    point q lines on line seqment 'pr'
    """
    if q.x <= max(p.x, r.x) and q.x >= min(p.x, r.x) and \
            q.y <= max(p.y, r.y) and q.y >= min(p.y, r.y):
        return True
    else:
        return False

def orientation(p, q, r):
    """
    Finds orientation of ordered triplet (p, q, r)
    0 --> (p, q, r) are colinear
    1 --> Clockwise
    2 --> Counterclockwise
    """
    val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
    if val == 0:
        return 0
    else:
        if val > 0:
            return 1
        else:
            return 2

def do_intersect(p1, q1, p2, q2):
    """
    Returns true if line segment p1q1 and 
    p2q2 intersect.
    """
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case
    if o1 != o2 and o3 != o4:
        return True

    # Special Cases
    # p1, q1, and p2 are colinear and p2 lies on segment p1q1
    if o1 == 0 and on_segment(p1, p2, q1):
        return True
    
    # p1, q1, and p2 are colinear and q2 lies on segment p1q1
    if o2 == 0 and on_segment(p1, q2, q1):
        return True
    
    # p2, q2, and p1 are colinear and p1 lies on segment p2q2
    if o3 == 0 and on_segment(p2, p1, q2):
        return True

    # p2, q2, and q1 are colinear and q1 lies on segment p2q2
    if o4 == 0 and on_segment(p2, q1, q2):
        return True

    return False

def point_in_polygon(query, polygon):
    """
    Fast algorithm for checking whether a query point
    is contained inside a polygon
    """
    # Must be at least 3 points in polygon
    if len(polygon) < 3:
        return False
    n = len(polygon)

    # Create a point for line segment from p to infinity
    extreme = Point(1000000, query.y)

    count = 0
    for i in range(1, n):
        # Check if the line segment from query to extreme
        # intersects with the line segment from polygon[i-1] to 
        # polygon[i]
        if do_intersect(polygon[i-1], polygon[i], query, extreme):
            # If the point query is colinear with line segment 
            # polygon[i-1]-polygon[i]
            if orientation(polygon[i-1], query, polygon[i]) == 0:
                return on_segment(polygon[i-1], query, polygon[i])
            count += 1
    return (count % 2 == 1)

# def get_lane_estimate(self, query, lane_history=None):
#     """
#     Return the lane estimate

#     Args:
#         query: Numpy 2D/3D point in world-coordinates
#         lane_history: 1-D array of previous (non-smoothed)
#             lane estimates
#     Returns:
#         smoothed_estimate: integer representing the moving
#             average estimate over the lane history and latest
#             point
#         lane: the polygon (lane) that the query is contained
#             within
#     """
#     q = Point(query[0], query[1])
#     lane_idx = -1
#     for idx, lane in enumerate(self.lanes):
#         if LaneEstimator.point_in_polygon(q, lane):
#             return idx+1
#     return lane_idx

if __name__ == '__main__':
    import sys
    intersection = sys.argv[1]
    lane_cfg = "/tmp/sensible/{}_lane_rois.csv".format(intersection)
    le = LaneEstimator(lane_cfg)

    query = np.array([800, 500])
    print(le.get_lane_estimate(query))


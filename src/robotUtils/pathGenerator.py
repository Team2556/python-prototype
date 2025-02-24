

import math
import numpy as np
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from constants import AprilTags

# Helper function to check if two line segments intersect
def is_intersecting(p1, p2, q1, q2):
    """Check if two line segments (p1->p2 and q1->q2) intersect."""
    p1 = point_xy_extractor(p1)
    p2 = point_xy_extractor(p2)
    q1 = point_xy_extractor(q1)
    q2 = point_xy_extractor(q2)
    def cross_product(a, b):
        return a[0] * b[1] - a[1] * b[0]

    def subtract(v1, v2):
        return (v1[0] - v2[0], v1[1] - v2[1])

    d1 = cross_product(subtract(q2, q1), subtract(p1, q1))
    d2 = cross_product(subtract(q2, q1), subtract(p2, q1))
    d3 = cross_product(subtract(p2, p1), subtract(q1, p1))
    d4 = cross_product(subtract(p2, p1), subtract(q2, p1))

    return (d1 * d2 < 0) and (d3 * d4 < 0)

def point_xy_extractor(point):
    """Extract the x and y coordinates from a Pose2d, Translation2d, or tuple (x,y)."""
    if isinstance(point, Pose2d):
        cx, cy = point.translation().x, point.translation().y
    elif isinstance(point, Translation2d):
        cx, cy = point.x, point.y
    else:
        cx, cy = point
    return cx,cy

def hexagon_edges(center, size, angle_offset=30):
    """Generate the edges of a hexagon given its center and size.
    Center can be Pose2d, Translation2d, or tuple (x,y).
    Angles are measured in degrees, with 0 degrees pointing from Blue Alliance to Red Alliance and running counterclockwise."""
    cx, cy = point_xy_extractor(center)
    
    vertices = [(cx + size * math.cos(math.radians(angle+angle_offset)), cy + size * math.sin(math.radians(angle+angle_offset)))
                for angle in range(0, 360, 60)]
    return [(vertices[i], vertices[(i + 1) % 6]) for i in range(6)]

# Function to project a point perpendicular to an edge
def project_perpendicular(point, edge, distance):
    """Project a point perpendicular to an edge by a given distance."""
    x1, y1 = edge[0]
    x2, y2 = edge[1]

    # Direction vector of the edge
    dx, dy = x2 - x1, y2 - y1

    # Perpendicular direction vector (rotate 90 degrees)
    perp_dx, perp_dy = -dy, dx

    # Normalize the perpendicular direction
    length = math.sqrt(perp_dx**2 + perp_dy**2)
    perp_dx, perp_dy = perp_dx / length, perp_dy / length

    # Calculate the projected point
    px = point[0] + perp_dx * distance
    py = point[1] + perp_dy * distance

    return (px, py)


def calculate_angle(p1, p2):
    """Calculate the angle between two points."""
    return np.arctan2(p2[1] - p1[1], p2[0] - p1[0])

def generate_pose2d_path(points):
    """Convert a list of (x, y) points into a list of Pose2d with averaged rotation angles."""
    poses = []
    num_points = len(points)
    
    for i in range(num_points):
        if i == 0:
            angle = calculate_angle(points[i], points[i + 1])
        elif i == num_points - 1:
            angle = calculate_angle(points[i - 1], points[i])
        else:
            angle1 = calculate_angle(points[i - 1], points[i])
            angle2 = calculate_angle(points[i], points[i + 1])
            angle = (angle1 + angle2) / 2
        
        poses.append(Pose2d(points[i][0], points[i][1], Rotation2d(angle)))
    
    return poses

def adjust_points_to_outside_circle(path, center, radius):
    """Adjust the points in a path to be outside a circle with the given center and radius; Except the last point."""

    adjusted_path = []
    if len(path) == 1:
        i=0
        x, y = point_xy_extractor( path[i] )
        cx, cy = center
        dx, dy = x - cx, y - cy
        distance = math.sqrt(dx**2 + dy**2)
        if distance < radius:
            scale = radius / distance
            x = cx + dx * scale
            y = cy + dy * scale
        print(f"Adjusting ONLY point {i} at {path[i]} with distance {distance} and radius {radius}:{distance < radius=}/n new point {x},{y}")
        adjusted_path.append((x, y))
    else:
        for i in range(len(path) - 1):
            print(f"Adjusting point {i} at {path[i]}")
            x, y = point_xy_extractor( path[i])
            cx, cy = center
            dx, dy = x - cx, y - cy
            distance = math.sqrt(dx**2 + dy**2)
            if distance < radius:
                scale = radius / distance
                x = cx + dx * scale
                y = cy + dy * scale
            adjusted_path.append((x, y))
        # if len(path) > 1:
        print(f"Adding last point {path[-1]}")
        adjusted_path.append( point_xy_extractor (path[-1]))
    return adjusted_path
    

# Function to iteratively create a path from start to finish
def create_path_with_projections(start, finish, 
                                 hex_centers = [Translation2d(x=4.489323, y=4.025900),
                                                Translation2d(x=13.058902, y=4.025900)], #TODOne: Got from april tag database- update with accurate values for defaults
                                 hex_sizes = [0.831723, 0.831723], 
                                 projection_distance = .75): 
    """Creates a path from start to finish, projecting out perpendicularly at intersections and adjusting iteratively.
    If a projected point is within the distance_threshold of the current point, the projection is extended by min_step_size.
    hex_centers and hex_sizes are lists of the centers and sizes of hexagons to avoid.
    """
    #((AprilTags[18-1].pose.translation() + AprilTags[21-1].pose.translation() )/2).toTranslation2d()
    #((AprilTags[7-1].pose.translation() + AprilTags[10-1].pose.translation() )/2).toTranslation2d()
    #((AprilTags[18-1].pose.translation() - AprilTags[21-1].pose.translation() )/2).toTranslation2d()
    edges =[]
    for center, size  in zip(hex_centers, hex_sizes):
        edges += hexagon_edges(center, size)
    current_point = point_xy_extractor(start)
    finish = point_xy_extractor(finish)
    path = [start]

    max_inc = 100000
    inc = 0
    while True and inc < max_inc:
        inc += 1
        found_intersection = False
        closest_edge = None
        closest_intersection = None
        min_distance = float('inf')

        for q1, q2 in edges:
            print(f"Checking edge {q1} -> {q2} for intersection with {current_point} -> {finish}")
            if is_intersecting(current_point, finish, q1, q2):
                found_intersection = True
                intersection_x = (current_point[0] + finish[0]) / 2
                intersection_y = (current_point[1] + finish[1]) / 2
                intersection_point = (intersection_x, intersection_y)
                
                distance_to_edge = min(math.dist(current_point, q1), math.dist(current_point, q2))
                if distance_to_edge < min_distance:
                    min_distance = distance_to_edge
                    closest_edge = (q1, q2)
                    closest_intersection = intersection_point

        if not found_intersection:#closest_edge is None:
            print("No intersection found, breaking")
            break
        # projection_distance = 0.75
        projected_point1 = adjust_points_to_outside_circle([closest_intersection], hex_centers[0], hex_sizes[0]+projection_distance)[0]
        projected_point = adjust_points_to_outside_circle([projected_point1], hex_centers[1], hex_sizes[1]+projection_distance)[0]
        # print(f"Projected point is same after second time through :{projected_point1==projected_point=} ")
        #project_perpendicular(closest_intersection, closest_edge, projection_distance)

        # while math.dist(current_point, projected_point) < distance_threshold:
        #     projected_point = (
        #         projected_point[0] + min_step_size * (closest_edge[1][0] - closest_edge[0][0]),
        #         projected_point[1] + min_step_size * (closest_edge[1][1] - closest_edge[0][1])
        #     )
        path.append(projected_point)
        current_point = projected_point
        print('another iteration')
        '''found_intersection = True
        
        if not found_intersection:
            break'''

    path.append(finish)
    radial_extension = projection_distance + 0.1
    path = adjust_points_to_outside_circle(path, hex_centers[0], hex_sizes[0]+radial_extension)
    path = adjust_points_to_outside_circle(path, hex_centers[1], hex_sizes[1]+radial_extension)
    print(f" Hex centers Red: {hex_centers[1]} Hex sizes Red: {hex_sizes[1]}")
    # return path
    print(f"path before midpoints: {path}")
    def add_midpoints(path):
        """Add midpoints to the path and check if the path is within the circle."""
        new_path = []#[path[0]]
        for i in range(len(path) - 1):
            new_path.append(path[i])
            new_path.append(((path[i][0] + path[i + 1][0]) / 2, (path[i][1] + path[i + 1][1]) / 2))
        new_path.append(path[-1])
        # new_path = adjust_points_to_outside_circle(new_path,  hex_centers[0], hex_sizes[0]+radial_extension)
        # new_path = adjust_points_to_outside_circle(new_path,  hex_centers[1], hex_sizes[1]+radial_extension)
        return new_path
    path = add_midpoints(path)
    print(f"Final path: {path}")

    return generate_pose2d_path(path)

# Example usage:
'''p1 = (1, 1)
p2 = (4, 4)
hex_center = (2.5, 2.5)
hex_size = 1.5
projection_distance = 0.5
min_step_size = 0.2
distance_threshold = 0.3

path = create_path_with_projections(p1, p2)#, hex_center, hex_size, projection_distance, min_step_size, distance_threshold)
print(f"Generated path: {path}")'''

#create a function that takes the center of a circle and a raduis
# it also takes a current point and a target point
# creates a line segment inside the circle perpendicular to the line segment from the current point to the target point
# if there is an intersection with the interior line then adds a point to the path that is just beyonf the curcumfernece of the circle
def intersect_circle(start, finish, center, radius):
    """Intersect a line segment from start to finish with a circle with the given center and radius."""
    dx, dy = finish[0] - start[0], finish[1] - start[1]
    cx, cy = center
    px, py = start
    a = dx**2 + dy**2
    b = 2 * dx * (px - cx) + 2 * dy * (py - cy)
    c = (px - cx)**2 + (py - cy)**2 - radius**2
    discriminant = b**2 - 4 * a * c
    if discriminant < 0:
        return None
    t1 = (-b + math.sqrt(discriminant)) / (2 * a)
    t2 = (-b - math.sqrt(discriminant)) / (2 * a)
    if 0 <= t1 <= 1:
        return (px + t1 * dx, py + t1 * dy)
    if 0 <= t2 <= 1:
        return (px + t2 * dx, py + t2 * dy)
    return None

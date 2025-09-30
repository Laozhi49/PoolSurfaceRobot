import numpy as np
from shapely.geometry import Polygon, LineString, MultiLineString
from scipy.interpolate import make_interp_spline

def bspline_3points(p0, p1, p2, num_points=50):
    points = np.array([p0, p1, p2])
    t = np.linspace(0, 1, len(points))
    spl_x = make_interp_spline(t, points[:, 0], k=2)
    spl_y = make_interp_spline(t, points[:, 1], k=2)

    t_new = np.linspace(0, 1, num_points)
    x_new = spl_x(t_new)
    y_new = spl_y(t_new)

    return list(zip(x_new, y_new))

def rotate_points(points, angle):
    rot = np.array([[np.cos(angle), -np.sin(angle)],
                    [np.sin(angle),  np.cos(angle)]])
    return np.dot(points, rot.T)

def interpolate_line(x_start, x_end, y, step=0.2):
    xs = np.arange(x_start, x_end, step) if x_start < x_end else np.arange(x_start, x_end, -step)
    return [(x, y) for x in xs] + [(x_end, y)]

def generate_coverage_path(polygon, robot_radius=1.0, step_size=2.0):
    polygon = np.array(polygon)
    # poly_shape = Polygon(polygon)
    
    dx, dy = polygon[1] - polygon[0]
    angle = np.arctan2(dy, dx)
    poly_rot = rotate_points(polygon, -angle)
    poly_rot_shape = Polygon(poly_rot)

    minx, miny, maxx, maxy = poly_rot_shape.bounds

    path_points = []
    y = miny
    left_to_right = True

    while y <= maxy:
        scan_line = LineString([(minx, y), (maxx, y)])
        inter = poly_rot_shape.intersection(scan_line)

        segments = []
        if isinstance(inter, LineString):
            segments = [inter]
        elif isinstance(inter, MultiLineString):
            segments = list(inter)

        for seg in segments:
            if seg.coords[-1][0]-seg.coords[0][0] < 2*robot_radius:
                break

            if left_to_right:
                x_start, x_end = (seg.coords[0][0]+robot_radius, seg.coords[-1][0]-robot_radius)
            else:
                x_start, x_end = (seg.coords[-1][0]-robot_radius, seg.coords[0][0]+robot_radius)

            line_pts = interpolate_line(x_start, x_end, y, step=0.1)
            # line_pts = [(x_start, y), (x_end, y)]
            
            if path_points:
                last_x, last_y = path_points[-1]
                p0 = (last_x,last_y)
                p2 = (x_start,y)
                p1 = ((last_x+x_start)/2-robot_radius,(last_y+y)/2) if left_to_right else ((last_x+x_start)/2+robot_radius,(last_y+y)/2)
                curve = bspline_3points(p0, p1, p2, num_points=50)
                curve_trimmed = curve[1:-1]
                path_points.extend(curve_trimmed)
            

            path_points.extend(line_pts)
            left_to_right = not left_to_right
        y += step_size

    path_points = rotate_points(np.array(path_points), angle)
    return path_points.tolist()

import matplotlib.pyplot as plt

# f1とf2のデータを統合してFとして定義
f1 = {f'f1_{i}': (x, 1) for i, x in enumerate([j * 0.5 for j in range(1, 21)])}
f2 = {f'f2_{i}': (x, -1) for i, x in enumerate([j * 0.5 for j in range(1, 21)])}
F = {f'F_{i}': value for i, value in enumerate([val for _, val in f1.items()] + [val for _, val in f2.items()])}
d = {f'd{i}': (x, 0) for i, x in enumerate(range(6))}

# Bounding boxesの基本定義
A1_base = [(0.5,0.75), (0.5,1.5), (2.5,1.5), (2.5,0.75)]
A2_base = [(0.5,-0.75), (0.5,-1.5), (2.5,-1.5), (2.5,-0.75)]

def translate_points(points, dx, dy):
    """Given a list of points, translate each by dx and dy."""
    return [(x+dx, y+dy) for x, y in points]

def point_inside_polygon(x, y, poly):
    """Check if a point (x, y) is inside a given polygon."""
    n = len(poly)
    inside = False

    p1x, p1y = poly[0]
    for i in range(n + 1):
        p2x, p2y = poly[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        x_intersection = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= x_intersection:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside

def get_included_points_for_d(d_value, F):
    """Get the F points inside the bounding boxes for a given d value."""
    dx, dy = d_value
    A1 = translate_points(A1_base, dx, dy)
    A2 = translate_points(A2_base, dx, dy)
    
    included_points = []

    # Check which F points are inside A1 or A2
    for key, value in F.items():
        if point_inside_polygon(value[0], value[1], A1) or point_inside_polygon(value[0], value[1], A2):
            included_points.append((key, value))

    return included_points

# Collecting the included F points for each d value
included_points_dict = {}
for key, value in d.items():
    included_points_dict[key] = get_included_points_for_d(value, F)

print(included_points_dict)

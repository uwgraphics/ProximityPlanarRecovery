import numpy as np
from scipy.optimize import minimize
import json

DIST_TO_BIN_SLOPE = 73.484
DIST_TO_BIN_INTERCEPT = 13.2521
TMF882X_BIN_WIDTH = 1/DIST_TO_BIN_SLOPE 

ZONE_SPEC_PATH = "zone_spec.json"
with open(ZONE_SPEC_PATH, "r") as f:
    ZONE_SPEC = json.load(f)

def TMF882X_dist_to_bin(dist, slope=DIST_TO_BIN_SLOPE, intercept=DIST_TO_BIN_INTERCEPT):
    """Relates a physical distance to the corresponding histogram bin that distance affects
    for the real world measurements of the TMF882X
    """
    bin = int((slope*dist)+intercept) #TODO is rounding up, down, or nearest best?
    if bin < 0 or bin > 127:
        return None
    return bin

def TMF882X_bin_to_dist(bin, slope=DIST_TO_BIN_SLOPE, intercept=DIST_TO_BIN_INTERCEPT):
    if bin < 0 or bin > 127:
        return None
    return (bin-intercept)/slope

def rots_to_u_vec(x_rot, y_rot):
    """
    Given some angular coordinate rotations (x_rot and y_rot), return a 3D unit vector which points
    in the given direction, relative to the camera's optical axis (which is in the positive z
    direction).

    Args:
        x_rot, y_rot: direction to face in angular coordinates

    Returns:
        3x1 numpy array: a unit vector pointing in the given direction
    """
    # start with the u vector facing out from the camera
    u = np.array([0, 0, 1])
    # to rotate in the positive x angular direction, you need to rotate around the y axis in a 
    # negative direction
    x_rot_mat = np.array([
        [np.cos(x_rot), 0, np.sin(x_rot)],
        [0, 1, 0],
        [-np.sin(x_rot), 0, np.cos(x_rot)]
    ])
    # to rotate in the positive y angular direction, you need to rotate around the x axis in a
    # positive direction
    y_rot_mat = np.array([
        [1, 0, 0],
        [0, np.cos(-y_rot), -np.sin(-y_rot)],
        [0, np.sin(-y_rot), np.cos(-y_rot)],
    ])
    u = x_rot_mat @ u
    u = y_rot_mat @ u

    return u


def fit_plane(pts, initial_est = [0, 0, 1, 0.5]):
    """
    Fit a plane given by ax+d = 0 to a set of points
    Works by minimizing the sum over all points x of ax+d
    Ars:
        pts: array of points in 3D space
    Returns:
        (3x1 numpy array): a vector for plane equation
        (float): d in plane equation
        (float): sum of residuals for points to plane (orthogonal l2 distance)
    """

    pts = np.array(pts)

    def loss_fn(x, points):
        a = np.array(x[:3])
        d = x[3]

        loss = 0
        for point in points:
            loss += np.abs(np.dot(a, np.array(point)) - d)

        return loss

    def a_constraint(x):
        return np.linalg.norm(x[:3]) - 1
    

    soln = minimize(
        loss_fn,
        np.array(initial_est),
        args=(pts),
        method='slsqp',
        constraints=[
            {
                'type': 'eq',
                'fun': a_constraint
            }
        ],
        bounds=[
            (-1, 1),
            (-1, 1),
            (-1, 1),
            (0, None)
        ]
    )

    a = soln.x[:3]
    d = soln.x[3]
    res = soln.fun

    return a, d, res

def fit_plane_zdist(pts):
    """
    https://math.stackexchange.com/a/2306029
    """
    pts = np.array(pts)
    xs = pts[:,0]
    ys = pts[:,1]
    zs = pts[:,2]
    tmp_A = []
    tmp_b = []
    for i in range(len(xs)):
        tmp_A.append([xs[i], ys[i], 1])
        tmp_b.append(zs[i])
    b = np.matrix(tmp_b).T
    A = np.matrix(tmp_A)

    fit = (A.T * A).I * A.T * b
    errors = b - A * fit
    residual = np.linalg.norm(errors)
    # print("solution: %f x + %f y + %f = z" % (fit[0].item(), fit[1].item(), fit[2].item()))

    a = [fit[0].item(), fit[1].item(), -1]
    d = fit[2].item()

    a = a / np.linalg.norm(a)
    d = -d * np.linalg.norm(a)

    if d < 0:
        a = -a
        d = -d

    return a, d, residual

def angle_between_vecs(v1, v2, acute=True):
    """
    Find the angle between two vectors

    https://stackoverflow.com/a/39533085/8841061

    Args:
        v1 (np.array): 3x1 vector
        v2 (np.array): 3x1 vector
        acute (bool): if True, return the acute angle between the vectors. Otherwise, return the
            obtuse angle between the vectors
    
    Returns:
        angle (float): angle between the vectors
    """
    angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
    if (acute == True):
        return angle
    else:
        return 2 * np.pi - angle
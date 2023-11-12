"""
Functions for recovering planar parameters from histograms
"""

import json

import numpy as np
import scipy

from util import *

ZONE_SPEC_PATH = "zone_spec.json"
with open(ZONE_SPEC_PATH, "r") as f:
    ZONE_SPEC = json.load(f)


def direct_method(
    hists,
    measurements,
    reference_hist,
    m=72.07336587889849,
    b=13.155326458933663,
    edge_fov_scale=0.9404948331338918,
    corner_fov_scale=0.8916354321171438,
):
    pts = []
    peak_dists = []
    for hist, single_zone_spec in zip(hists, ZONE_SPEC):
        cx, cy = single_zone_spec["center"]
        # if cx == 0 or cy == 0, this zone spec is an edge zone, so apply edge fov scale
        if cx == 0 or cy == 0:
            cx = cx * edge_fov_scale
            cy = cy * edge_fov_scale
        # otherwise, it's a corner zone, so apply corner fov scale
        else:
            cx = cx * corner_fov_scale
            cy = cy * corner_fov_scale

        # find the peak bin by fitting a cubic
        cubic_hist = scipy.interpolate.CubicSpline(np.arange(128), hist)
        interpolated_hist = cubic_hist(np.arange(0, 128, 0.1))
        peak_bin = interpolated_hist.argmax() / 10

        peak_dist = TMF882X_bin_to_dist(peak_bin, slope=m, intercept=b)
        u = rots_to_u_vec(cx, cy)
        if peak_dist is not None:
            pts.append(u * peak_dist)
            peak_dists.append(peak_dist)

    if np.array(peak_dists).max() < 0.03:
        a, d, res = fit_plane_zdist(pts)
    else:
        a, d, res = fit_plane(pts)

    return a, d, res

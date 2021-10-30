# Copyright (c) 2019-2021, Md Imam Hossain (emamhd at gmail dot com)
# see LICENSE.txt for details

"""
Track segment object
"""

STRAIGHT_CURVATURE = float(1/10000)

class DTrackSegment():
    def __init__(self, _len, _wid, _cf, _ct, _rf, _rt):
        self.length = float(_len)
        self.width = float(_wid)
        self.camber_from = float(_cf)
        self.camber_to = float(_ct)
        self.curvature_from = float(1/_rf)
        self.curvature_to = float(1/_rt)
        self.camber_rate = float((_ct - _cf)/_len)
        self.curvature_rate = float((self.curvature_to - self.curvature_from)/_len)

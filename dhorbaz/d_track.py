# Copyright (c) 2019-2022, Md Imam Hossain (emamhd at gmail dot com)
# see LICENSE.txt for details

"""
Track segment object
"""

STRAIGHT_CURVATURE = float(1/10000)

class DTrackSegment():
    def __init__(self, _len, _widf, _widt, _cf, _ct, _rf, _rt, _ef, _et):
        self.length = float(_len)
        self.width_from = float(_widf)
        self.width_to = float(_widt)
        self.camber_from = float(_cf)
        self.camber_to = float(_ct)
        self.elevation_from = float(_ef)
        self.elevation_to = float(_et)
        self.curvature_from = float(1/_rf)
        self.curvature_to = float(1/_rt)
        self.width_rate = (self.width_to - self.width_from) / self.length
        self.camber_rate = (self.camber_to - self.camber_from) / self.length
        self.curvature_rate = (self.curvature_to - self.curvature_from) / self.length
        self.elevation_rate = (self.elevation_to - self.elevation_from) / self.length

# Copyright (c) 2019-2020, Md Imam Hossain (emamhd at gmail dot com)
# see LICENSE.txt for details

"""
Racing subject object
"""

from math import atan, degrees

class DRacer():
    def __init__(self, _sl, _sf, _m, _ts, _sp):

        self.speed_profile = _sp
        self.speed_profile_len = len(self.speed_profile)
        self.speed = self.speed_profile[0]
        self.stride_length = float(_sl)
        self.stride_frequency = float(_sf)
        self.stride_duration = 1/self.stride_frequency
        self.mass = float(_m)
        self.weight = self.mass * 9.81
        self.distance = 0.0
        self.total_distance = 0.0
        self.stride_no = 0
        self.time = 0.0
        self.path_segments = _ts
        self.current_path_segment = 0
        self.previous_turning_radius = 0.0
        self.turning_radius = 0.0
        self.turning_curvature = 0.0
        self.centrifugal_acceleration = 0.0
        self.centrifugal_force = 0.0
        self.jerk = 0.0
        self.lean_to_horizon = 0.0 # relative leaning to horizon
        self.lean_to_track = 0.0 # relative leaning to track surface
        self.track_camber = None

    def next_stride(self):

        end_of_track = False

        # calculation of racer location in the track

        self.distance += self.stride_length
        self.total_distance += self.stride_length
        self.stride_no += 1
        self.time += self.stride_duration

        if self.distance > self.path_segments[self.current_path_segment].length:
            self.distance = self.distance - self.path_segments[self.current_path_segment].length
            if self.current_path_segment == (len(self.path_segments) - 1):
                end_of_track = True
            else:
                self.current_path_segment += 1

        # calculations of racer dynamic states

        if self.stride_no > self.speed_profile_len:
            self.speed = self.speed_profile[-1]
        else:
            self.speed = self.speed_profile[self.stride_no - 1]

        self.previous_turning_radius = self.turning_radius

        if self.path_segments[self.current_path_segment].curvature_rate < 0:
            self.turning_curvature = self.path_segments[self.current_path_segment].curvature_from + (self.path_segments[self.current_path_segment].curvature_rate * self.distance)
            self.turning_radius = 1 / self.turning_curvature
        else:
            self.turning_curvature = self.path_segments[self.current_path_segment].curvature_from + (self.path_segments[self.current_path_segment].curvature_rate * self.distance)
            self.turning_radius = 1 / self.turning_curvature

        self.centrifugal_acceleration = (self.speed**2)/self.turning_radius
        self.centrifugal_force = self.mass * self.centrifugal_acceleration

        if self.stride_no > 1:

            self.jerk = (self.centrifugal_acceleration - ((self.speed**2)/self.previous_turning_radius))/self.stride_duration

        if self.path_segments[self.current_path_segment].camber_rate < 0:
            self.track_camber = self.path_segments[self.current_path_segment].camber_from + (self.path_segments[self.current_path_segment].camber_rate * self.distance)
        else:
            self.track_camber = self.path_segments[self.current_path_segment].camber_from + self.path_segments[self.current_path_segment].camber_rate * self.distance

        if self.centrifugal_force == 0:
            self.lean_to_horizon = 0.0
        else:
            self.lean_to_horizon = degrees(atan(self.weight / self.centrifugal_force))

        self.lean_to_track = self.lean_to_horizon + degrees(atan(self.track_camber/100))

        return end_of_track

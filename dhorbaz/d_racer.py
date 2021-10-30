# Copyright (c) 2019-2021, Md Imam Hossain (emamhd at gmail dot com)
# see LICENSE.txt for details

"""
Racing subject object
"""

from math import atan, degrees

class DRacer():
    def __init__(self, _sl, _sf, _m, _ts, _sp):

        self.speed_profile = _sp
        self.speed_profile_len = len(self.speed_profile)
        self.speed = float(self.speed_profile[0])
        self.stride_length = float(_sl)
        self.stride_frequency = float(_sf)
        self.stride_duration = float(1/self.stride_frequency)
        self.mass = float(_m)
        self.weight = float(self.mass * 9.81)
        self.distance = 0.0
        self.distance_travelled = 0.0
        self.stride_no = 0
        self.time = 0.0
        self.path_segments = _ts
        self.path_segments_len = len(self.path_segments)
        self.path_segment_cursor = 0
        self.turning_radius = 0.0
        self.turning_curvature = 0.0
        self.yaw_rate = 0.0
        self.total_yaw = 0.0
        self.__previous_centrifugal_acceleration = 0.0
        self.centrifugal_acceleration = 0.0
        self.centrifugal_force = 0.0
        self.__previous_jerk = 0.0
        self.jerk = 0.0
        self.snap = 0.0
        self.lean_to_horizon = 0.0 # leaning relative to horizon
        self.lean_to_track = 0.0 # leaning relative to track surface
        self.track_camber = None

    def next_stride(self):

        end_of_track = False

        # calculation of racer location in the track

        self.distance += self.stride_length
        self.distance_travelled += self.stride_length
        self.stride_no += 1
        self.time += self.stride_duration

        if self.distance > self.path_segments[self.path_segment_cursor].length:
            self.distance -= self.path_segments[self.path_segment_cursor].length
            if self.path_segment_cursor == (self.path_segments_len - 1):
                end_of_track = True
            else:
                self.path_segment_cursor += 1

        # calculations of racer dynamic states

        if self.stride_no > self.speed_profile_len:
            self.speed = self.speed_profile[-1]
        else:
            self.speed = self.speed_profile[self.stride_no - 1]

        self.__previous_centrifugal_acceleration = self.centrifugal_acceleration
        self.__previous_jerk = self.jerk

        self.turning_curvature = self.path_segments[self.path_segment_cursor].curvature_from + (self.path_segments[self.path_segment_cursor].curvature_rate * self.distance)
        self.turning_radius = float(1 / self.turning_curvature)
        self.yaw_rate = float(self.speed / self.turning_radius)

        self.total_yaw += abs(self.yaw_rate * self.stride_duration)

        self.centrifugal_acceleration = float((self.speed**2)/self.turning_radius)
        self.centrifugal_force = float(self.mass * self.centrifugal_acceleration)

        if self.stride_no > 1:

            self.jerk = float((self.centrifugal_acceleration - self.__previous_centrifugal_acceleration)/self.stride_duration)

        if self.stride_no > 2:

            self.snap = float((self.jerk - self.__previous_jerk) / self.stride_duration)

        self.track_camber = self.path_segments[self.path_segment_cursor].camber_from + (self.path_segments[self.path_segment_cursor].camber_rate * self.distance)

        if self.centrifugal_force == 0.0:
            self.lean_to_horizon = 0.0
        else:
            self.lean_to_horizon = float(degrees(atan(self.weight / self.centrifugal_force)))

        self.lean_to_track = float(self.lean_to_horizon + degrees(atan(self.track_camber/100)))

        return end_of_track

# Copyright (c) 2019-2023, Md Imam Hossain (emamhd at gmail dot com)
# see LICENSE.txt for details

"""
Racing subject object
"""

from math import atan, degrees

EARTH_GRAVITATIONAL_ACCELERATION = 9.80665

class DRacer():
    def __init__(self, _sv, _m, _ts):

        self.stride_variables = _sv
        self.stride_variables_len = len(self.stride_variables)
        self.stride_frequency = 0.0
        self.stride_length = 0.0
        self.instantaneous_speed = 0.0
        self.stride_duration = 0.0
        self.mass = float(_m)
        self.weight = float(self.mass * EARTH_GRAVITATIONAL_ACCELERATION)
        self.distance = 0.0
        self.distance_travelled = 0.0
        self.stride_no = 0
        self.time = 0.0
        self.path_segments = _ts
        self.path_segments_len = len(self.path_segments)
        self.path_segment_cursor = 0
        self.turning_radius = 0.0
        self.turning_curvature = None
        self.yaw_rate = 0.0
        self.total_yaw = 0.0
        self.__previous_centrifugal_acceleration = 0.0
        self.centrifugal_acceleration = 0.0
        self.centrifugal_force = 0.0
        self.__previous_jerk = 0.0
        self.jerk = 0.0
        self.snap = 0.0
        self.lean_to_horizon = 0.0  # leaning relative to horizon
        self.lean_to_track = 0.0  # leaning relative to track surface
        self.__previous_track_camber = None
        self.track_camber = None
        self.track_camber_rate = 0.0
        self.__previous_track_elevation = None
        self.track_elevation = None
        self.track_elevation_rate = 0.0
        self.track_elevation_camber = 0.0
        self.track_width = None
        self.__previous_track_camber_elevation = None
        self.track_camber_elevation = None
        self.track_camber_elevation_rate = 0.0
        self.track_path_offset = None

    def next_stride(self):

        end_of_track = False

        self.stride_no += 1

        if self.stride_no > self.stride_variables_len:
            self.stride_frequency = float(self.stride_variables[-1][0])
            self.stride_length = float(self.stride_variables[-1][1])
        else:
            self.stride_frequency = float(self.stride_variables[self.stride_no - 1][0])
            self.stride_length = float(self.stride_variables[self.stride_no - 1][1])

        self.stride_duration = float(1 / self.stride_frequency)

        self.instantaneous_speed = self.stride_frequency * self.stride_length

        # calculation of racer location in the track

        self.distance += self.stride_length
        self.distance_travelled += self.stride_length
        self.time += self.stride_duration

        if self.distance > self.path_segments[self.path_segment_cursor].length:
            self.distance -= self.path_segments[self.path_segment_cursor].length
            if self.path_segment_cursor == (self.path_segments_len - 1):
                end_of_track = True
            else:
                self.path_segment_cursor += 1

        # calculations of racer dynamic states

        self.__previous_centrifugal_acceleration = self.centrifugal_acceleration
        self.__previous_jerk = self.jerk

        self.turning_curvature = self.path_segments[self.path_segment_cursor].curvature_from + (self.path_segments[self.path_segment_cursor].curvature_rate * self.distance)
        self.yaw_rate = float(self.instantaneous_speed * self.turning_curvature)
        self.turning_radius = abs(float(1 / self.turning_curvature))

        self.total_yaw += abs(self.yaw_rate * self.stride_duration)

        self.centrifugal_acceleration = float((self.instantaneous_speed**2)*self.turning_curvature)
        self.centrifugal_force = float(self.mass * self.centrifugal_acceleration)

        if self.stride_no > 1:

            self.jerk = float((self.centrifugal_acceleration - self.__previous_centrifugal_acceleration)/self.stride_duration)

        if self.stride_no > 2:

            self.snap = float((self.jerk - self.__previous_jerk) / self.stride_duration)

        self.__previous_track_camber = self.track_camber

        self.track_camber = self.path_segments[self.path_segment_cursor].camber_from + (self.path_segments[self.path_segment_cursor].camber_rate * self.distance)

        if (self.__previous_track_camber is not None) and (self.track_camber is not None):

            self.track_camber_rate = self.track_camber - self.__previous_track_camber

        if self.centrifugal_force == 0.0:
            self.lean_to_horizon = 0.0
        else:
            self.lean_to_horizon = float(degrees(atan(self.weight / self.centrifugal_force)))

        self.lean_to_track = float(self.lean_to_horizon + degrees(atan(self.track_camber/100)))

        self.__previous_track_elevation = self.track_elevation

        self.track_elevation = self.path_segments[self.path_segment_cursor].elevation_from + (self.path_segments[self.path_segment_cursor].elevation_rate * self.distance)

        if (self.__previous_track_elevation is not None) and (self.track_elevation is not None):

            self.track_elevation_rate = self.track_elevation - self.__previous_track_elevation

        self.track_path_offset = self.path_segments[self.path_segment_cursor].path_offset_from + (self.path_segments[self.path_segment_cursor].path_offset_rate * self.distance)

        self.__previous_track_camber_elevation = self.track_camber_elevation

        self.track_camber_elevation = self.track_elevation + ((self.track_camber / 100)*self.track_path_offset)

        if (self.__previous_track_camber_elevation is not None) and (self.track_camber_elevation is not None):

            self.track_elevation_camber = ((self.track_camber_elevation - self.__previous_track_camber_elevation)/self.stride_length)*100
            self.track_camber_elevation_rate = self.track_camber_elevation - self.__previous_track_camber_elevation

        self.track_width = self.path_segments[self.path_segment_cursor].width_from + (self.path_segments[self.path_segment_cursor].width_rate * self.distance)

        return end_of_track

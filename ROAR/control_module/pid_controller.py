from pydantic import BaseModel, Field
from ROAR.control_module.controller import Controller
from ROAR.utilities_module.vehicle_models import VehicleControl, Vehicle

from ROAR.utilities_module.data_structures_models import Transform, Location
from collections import deque
import numpy as np
import math
import logging
from ROAR.agent_module.agent import Agent
from typing import Tuple
import json
from pathlib import Path

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

class PIDController(Controller):
    def __init__(self, agent, steering_boundary: Tuple[float, float],
                 throttle_boundary: Tuple[float, float], **kwargs):
        super().__init__(agent, **kwargs)
        self.max_speed = self.agent.agent_settings.max_speed
        self.throttle_boundary = throttle_boundary
        self.steering_boundary = steering_boundary
        self.old_pitch = 0
        self.pitch_difference = 0
        self.config = json.load(Path(agent.agent_settings.pid_config_file_path).open(mode='r'))
        self.lat_pid_controller = LatPIDController(
            agent=agent,
            config=self.config["latitudinal_controller"],
            steering_boundary=steering_boundary
        )
        self.logger = logging.getLogger(__name__)

    def run_in_series(self, next_waypoint: Transform, close_waypoint: Transform, far_waypoint: Transform, **kwargs) -> VehicleControl:
        steering, error, wide_error, sharp_error, p = self.lat_pid_controller.run_in_series(next_waypoint=next_waypoint, close_waypoint=close_waypoint, far_waypoint=far_waypoint)

        error = abs(round(error, 3))
        wide_error = abs(round(wide_error, 3))
        sharp_error = abs(round(sharp_error, 3))
        pitch = float(next_waypoint.record().split(",")[4])

        self.pitch_difference = pitch - self.old_pitch
        self.old_pitch = pitch

        speed = Vehicle.get_speed(self.agent.vehicle)
        gear = math.ceil((speed - (2*pitch)) / 60)

        #Zoomies:tm:
        brake = 0
        throttle = 1

        #Crash logging
        f = open("C:/Users/aaron/Downloads/S1/ROAR/logs.txt", "a")
        f.write("\nCurrent Location: " + str(p) + "\nNext Waypoint: " + str(next_waypoint) + "\n\n")
        #Hard speed limit
        if speed > 160:
            f.write("Current Action: Speed control")
            throttle = 0.7

        #Steering control
        if sharp_error > 0.6 and speed > 80:
            f.write("Obstacle Encountered: Sharp turn --> Decelerating & Stopping")
            throttle = -1
            brake = 1
        elif abs(steering) > 0.275 and speed > 60:
            f.write("Obstacle Encountered: Moderate Curve --> Slowing Down")
            throttle = 0.3
        elif abs(steering) > 0.2 and speed > 75:
            f.write("Obstacle Encountered: Large Curve --> Slowing Down")
            throttle = 0.3

        if self.pitch_difference < -1.5:
            f.write("Obstacle Encountered: Bump --> Decelerating & Stopping")
            throttle = -1
            brake = 1

        #Special crash prevention
        w = str(next_waypoint)
        i = w.split("x: ")
        i = i[1].split(",")
        nextX = int(float(i[0]))
        i = i[1].split("y: ")
        nextY = int(float(i[1]))
        #Funky collisions in the mountains
        if 3061 <= nextX <= 3085 and 155 <= nextY <= 157:
            brake = 1
            throttle = -1
        #Border is in the road(?)
        if 3130 <= nextX <= 3150 and 344 <= nextY <= 345:
            brake = 1
            throttle = -1

        #Gear shifting (Totally not stolen)
        if gear == 0:
            gear += 1
        f.close()
        return VehicleControl(throttle=throttle, steering=steering, brake=brake, manual_gear_shift=True, gear=gear)

    @staticmethod
    def find_k_values(vehicle: Vehicle, config: dict) -> np.array:
        current_speed = Vehicle.get_speed(vehicle=vehicle)
        k_p, k_d, k_i = 1, 0, 0
        for speed_upper_bound, kvalues in config.items():
            speed_upper_bound = float(speed_upper_bound)
            if current_speed < speed_upper_bound:
                k_p, k_d, k_i = kvalues["Kp"], kvalues["Kd"], kvalues["Ki"]
                break
        return np.array([k_p, k_d, k_i])


class LatPIDController(Controller):
    def __init__(self, agent, config: dict, steering_boundary: Tuple[float, float],
                 dt: float = 0.03, **kwargs):
        super().__init__(agent, **kwargs)
        self.config = config
        self.steering_boundary = steering_boundary
        self._error_buffer = deque(maxlen=10)
        self._dt = dt

    def run_in_series(self, next_waypoint: Transform, close_waypoint: Transform, far_waypoint: Transform, **kwargs) -> float:
        """
        Calculates a vector that represent where you are going.
        Args:
            next_waypoint ():
            **kwargs ():

        Returns:
            lat_control
        """
        # calculate a vector that represent where you are going
        v_begin = self.agent.vehicle.transform.location.to_array()
        direction_vector = np.array([-np.sin(np.deg2rad(self.agent.vehicle.transform.rotation.yaw)),
                                     0,
                                     -np.cos(np.deg2rad(self.agent.vehicle.transform.rotation.yaw))])
        v_end = v_begin + direction_vector

        v_vec = np.array([(v_end[0] - v_begin[0]), 0, (v_end[2] - v_begin[2])])
        # calculate error projection
        w_vec = np.array(
            [
                next_waypoint.location.x - v_begin[0],
                0,
                next_waypoint.location.z - v_begin[2],
            ]
        )

        v_vec_normed = v_vec / np.linalg.norm(v_vec)
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        error = np.arccos(v_vec_normed @ w_vec_normed.T)
        _cross = np.cross(v_vec_normed, w_vec_normed)

        # calculate close error projection
        w_vec = np.array(
            [
                close_waypoint.location.x - v_begin[0],
                0,
                close_waypoint.location.z - v_begin[2],
            ]
        )
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        # wide_error = np.arccos(v_vec_normed @ w_vec_normed.T)
        wide_error = np.arccos(
            min(max(v_vec_normed @ w_vec_normed.T, -1), 1))  # makes sure arccos input is between -1 and 1, inclusive

        # calculate far error projection
        w_vec = np.array(
            [
                far_waypoint.location.x - v_begin[0],
                0,
                far_waypoint.location.z - v_begin[2],
            ]
        )
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        # sharp_error = np.arccos(v_vec_normed @ w_vec_normed.T)
        sharp_error = np.arccos(
            min(max(v_vec_normed @ w_vec_normed.T, -1), 1))  # makes sure arccos input is between -1 and 1, inclusive

        if _cross[1] > 0:
            error *= -1
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        k_p, k_d, k_i = PIDController.find_k_values(config=self.config, vehicle=self.agent.vehicle)

        lat_control = float(
            np.clip((k_p * error) + (k_d * _de) + (k_i * _ie), self.steering_boundary[0], self.steering_boundary[1])
        )

        testPosition = self.agent.vehicle.transform.location.to_array()
        return lat_control, error, wide_error, sharp_error, testPosition

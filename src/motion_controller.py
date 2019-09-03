#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#
# Currently only support Python 3.x

import threading
import copy
import time
import queue as Queue

from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory


class MotionController:
    def __init__(self, initial_joint_state, robot_servo=None, update_rate=100, buff_size=0):
        # Class lock
        self.lock = threading.Lock()

        # Motion loop update rate (higher update rates result in smoother simulated motion)
        self.update_rate = update_rate
        if self.update_rate != 0.:
            self.update_duration = 1/update_rate

        # Initialize joint position
        self.joint_positions = initial_joint_state

        self.robot_servo = robot_servo

        # Initialize motion buffer (contains joint position lists)
        self.motion_buffer = Queue.Queue(buff_size)     # default buff_size=0 for infinite size

        # Signals
        self.sig_shutdown = False
        self.sig_stop = False

        # Motion thread
        self.motion_thread = threading.Thread(target=self._motion_worker)
        self.motion_thread.daemon = True
        self.motion_thread.start()

    def add_motion_waypoint(self, point):
        self.motion_buffer.put(point)

    def get_joint_position(self):
        return self.joint_positions[:]

    def is_in_motion(self):
        return not self.motion_buffer.empty()

    def shutdown(self):
        self.sig_shutdown = True

    def stop(self):
        with self.lock:
            self._clear_buffer()
            self.sig_stop = True

    def _clear_buffer(self):
        with self.motion_buffer.mutex:
            self.motion_buffer.queue.clear()

    def interpolate(self, start_point, goal_point, alpha):
        """
        Creating intermediate point by linear interpolation.
        Linear interpolation:
            from start point (x',t') to goal point (x,t), create intermediate point (xi,ti)
            using linear gradiation alpha: a
            xi = x' + a (x - x')
            ti = t' + a (t - t')
        :param start_point: starting point or last goal point
        :param goal_point: cureent goal point
        :param alpha:
        :return:
        """
        intermediate_point = JointTrajectoryPoint()

        for start_pos, goal_pos in zip(start_point.positions, goal_point.positions):
            intermediate_point.positions.append(start_pos + alpha*(goal_pos - start_pos))
        intermediate_point.time_from_start = goal_point.time_from_start + \
                                             alpha*(goal_point.time_from_start.to_sec() - start_point.time_from_start.to_sec())
        return intermediate_point

    def _move_to(self, goal_point, goal_duration):
        time.sleep(goal_duration)

        with self.lock:
            # Move the motors
            if not self.sig_stop:
                for i in range(len(self.joint_positions)):
                    self.robot_servo[i].setAngle(goal_point.positions[i])

                # Update current joint positions, since there is no sensors
                self.joint_positions = goal_point.positions[:]
            else:
                print('Stopping motion immediately, clearing stop signal')
                self.sig_stop = False

    def _motion_worker(self):

        last_goal_point = JointTrajectoryPoint()
        with self.lock:
            try:
                last_goal_point = self.joint_positions
                current_goal_point = self.motion_buffer.get()

                # if current new goal time is less than last goal time, it is a new trajectory. just copy it
                if current_goal_point.time_from_start < last_goal_point.time_from_start:
                    goal_duration = current_goal_point.time_from_start
                else:
                    goal_duration = current_goal_point.time_from_start - last_goal_point.time_from_start
                    # if motion update duration is less than trajectory duration, it is possible to interpolate
                    if self.update_duration > 0:
                        trajectory_duration = goal_duration.to_sec()

                        # move
                        while self.update_duration < goal_duration:
                            intermediate_point = self.interpolate(last_goal_point, current_goal_point,
                                                                       self.update_duration.to_sec()/goal_duration.to_sec())

                            # Move to intermediate point
                            self._move_to(intermediate_point, self.update_duration.to_sec())

                            # Update the last goal point to current point
                            last_goal_point = copy.deepcopy(intermediate_point)
                            goal_duration = current_goal_point.time_from_start - intermediate_point.time_from_start

                self._move_to(current_goal_point, goal_duration)
                last_goal_point = copy.deepcopy(current_goal_point)
            except Exception as e:
                pass


class RobotJoint:
    def __init__(self, robot_servo=None):
        self.robot_servo = robot_servo
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        initial_joint_state = [0]*len(self.joint_names)
        self.motion_control = MotionController(initial_joint_state, robot_servo=self.robot_servo)

    def

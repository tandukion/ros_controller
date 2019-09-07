#!/usr/bin/env python
#
# Copyright (c) 2019, Dwindra Sulistyoutomo
#
# Currently only support Python 3.x

import threading
import copy
import time
import queue as Queue
from math import degrees

# from trajectory_msgs.msg import JointTrajectoryPoint


class JointTrajectoryPt(object):
    """
    Class that handles Joint Trajectory Point.
    This class replace the JointTrajectoryPoint class from trajectory_msgs ROS package
    """
    def __init__(self, joint_num):
        """
        :param joint_num: number of Robot joints
        """
        self.positions = []
        for i in range(joint_num):
            self.positions.append(None)
        self.velocities = 0
        self.accelerations = 0
        self.effort = 0
        self.duration = 0

    def set_positions(self, pos):
        self.positions = pos[:]

    def set_velocities(self, vel):
        self.velocities = vel

    def set_accelerations(self, acc):
        self.accelerations = acc

    def set_effort(self, effort):
        self.effort = effort

    def set_duration(self, duration):
        self.duration = duration


class MotionController:
    """
    Class that controls the robot motion.
    Trajectory points use JointTrajectoryPt class.
    """
    def __init__(self, initial_joint_pos, robot_servo=None, update_rate=100, buff_size=0):
        """
        :param initial_joint_pos: initial joint positions after initialization
        :type  initial_joint_pos: list of float
        :param robot_servo: Robot servos
        :type  robot_servo: list of RobotServo
        :param update_rate: moving rate (Hz)
        :param buff_size: max size for motion buffer
        """
        # Class lock
        from math import degrees, radians
        self.lock = threading.Lock()

        # Motion loop update rate (higher update rates result in smoother simulated motion)
        self.update_rate = update_rate
        if self.update_rate != 0.:
            self.update_duration = 1/update_rate

        # Initialize Joint Positions
        self.joint_positions = initial_joint_pos

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

    def trigger_new_trajectory(self):
        self.new_trajectory = True

    def add_motion_waypoint(self, point_message):
        """
        Adding goal point to motion buffer
        :param point_message: goal point received from TrajectoryPoint message
        :type  point_message: JointTrajectoryPt
        """
        set_angle = list(map(degrees, point_message.data[:10]))
        way_point = JointTrajectoryPt(len(set_angle))
        way_point.set_positions(set_angle)
        way_point.set_velocities(point_message.data[10])
        way_point.set_duration(point_message.data[11])

        print_str = 'WAYPOINT: '
        for d in range(len(way_point.positions)):
            print_str += "%d, " % way_point.positions[d]
        print_str += "%.2f, " % way_point.velocities
        print_str += "%.2f, " % way_point.duration
        print(print_str)
        self.motion_buffer.put(way_point)

    def get_joint_position(self):
        joint_pos = self.joint_positions[:]
        return joint_pos

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
            using linear gradient alpha: a
            xi = x' + a (x - x')
            ti = t' + a (t - t')
        :param start_point: starting point or last goal point
        :param goal_point: cureent goal point
        :param alpha:
        :return: interpolation point
        """
        # print('Interpolating')
        inter_pt = JointTrajectoryPt(len(self.joint_positions))

        for i in range(len(self.joint_positions)):
            inter_pt.positions[i] = start_point.positions[i] + alpha*(goal_point.positions[i] - start_point.positions[i])
        inter_pt.duration = alpha*(goal_point.duration - start_point.duration)
        return inter_pt

    def _move_to(self, goal_point, move_duration):
        """
        Moves robot servos and updating Joint Positions
        :param goal_point: target goal point
        :type  goal_point: JointTrajectoryPt
        :param move_duration: duration to move to target goal point
        :type  move_duration: JointTrajectoryPt
        """
        if move_duration >= 0:
            time.sleep(move_duration)

        with self.lock:
            # Move the motors
            if not self.sig_stop:
                for i in range(len(self.joint_positions)):
                    self.robot_servo[i].setAngle(goal_point.positions[i])
                    self.joint_positions[i] = goal_point.positions[i]

                # Update current joint positions, since there is no sensors
                # self.joint_positions = goal_point.positions[:]
            else:
                print('Stopping motion immediately, clearing stop signal')
                self.sig_stop = False

    def _motion_worker(self):
        """
        Main handler thread to move the robot after getting the goal point from buffer
        """
        last_goal_point = JointTrajectoryPt(len(self.joint_positions))
        goal_duration = 0

        while not self.sig_shutdown:
            try:
                last_goal_point.set_positions(self.joint_positions)
                current_goal_point = self.motion_buffer.get()

                # if we set update rate/duration
                if self.update_duration > 0:
                    goal_duration = current_goal_point.duration

                    # move during goal duration
                    while self.update_duration < goal_duration:
                        intermediate_point = self.interpolate(last_goal_point, current_goal_point, self.update_duration/goal_duration)

                        # print_str = 'INTER: '
                        # for d in range(len(intermediate_point.positions)):
                        #     print_str += "%d, " % intermediate_point.positions[d]
                        # print_str += "%.2f, " % intermediate_point.velocities
                        # print_str += "%.2f, " % intermediate_point.duration
                        # print(print_str)

                        # Move to intermediate point
                        self._move_to(intermediate_point, intermediate_point.duration)

                        # Update the last goal point and goal duration
                        last_goal_point = copy.deepcopy(intermediate_point)
                        goal_duration -= intermediate_point.duration

                # Handling the last point when goal duration already <= 0
                self._move_to(current_goal_point, goal_duration)

            except Exception as e:
                pass

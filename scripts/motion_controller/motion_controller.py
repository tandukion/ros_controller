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

MIN_VELOCITY = 0.05
MIN_DURATION = 0.1


class JointTrajectoryPt(object):
    """
    Class that handles Joint Trajectory Point (trajectory_msgs/JointTrajectoryPoint).
    This class replace the JointTrajectoryPoint class from trajectory_msgs ROS package
    """
    def __init__(self, joint_num):
        """
        :param joint_num: number of Robot joints
        """
        self.positions = []
        for i in range(joint_num):
            self.positions.append(None)

        # Trajectory Point
        self.velocity = 0
        self.duration = 0

        # Trajectory Point Full
        self.velocities = []
        self.accelerations = []
        self.effort = 0

    def set_positions(self, pos):
        self.positions = pos[:]

    def set_velocity(self, vel):
        self.velocity = vel

    def set_velocities(self, vel_list):
        self.velocities = copy.deepcopy(vel_list)

    def set_accelerations(self, acc_list):
        self.accelerations = copy.deepcopy(acc_list)

    def set_effort(self, effort):
        self.effort = effort

    def set_duration(self, duration):
        self.duration = duration


class MotionController:
    """
    Class that controls the robot motion.
    Trajectory points use JointTrajectoryPt class.
    """
    def __init__(self, initial_joint_pos, home_pos, robot_servo=None, update_rate=100, buff_size=0):
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
        self.joint_velocity = 0
        self.home_position = home_pos

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

        # Trajectory Point
        if point_message.velocity:
            way_point.set_velocity(point_message.velocity)
        if point_message.duration:
            way_point.set_duration(point_message.duration)

        # Trajectory Point Full
        if point_message.velocities:
            # FIXME: For now only add the first
            way_point.set_velocity(point_message.velocities[0])
            if way_point.velocity < MIN_VELOCITY:
                way_point.set_velocity(MIN_VELOCITY)
        if point_message.time:
            way_point.set_duration(point_message.time)
            if way_point.duration < MIN_DURATION:
                way_point.set_duration(MIN_DURATION)

        print_str = 'WAYPOINT: '
        print_str += "["
        for d in range(len(way_point.positions)):
            print_str += "%d" % way_point.positions[d]
            if d+1 < len(way_point.positions):
                print_str += ", "
            else:
                print_str += "] "
        print_str += "[%.2f, " % way_point.velocity
        print_str += "%.2f]" % way_point.duration
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

    def cubic_interpolate(self, start_point, goal_point, time_inter, delta_time):
        """
        Use Cubic Hermite spline for cubic interpolation https://en.wikipedia.org/wiki/Cubic_Hermite_spline
            h1(t) =  2t^3 - 3t^2 - 1
            h2(t) = -2t^3 + 3t^2
            h3(t) =   t^3 - 2t^2 + t
            h4(t) =   t^3 -  t^2
        with initial x at t=0 (x',v',t') and last x at t=1 (x,v,t),
        positon at interpolation xi:
            xi = h1.x' + h2.x + h3.v' + h4.v
        equation (after derivated):
            xi = x' + a1.ti + a2.ti^2 + a3.ti^3
            a1 =    v'/dt
            a2 =  3(dx/dt^2) - (dv+v')/dt^2
            a3 = -2(dx/dt^3) +      dv/dt^3
            with  dt = t - t'
                  dx = x - x'
                  dv = v + v'
        """
        # print('Cubic Interpolation')
        inter_pt = JointTrajectoryPt(len(self.joint_positions))
        for i in range(len(self.joint_positions)):
            dt = delta_time
            dx = goal_point.positions[i] - start_point.positions[i]
            dv = goal_point.velocity + start_point.velocity
            a1 = start_point.velocity / dt
            a2 =  3*(dx/pow(dt,2)) - (dv + start_point.velocity)/pow(dt,2)
            a3 = -2*(dx/pow(dt,3)) + dv/pow(dt,3)

            xi = start_point.positions + a1*time_inter + a2*pow(time_inter,2) + a3*pow(time_inter,3)
            inter_pt.positions[i] = xi
        inter_pt.duration = time_inter
        inter_pt.velocity = goal_point.velocity
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
                    if self.robot_servo:
                        self.robot_servo[i].setAngle(goal_point.positions[i])
                    self.joint_positions[i] = goal_point.positions[i]
                self.joint_velocity = goal_point.velocity

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
                last_goal_point.set_velocity(self.joint_velocity)
                current_goal_point = self.motion_buffer.get()

                # Updating with current joint positions
                with self.lock:
                    last_goal_point.set_positions(self.joint_positions)

                # if we set update rate/duration
                if self.update_duration > 0:
                    goal_duration = current_goal_point.duration

                    # move during goal duration
                    while self.update_duration < goal_duration:

                        # Do liner interpolation if no velocity
                        # if last_goal_point.velocity == 0 or current_goal_point.velocity == 0:
                        intermediate_point = self.interpolate(last_goal_point, current_goal_point, self.update_duration/goal_duration)
                        #     print("ALPHA %f" % (self.update_duration/goal_duration))
                        #
                        # # or do cubic interpolation
                        # else:
                        #     intermediate_point = self.cubic_interpolate(last_goal_point, current_goal_point, self.update_duration, goal_duration)

                        # print_str = 'INTER: '
                        # for d in range(len(intermediate_point.positions)):
                        #     print_str += "%d, " % intermediate_point.positions[d]
                        # print_str += "%.2f, " % intermediate_point.velocity
                        # print_str += "%.2f, " % intermediate_point.duration
                        # print(print_str)

                        # Move to intermediate point
                        self._move_to(intermediate_point, intermediate_point.duration)

                        # Update the last goal point and goal duration
                        last_goal_point = copy.deepcopy(intermediate_point)
                        goal_duration -= intermediate_point.duration
                        # print("Update duration %f" % intermediate_point.duration)
                        # print("Move duration %f" % goal_duration)

                # Handling the last point when goal duration already <= 0
                self._move_to(current_goal_point, goal_duration)
                # print("Move duration %f" % goal_duration)

            except Exception as e:
                pass

    def move_to_home(self):
        """
        Moves the robot to home position
        """
        home_pt = JointTrajectoryPt(len(self.home_position))
        home_pt.set_positions(self.home_position)
        home_pt.set_velocity(0)
        home_pt.set_duration(0)
        self._move_to(home_pt, home_pt.duration)

    def move_to_home_from_uninitialized(self):
        """
        Moves the robot to home position from uninitialized.
        Need to move the robot servo one by one for safety
        """
        home_pt = JointTrajectoryPt(len(self.home_position))
        home_pt.set_positions(self.joint_positions)
        home_pt.set_duration(0.5)
        for i in range(len(self.home_position)):
            home_pt.positions[i] = self.home_position[i]
            self._move_to(home_pt, home_pt.duration)

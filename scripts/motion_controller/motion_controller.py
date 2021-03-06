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
        self.duration = 0  # time_from_start

        # Trajectory Point Full
        self.velocities = [0.0] * joint_num
        self.accelerations = [0.0] * joint_num
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
    def __init__(self, initial_joint_pos, home_pos, robot_servo=None, duration_limit=1, buff_size=0):
        """
        :param initial_joint_pos: initial joint positions after initialization
        :type  initial_joint_pos: list of float
        :param robot_servo: Robot servos
        :type  robot_servo: list of RobotServo
        :param duration_limit: maximum motion duration for a single way point (in seconds).
        :param duration_limit: float
        :param buff_size: max size for motion buffer
        """
        # Class lock
        from math import degrees, radians
        self.lock = threading.Lock()

        # Motion duration limit. Any motion with duration bigger than duration_limit will trigger interpolation.
        # FIXME: large duration limit is good for simulation, but need to confirm with real robot
        self.duration_limit = duration_limit

        # Time flag for motion
        self.start_move_time = 0

        # Initialize Joint Positions
        self.joint_positions = initial_joint_pos
        self.joint_velocities = [0.0] * len(initial_joint_pos)
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

        # Set velocities
        # Trajectory Point Full
        if point_message.velocities:
            way_point.set_velocities(point_message.velocities)
        # Trajectory Point
        elif point_message.velocity:
            # Duplicate the velocity for all joints
            way_point.set_velocities([point_message.velocity] * len(way_point.positions))

        # Set Duration
        # Trajectory Point
        if point_message.duration:
            way_point.set_duration(point_message.duration)

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
        print_str += "["
        for v in range(len(way_point.velocities)):
            print_str += "%.2f" % way_point.velocities[v]
            if v+1 < len(way_point.velocities):
                print_str += ", "
            else:
                print_str += "] "
        print_str += "[%.2f]" % way_point.duration
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

            # Copy the velocities as it is
            inter_pt.velocities[i] = goal_point.velocities[i]

        inter_pt.duration = alpha*(goal_point.duration - start_point.duration)

        # print_str = 'INTER: '
        # print_str += "["
        # for d in range(len(inter_pt.positions)):
        #     print_str += "%d" % inter_pt.positions[d]
        #     if d+1 < len(inter_pt.positions):
        #         print_str += ", "
        #     else:
        #         print_str += "] "
        # print_str += "["
        # for v in range(len(inter_pt.velocities)):
        #     print_str += "%.2f" % inter_pt.velocities[v]
        #     if v+1 < len(inter_pt.velocities):
        #         print_str += ", "
        #     else:
        #         print_str += "] "
        # print_str += "[%.2f]" % inter_pt.duration
        # print(print_str)

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
            dv = goal_point.velocities[i] + start_point.velocities[i]
            a1 = start_point.velocities[i] / dt
            a2 =  3*(dx/pow(dt,2)) - (dv + start_point.velocities[i])/pow(dt,2)
            a3 = -2*(dx/pow(dt,3)) + dv/pow(dt,3)

            xi = start_point.positions[i] + a1*time_inter + a2*pow(time_inter,2) + a3*pow(time_inter,3)
            inter_pt.positions[i] = xi

            # Copy the velocities as it is
            inter_pt.velocities[i] = goal_point.velocities[i]

        inter_pt.duration = time_inter

        # print_str = 'INTER: '
        # print_str += "["
        # for d in range(len(inter_pt.positions)):
        #     print_str += "%d" % inter_pt.positions[d]
        #     if d+1 < len(inter_pt.positions):
        #         print_str += ", "
        #     else:
        #         print_str += "] "
        # print_str += "["
        # for v in range(len(inter_pt.velocities)):
        #     print_str += "%.2f" % inter_pt.velocities[v]
        #     if v+1 < len(inter_pt.velocities):
        #         print_str += ", "
        #     else:
        #         print_str += "] "
        # print_str += "[%.2f]" % inter_pt.duration
        # print(print_str)

        return inter_pt

    def _move_to(self, goal_point, move_duration):
        """
        Moves robot servos and updating Joint Positions
        :param goal_point: target goal point
        :type  goal_point: JointTrajectoryPt
        :param move_duration: duration to move to target goal point
        :type  move_duration: float
        """

        with self.lock:
            # Move the motors
            if not self.sig_stop:
                for i in range(len(self.joint_positions)):
                    if self.robot_servo:
                        self.robot_servo[i].setAngle(goal_point.positions[i])
                    self.joint_positions[i] = goal_point.positions[i]
                    self.joint_velocities[i] = goal_point.velocities[i]

                # Check move duration
                elapsed_time = time.time() - self.start_move_time
                if elapsed_time < move_duration:
                    # print("Sleep %.2f" % (move_duration-elapsed_time))
                    # time.sleep(move_duration-elapsed_time)
                    pass
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
                last_goal_point.set_velocities(self.joint_velocities)
                current_goal_point = self.motion_buffer.get()
                goal_duration = current_goal_point.duration

                self.start_move_time = time.time()

                # Updating with current joint positions
                with self.lock:
                    last_goal_point.set_positions(self.joint_positions)

                # if we set update rate/duration
                if self.duration_limit > 0:
                    # move during goal duration
                    # Do interpolation if goal_duration is larger than the move duration
                    while self.duration_limit < goal_duration:

                        # Do liner interpolation if no velocities
                        # if last_goal_point.velocities == 0 or current_goal_point.velocities == 0:
                        if 0.0 in last_goal_point.velocities or 0.0 in current_goal_point.velocities:
                            # print("Do simple interpolation")
                            intermediate_point = self.interpolate(last_goal_point, current_goal_point, self.duration_limit/goal_duration)
                        #     print("ALPHA %f" % (self.duration_limit/goal_duration))
                        #
                        # # or do cubic interpolation
                        else:
                            # print("Do cubic interpolation")
                            intermediate_point = self.cubic_interpolate(last_goal_point, current_goal_point, self.duration_limit, goal_duration)

                        # Move to intermediate point
                        self._move_to(intermediate_point, intermediate_point.duration)

                        # Update the last goal point and goal duration
                        last_goal_point = copy.deepcopy(intermediate_point)
                        goal_duration -= intermediate_point.duration
                        # print("Update duration %f" % intermediate_point.duration)
                        # print("Move duration %f" % goal_duration)

                # Handling the last point when goal duration already <= 0
                # or move directly without interpolating
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
        home_pt.set_duration(0)
        self.start_move_time = time.time()
        self._move_to(home_pt, home_pt.duration)

    def move_to_home_from_uninitialized(self):
        """
        Moves the robot to home position from uninitialized.
        Need to move the robot servo one by one for safety
        """
        home_pt = JointTrajectoryPt(len(self.home_position))
        home_pt.set_positions(self.joint_positions)
        home_pt.set_duration(0.5)
        self.start_move_time = time.time()
        for i in range(len(self.home_position)):
            home_pt.positions[i] = self.home_position[i]
            self._move_to(home_pt, home_pt.duration)

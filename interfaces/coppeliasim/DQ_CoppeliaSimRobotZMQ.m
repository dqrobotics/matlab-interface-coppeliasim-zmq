% (C) Copyright 2011-2025 DQ Robotics Developers
% 
% This file is part of DQ Robotics.
% 
%     DQ Robotics is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     DQ Robotics is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU Lesser General Public License for more details.
% 
%     You should have received a copy of the GNU Lesser General Public License
%     along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.
% 
% DQ Robotics website: dqrobotics.github.io
% 
% Contributors:
% 
%    1. Frederico Fernandes Afonso Silva (frederico.silva@ieee.org)
%         - Responsible for the original implementation

classdef DQ_CoppeliaSimRobotZMQ < DQ_CoppeliaSimRobot
    properties (SetAccess = protected)
        base_frame_name_;
        joint_names_;
    end

    methods
        function obj = DQ_CoppeliaSimRobotZMQ(robot_name, coppeliasim_interface)
            % This method constructs an instance of a
            % DQ_CoppeliaSimRobotZMQ object.
            % Usage:
            %     DQ_CoppeliaSimRobotZMQ(robot_name, coppeliasim_interface)
            %          robot_name: The name of the robot in the CoppeliaSim
            %          scene.
            %          coppeliasim_interface: The DQ_CoppeliaSimInterfaceZMQ
            %          object connected to the CoppeliaSim scene.
            %
            % Example: 
            %      cs = DQ_CoppeliaSimInterfaceZMQ();
            %      cs.connect;
            %      robot_coppeliasim = DQ_CoppeliaSimRobotZMQ("/LBR4p", cs);
            %     
            %    Note that the name of the robot should be EXACTLY the same
            %    as in CoppeliaSim. For instance, if you drag-and-drop a
            %    second robot to the scene, CoppeliaSim will rename the
            %    first robot to "/LBR4p[0]" and name the second one
            %    "/LBR4p[1]". A third robot will be named "/LBR4p[2]", and
            %    so on.

            arguments 
                robot_name (1,:) {mustBeText}
                coppeliasim_interface (1,1) DQ_CoppeliaSimInterfaceZMQ
            end

            obj@DQ_CoppeliaSimRobot;
            obj.robot_name_ = robot_name;
            obj.coppeliasim_interface_ = coppeliasim_interface;

            obj.joint_names_ = ...
                obj.coppeliasim_interface_.get_jointnames_from_object( ...
                    robot_name);
            obj.base_frame_name_ = obj.joint_names_{1};
        end

        function set_configuration(obj, q)
            % This method sets the joint configurations of the robot in the
            % CoppeliaSim scene.
            % Usage:
            %     set_configuration(q);  
            %          q: The joint configurations for the robot in the
            %          CoppeliaSim scene.
            %
            % Example: 
            %     cs = DQ_CoppeliaSimInterfaceZMQ();
            %     cs.connect;
            %     robot_coppeliasim = DQ_CoppeliaSimRobotZMQ("/LBR4p", cs);
            %     q = zeros(7, 1);
            %     robot_coppeliasim.set_configuration(q);
            %     
            %     Note that this calls "set_joint_positions" in
            %     DQ_CoppeliaSimInterfaceZMQ, meaning that it requires a
            %     dynamics disabled scene.

            arguments 
                obj (1,1) DQ_CoppeliaSimRobotZMQ
                q (1,:) {mustBeNumeric}
            end

            obj.coppeliasim_interface_.set_joint_positions( ...
                obj.joint_names_, q);
        end

        function set_target_configuration(obj, q)
            % This method sets the target joint configurations for the robot
            % in the CoppeliaSim scene.
            % Usage:
            %     set_target_configuration(q);  
            %          q: The target joint configurations for the robot in
            %          the CoppeliaSim scene.
            %
            % Example: 
            %     cs = DQ_CoppeliaSimInterfaceZMQ();
            %     cs.connect;
            %     robot_coppeliasim = DQ_CoppeliaSimRobotZMQ("/LBR4p", cs);
            %     q = zeros(7, 1);
            %     robot_coppeliasim.set_target_configuration(q);
            %     
            %     Note that this calls "set_joint_target_positions" in
            %     DQ_CoppeliaSimInterfaceZMQ, meaning that it requires a
            %     dynamics dynamics-enabled scene and joints in dynamic
            %     mode with position control mode.

            arguments 
                obj (1,1) DQ_CoppeliaSimRobotZMQ
                q (1,:) {mustBeNumeric}
            end

            obj.coppeliasim_interface_.set_joint_target_positions( ...
                obj.joint_names_, q);
        end

        function q = get_configuration(obj)
            % This method gets the joint configurations of the robot in the
            % CoppeliaSim scene.
            % Usage:
            %     get_configuration();  
            %
            % Example: 
            %     cs = DQ_CoppeliaSimInterfaceZMQ();
            %     cs.connect;
            %     robot_coppeliasim = DQ_CoppeliaSimRobotZMQ("/LBR4p", cs);
            %     q = robot_coppeliasim.get_configuration;

            arguments 
                obj (1,1) DQ_CoppeliaSimRobotZMQ
            end

            q = obj.coppeliasim_interface_.get_joint_positions( ...
                    obj.joint_names_);
        end

        function set_target_configuration_velocities(obj, dq)
            % This method sets the target joint configuration velocities
            % for the robot in the CoppeliaSim scene.
            % Usage:
            %     set_target_configuration_velocities(dq);  
            %          dq: The target joint configuration velocities for
            %          the robot in the CoppeliaSim scene.
            %
            % Example: 
            %     cs = DQ_CoppeliaSimInterfaceZMQ();
            %     cs.connect;
            %     robot_coppeliasim = DQ_CoppeliaSimRobotZMQ("/LBR4p", cs);
            %     dq = zeros(7, 1);
            %     robot_coppeliasim.set_target_configuration_velocities(dq);
            %     
            %     Note that this calls "set_joint_target_velocities" in
            %     DQ_CoppeliaSimInterfaceZMQ, meaning that it requires a
            %     dynamics dynamics-enabled scene and joints in dynamic
            %     mode with velocity control mode.

            arguments 
                obj (1,1) DQ_CoppeliaSimRobotZMQ
                dq (1,:) {mustBeNumeric}
            end

            obj.coppeliasim_interface_.set_joint_target_velocities( ...
                obj.joint_names_, dq);
        end

        function dq = get_configuration_velocities(obj)
            % This method gets the joint configuration velocities of the
            % robot in the CoppeliaSim scene.
            % Usage:
            %     get_configuration_velocities();  
            %
            % Example: 
            %     cs = DQ_CoppeliaSimInterfaceZMQ();
            %     cs.connect;
            %     robot_coppeliasim = DQ_CoppeliaSimRobotZMQ("/LBR4p", cs);
            %     dq = robot_coppeliasim.get_configuration_velocities;

            arguments 
                obj (1,1) DQ_CoppeliaSimRobotZMQ
            end

            dq = obj.coppeliasim_interface_.get_joint_velocities( ...
                    obj.joint_names_);
        end

        function set_target_configuration_forces(obj, tau)
            % This method sets the target joint configuration forces for
            % the robot in the CoppeliaSim scene.
            % Usage:
            %     set_target_configuration_forces(tau);  
            %          tau: The target joint configuration forces for the
            %          robot in the CoppeliaSim scene.
            %
            % Example: 
            %     cs = DQ_CoppeliaSimInterfaceZMQ();
            %     cs.connect;
            %     robot_coppeliasim = DQ_CoppeliaSimRobotZMQ("/LBR4p", cs);
            %     tau = zeros(7, 1);
            %     robot_coppeliasim.set_target_configuration_forces(tau);
            %     
            %     Note that this calls "set_joint_target_forces" in
            %     DQ_CoppeliaSimInterfaceZMQ, meaning that it requires a
            %     dynamics dynamics-enabled scene and joints in dynamic
            %     mode with force control mode.

            arguments 
                obj (1,1) DQ_CoppeliaSimRobotZMQ
                tau (1,:) {mustBeNumeric}
            end

            obj.coppeliasim_interface_.set_joint_target_forces( ...
                obj.joint_names_, tau);
        end

        function tau = get_configuration_forces(obj)
            % This method gets the joint configuration forces of the
            % robot in the CoppeliaSim scene.
            % Usage:
            %     get_configuration_forces();  
            %
            % Example: 
            %     cs = DQ_CoppeliaSimInterfaceZMQ();
            %     cs.connect;
            %     robot_coppeliasim = DQ_CoppeliaSimRobotZMQ("/LBR4p", cs);
            %     tau = robot_coppeliasim.get_configuration_forces;

            arguments 
                obj (1,1) DQ_CoppeliaSimRobotZMQ
            end

            tau = obj.coppeliasim_interface_.get_joint_forces( ...
                    obj.joint_names_);
        end
    end
end
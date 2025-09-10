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
%
%
% YouBotCoppeliaSimRobotZMQ - Concrete class to interface with the "youBot"
% robot in CoppeliaSim.
%
% Usage:
%
%   1. Drag-and-drop a "youBot" robot to a CoppeliaSim scene.
%
%   2. Run
%       >> cs = DQ_CoppeliaSimInterfaceZMQ();
%       >> cs.connect;
%       >> robot_coppeliasim = YouBotCoppeliaSimRobotZMQ("/youBot", cs);
%       >> cs.start_simulation();
%       >> robot_coppeliasim.get_configuration();
%       >> pause(1);
%       >> cs.stop_simulation();
%    Note that the name of the robot should be EXACTLY the same as in
%    CoppeliaSim. For instance, if you drag-and-drop a second robot to the
%    scene, CoppeliaSim will rename the first robot to "/youBot[0]" and
%    name the second one "/youBot[1]". A third robot will be named
%    "/youBot[2]", and so on.

classdef YouBotCoppeliaSimRobotZMQ < DQ_SerialCoppeliaSimRobotZMQ
    properties (Constant)
        adjust_ = ((cos(pi/2) + DQ.i*sin(pi/2))*(cos(pi/4) + ...
                        DQ.j*sin(pi/4)))*(1 + 0.5*DQ.E* - 0.1*DQ.k);
    end

    methods
        function obj = YouBotCoppeliaSimRobotZMQ(robot_name, coppeliasim_interface)
            % This method constructs an instance of a youBot robot.
            % Usage:
            %     YouBotCoppeliaSimRobotZMQ(robot_name, coppeliasim_interface)
            %          robot_name: The name of the robot in the CoppeliaSim
            %          scene.
            %          coppeliasim_interface: The DQ_CoppeliaSimInterfaceZMQ
            %          object connected to the CoppeliaSim scene.
            %
            % Example: 
            %      cs = DQ_CoppeliaSimInterfaceZMQ();
            %      cs.connect;
            %      robot_coppeliasim = YouBotCoppeliaSimRobotZMQ("/youBot", cs);
            %     
            %    Note that the name of the robot should be EXACTLY the same
            %    as in CoppeliaSim. For instance, if you drag-and-drop a
            %    second robot to the scene, CoppeliaSim will rename the
            %    first robot to "/youBot[0]" and name the second one
            %    "/youBot[1]". A third robot will be named "/youBot[2]",
            %    and so on.

            arguments 
                robot_name (1,:) {mustBeText}
                coppeliasim_interface (1,1) DQ_CoppeliaSimInterfaceZMQ
            end

            obj@DQ_SerialCoppeliaSimRobotZMQ(robot_name, ...
                                          coppeliasim_interface);

            % Fix the base name because youBot is a mobile manipulator
            obj.base_frame_name_ = robot_name;

            % Fix the joint names set by DQ_SerialCoppeliaSimRobot because
            % youBot does not follow the standard naming convention in
            % CoppeliaSim
            auto_joint_names = obj.joint_names_;
            obj.joint_names_ = cell(5,1);
            [obj.joint_names_{:}] = deal(auto_joint_names{4}, ...
                                         auto_joint_names{9}, ...
                                         auto_joint_names{12:14});
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
            %     robot_coppeliasim = YouBotCoppeliaSimRobotZMQ("/youBot", cs);
            %     q = zeros(8, 1);
            %     robot_coppeliasim.set_configuration(q);
            %     
            %     Note that this calls "set_joint_positions" in
            %     DQ_CoppeliaSimInterfaceZMQ, meaning that it requires a
            %     dynamics disabled scene.

            arguments 
                obj (1,1) YouBotCoppeliaSimRobotZMQ
                q (1,:) {mustBeNumeric}
            end

            % Calculate the base's pose from its configuration
            x = q(1);
            y = q(2);
            phi = q(3);
            
            r = cos(phi/2.0)+DQ.k*sin(phi/2.0);
            p = x*DQ.i + y*DQ.j;
            pose = (1 + DQ.E*0.5*p)*r;

            % Set the base's pose and the arm's configuration in
            % CoppeliaSim
            obj.coppeliasim_interface_.set_object_pose( ...
                obj.base_frame_name_, pose*(obj.adjust_'));
            obj.coppeliasim_interface_.set_joint_positions( ...
                obj.joint_names_, q(4:8));
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
            %     robot_coppeliasim = YouBotCoppeliaSimRobotZMQ("/youBot", cs);
            %     q = robot_coppeliasim.get_configuration;

            arguments 
                obj (1,1) YouBotCoppeliaSimRobotZMQ
            end

            % Get the base's pose from CoppeliaSim
            base_pose = obj.coppeliasim_interface_.get_object_pose( ...
                obj.base_frame_name_)*obj.adjust_;
            base_translation = vec3(translation(base_pose));
            base_phi = rotation_angle(rotation(base_pose));

            % Get the arm's configuration from CoppeliaSim
            base_arm_q = obj.coppeliasim_interface_.get_joint_positions( ...
                obj.joint_names_);
            
            % Concatenate the base's pose and the arm's configuration
            q = [base_translation(1);
                 base_translation(2);
                 base_phi;
                 base_arm_q];
        end

        function kin = kinematics(~)
            % This method constructs an instance of a DQ_WholeBody.
            % Usage:
            %     kinematics();
            %
            % Example: 
            %      cs = DQ_CoppeliaSimInterfaceZMQ();
            %      cs.connect;
            %      robot_coppeliasim = YouBotCoppeliaSimRobotZMQ("/youBot", cs);
            %      robot_kinematics = robot_coppeliasim.kinematics();
            
            include_namespace_dq

            % The DH parameters and other geometric parameters are based on
            % Kuka's documentation:
            %   http://www.youbot-store.com/wiki/index.php/YouBot_Detailed_Specifications
            %   https://www.generationrobots.com/img/Kuka-YouBot-Technical-Specs.pdf

            arm_DH_theta = [0, pi/2, 0, pi/2, 0];
            arm_DH_d = [0.147, 0, 0, 0, 0.218];
            arm_DH_a = [0.033, 0.155, 0.135, 0, 0];
            arm_DH_alpha = [pi/2, 0, 0, pi/2, 0];
            arm_DH_type = double(repmat( ...
                DQ_SerialManipulatorDH.JOINT_ROTATIONAL, 1, 5));
            arm_DH_matrix = [arm_DH_theta;
                             arm_DH_d;
                             arm_DH_a;
                             arm_DH_alpha;
                             arm_DH_type];
            
            arm =  DQ_SerialManipulatorDH(arm_DH_matrix, 'standard');
            base = DQ_HolonomicBase();
            
            % Set the base's frame displacement
            x_bm = 1 + E_*0.5*(0.165*i_ + 0.11*k_);            
            base.set_frame_displacement(x_bm);
            
            % Create a DQ_WholeBody object
            kin = DQ_WholeBody(base);
            kin.add(arm);
            
            % Set the end-effector
            kin.set_effector(1 + E_*0.5*0.3*k_);
        end
    end
end
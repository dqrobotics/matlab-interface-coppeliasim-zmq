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
% LBR4pCoppeliaSimRobotZMQ - Concrete class to interface with the "KUKA
% LBR4+" robot in CoppeliaSim.
%
% Usage:
%
%   1. Drag-and-drop a "KUKA LBR4+" robot to a CoppeliaSim scene.
%
%   2. Run
%       >> cs = DQ_CoppeliaSimInterfaceZMQ();
%       >> cs.connect;
%       >> robot_coppeliasim = LBR4pVrepRobot("/LBR4p", cs);
%       >> cs.start_simulation();
%       >> robot_coppeliasim.get_configuration();
%       >> pause(1);
%       >> cs.stop_simulation();
%    Note that the name of the robot should be EXACTLY the same as in
%    CoppeliaSim. For instance, if you drag-and-drop a second robot to the
%    scene, CoppeliaSim will rename the first robot to "/LBR4p[0]" and name
%    the second one "/LBR4p[1]". A third robot will be named "/LBR4p[2]",
%    and so on.

classdef LBR4pCoppeliaSimRobotZMQ < DQ_SerialCoppeliaSimRobotZMQ
    methods
        function obj = LBR4pCoppeliaSimRobotZMQ(robot_name, coppeliasim_interface)
            % This method constructs an instance of a KUKA LBR4+ robot.
            % Usage:
            %     LBR4pCoppeliaSimRobotZMQ(robot_name, coppeliasim_interface)
            %          robot_name: The name of the robot in the CoppeliaSim
            %          scene.
            %          coppeliasim_interface: The DQ_CoppeliaSimInterfaceZMQ
            %          object connected to the CoppeliaSim scene.
            %
            % Example: 
            %      cs = DQ_CoppeliaSimInterfaceZMQ();
            %      cs.connect;
            %      robot_coppeliasim = LBR4pCoppeliaSimRobotZMQ("/LBR4p", cs);
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

            obj@DQ_SerialCoppeliaSimRobotZMQ(robot_name, ...
                coppeliasim_interface);
        end

        function kin = kinematics(obj)
            % This method constructs an instance of a DQ_SerialManipulatorDH.
            % Usage:
            %     kinematics();
            %
            % Example: 
            %      cs = DQ_CoppeliaSimInterfaceZMQ();
            %      cs.connect;
            %      robot_coppeliasim = LBR4pCoppeliaSimRobotZMQ("/LBR4p", cs);
            %      robot_kinematics = robot_coppeliasim.kinematics();

            arguments 
                obj (1,1) LBR4pCoppeliaSimRobotZMQ
            end

            LBR4p_DH_theta = [0, 0, 0, 0, 0, 0, 0];
            LBR4p_DH_d = [0.200, 0, 0.4, 0, 0.39, 0, 0];
            LBR4p_DH_a = [0, 0, 0, 0, 0, 0, 0];
            LBR4p_DH_alpha = [pi/2, -pi/2, pi/2, -pi/2, pi/2, -pi/2, 0];
            LBR4p_DH_type = double(repmat( ...
                DQ_SerialManipulatorDH.JOINT_ROTATIONAL, 1, 7));
            LBR4p_DH_matrix = [LBR4p_DH_theta;
                               LBR4p_DH_d;
                               LBR4p_DH_a;
                               LBR4p_DH_alpha
                               LBR4p_DH_type];
            
            kin = DQ_SerialManipulatorDH(LBR4p_DH_matrix, 'standard');
            
            % Set the reference frames
            kin.set_reference_frame( ...
                obj.coppeliasim_interface_.get_object_pose( ...
                    obj.base_frame_name_));
            kin.set_base_frame(obj.coppeliasim_interface_.get_object_pose( ...
                obj.base_frame_name_));

            % Set the end-effector
            kin.set_effector(1 + 0.5*DQ.E*DQ.k*0.07);
        end
    end
end
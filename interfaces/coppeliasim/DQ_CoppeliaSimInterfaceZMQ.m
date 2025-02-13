% (C) Copyright 2025 DQ Robotics Developers
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
%    1. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
%         - Responsible for the original implementation, which is based on
%           https://github.com/dqrobotics/cpp-interface-coppeliasim-zmq/blob/main/src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.cpp

classdef DQ_CoppeliaSimInterfaceZMQ < DQ_CoppeliaSimInterface

    properties (Access=protected)
        host_;
        rpcPort_;
        cntPort_;
        verbose_;
        MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION_;
    end
    
    methods (Access = protected)
        function validate_input_(~, input, class, msg)
              if ~isa(input, class)
                  error(msg)
              end
        end

        function connect_(obj, host, rpcPort, MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION, cntPort, verbose)
            try
                obj.host_ = host;
                obj.rpcPort_ = rpcPort;
                obj.cntPort_ = cntPort;
                obj.verbose_ = verbose;
                obj.MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION_ = MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION; 
                
                fut = parfeval(@zmq_wrapper.create_client, 4, host, rpcPort, cntPort, verbose); 
                finished = wait(fut, 'finished', 1);    

            catch ME
                rethrow(ME)
            end
        end
    end
    methods

        function obj = DQ_CoppeliaSimInterfaceZMQ()
        end
        
        function connect(obj, host, port, TIMEOUT_IN_MILISECONDS)
            % This method connects to the remote api server (i.e. CoppeliaSim).
            % Calling this function is required before anything else can happen.
            switch nargin
                case 1
                    host = 'localhost';
                    port = 23000;
                    TIMEOUT_IN_MILISECONDS = 300;
                case 2
                    obj.validate_input_(host, 'string', 'The host argument must be a string. Example: "localhost", "192.168.120.1".');
                    port = 23000;
                    TIMEOUT_IN_MILISECONDS = 300;
           
                case 3
                   obj.validate_input_(host, 'string', 'The host argument must be a string. Example: "localhost", "192.168.120.1".');
                   obj.validate_input_(port, 'numeric', 'The port argument must be numeric. Example: 23000.');
                    TIMEOUT_IN_MILISECONDS = 300;
           

                case 4
                   obj.validate_input_(host, 'string', 'host must be a string. Example: "localhost", "192.168.120.1".');
                   obj.validate_input_(port, 'numeric', 'port must be numeric. Example: 23000.');
                   obj.validate_input_(TIMEOUT_IN_MILISECONDS, 'numeric', 'TIMEOUT_IN_MILISECONDS must be numeric. Example: 300.');

                otherwise
                  error('Wrong number of arguments');
            end

           obj.connect_(host, port, TIMEOUT_IN_MILISECONDS, -1, false);


        end

        function set_stepping_mode(obj, flag)
        end

        function trigger_next_simulation_step(obj)
        end

        function start_simulation(obj)
        end

        function stop_simulation(obj)
        end

        function handles = get_object_handles(obj,names)
        end

        function handle = get_object_handle(obj,name)
        end

        function t = get_object_translation(obj,objectname)
        end

        function set_object_translation(obj,objectname,translation)
        end

        function r = get_object_rotation(obj, objectname)
        end

        function set_object_rotation(obj,objectname,rotation)
        end

        function x = get_object_pose(obj,objectname)
        end

        function set_object_pose(obj,objectname,pose)
        end

        function set_joint_positions(obj,jointnames,joint_positions)
        end

        function joint_positions = get_joint_positions(obj,jointnames)
        end

        function set_joint_target_positions(obj,jointnames,joint_target_positions)
        end

        function joint_velocities = get_joint_velocities(obj,jointnames)
        end

        function set_joint_target_velocities(obj,jointnames,joint_target_velocities)
        end

        function set_joint_torques(obj,jointnames,torques)
        end

        function joint_torques = get_joint_torques(obj,jointnames)
        end

    end
end


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

    properties (Access = private)
        client_;
        sim_;
        client_created_;
    end

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

        function rtn = connect_(obj, host, rpcPort, MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION, cntPort, verbose)
                % This method initializes the remote API client. 
                %        host: The IP address of the computer that hosts the CoppeliaSim simulation. If the client (your code)
                %             and the simulation are running in the same computer, you can use "localhost".
                %        port: The port to establish a connection. (e.g. 23000, 23001, 23002, 23003...).
                %        TIMEOUT_IN_MILISECONDS The timeout to establish the connection. 
                %                               However, this timeout feature is not implemented yet.
                %        cntPort: This parameter is not well documented on
                %        the remote API. A typical value is -1
                %        verbose: This parameter is not well documented on
                %        the remote API. A typical value is false.
                obj.host_ = host;
                obj.rpcPort_ = rpcPort;
                obj.cntPort_ = cntPort;
                obj.verbose_ = verbose;
                obj.MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION_ = MAX_TIME_IN_MILLISECONDS_TO_TRY_CONNECTION; 

                try
                    if ~obj.client_created_ 
                         obj.client_ = RemoteAPIClient('host', obj.host_,'port',obj.rpcPort_, 'cntPort', obj.cntPort_, 'verbose', obj.verbose_);
                         obj.sim_ = obj.client_.require('sim');
                         obj.client_created_ = true;
                         obj.set_status_bar_message_('DQ Robotics established a connection on port ' + string(obj.rpcPort_), ...
                             obj.sim_.verbosity_warnings);
                    end
                catch ME
                    rethrow(ME)
                end
                rtn = true;
        end

        function check_client_(obj)
            % This method throws an exception if the client is not
            % initialized. 
            if (~obj.client_created_)
                error('Unestablished connection. Did you use connect()?');
            end
        end

        function set_status_bar_message_(obj, message, verbosity_type)
            % This method sets a message on the status bar of CoppeliaSim
            % Usage:
            %     set_status_bar_message_('DQ Robotics', obj.sim_.verbosity_warnings)
            %     
            %     message. The desired message to show on the status bar.
            %     verbosity_type. The verbosity level, as explained in the
            %                     documentation:
            %                     https://manual.coppeliarobotics.com/en/apiConstants.htm#verbosity
            obj.sim_.addLog(verbosity_type, message);
        end

    end

    methods

        function obj = DQ_CoppeliaSimInterfaceZMQ()
            obj.client_created_ = false;
        end
        
        function status = connect(obj, host, port, TIMEOUT_IN_MILISECONDS)
            % This method connects to the remote api server (i.e.CoppeliaSim). 
            % Returns true if the connection is established. False otherwise.
            % Calling this function is required before anything else can happen.
            %
            % Usage: vi.connect(host, port, TIMEOUT_IN_MILISECONDS)
            %
            %        host: The IP address of the computer that hosts the CoppeliaSim simulation. If the client (your code)
            %             and the simulation are running in the same computer, you can use "localhost".
            %        port: The port to establish a connection. (e.g. 23000, 23001, 23002, 23003...).
            %        TIMEOUT_IN_MILISECONDS The timeout to establish the connection.       
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                host (1,1) {mustBeText} = 'localhost'
                port (1,1) {mustBeNumeric} = 23000
                TIMEOUT_IN_MILISECONDS (1,1) {mustBeNumeric} = 1000
            end     
            status = obj.connect_(host, port, TIMEOUT_IN_MILISECONDS, -1, false);
        end

        function set_stepping_mode(obj, flag)
            % This method enables or disables the stepping mode (formerly known as synchronous mode). 
            % Usage : set_stepping_mode(true)   % enables the stepping mode
            %         set_stepping_mode(false)  % disables the stepping mode
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                flag (1,1) {mustBeNumericOrLogical}
            end
            obj.check_client_();
            obj.sim_.setStepping(flag);
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


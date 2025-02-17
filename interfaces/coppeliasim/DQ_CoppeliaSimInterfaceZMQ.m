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
        handles_map_;
        enable_deprecated_name_compatibility_ = true;
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

        function rtn = start_with_slash_(~, objectname)
            % This method returns true if the first character of the
            % objectname string is '/'. Returns false otherwise.
            %
            % Example: start_with_slash_('/joint') % returns true.
            %          start_with_slash_('frame')  % returns false.
            %
            k = strfind(objectname,'/');
            n = length(k);
            if (n==0)
                rtn = false;
            else
              if (k(1)==1)
                rtn = true;
              else
                rtn = false;
              end
            end
        end

        function newstr = remove_first_slash_from_string_(obj, str)  
            % This method removes the slash from the given string only if
            % the slash is in the first position of the string.
            % Example:
            %    remove_first_slash_from_string_('/joint') returns 'joint'
            %    remove_first_slash_from_string_('robot/joint') returns 'robot/joint'  
            %
            if obj.start_with_slash_(str)
                 newstr = erase(str,"/");
            else
                 newstr = str;
            end
        end

        function standard_str = get_standard_name_(obj, str)
            % This method returns a string that starts with a slash in the 
            % first position.
            %
            % Example: get_standard_name_('/robot') % returns '/joint'
            % Example: get_standard_name_('robot')  % returns '/joint'
            %
              standard_str = str;
              if (~obj.start_with_slash_(str) && obj.enable_deprecated_name_compatibility_ == true)
                 standard_str = '/'+str;
              end
        end

        function check_sizes_(~, v1, v2, message)
            % This method throws an exception with the desired message if
            % the sizes of v1 and v2 are different.
            if (length(v1) ~= length(v2))
                error(message);
            end
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
            % This method sends a trigger signal to the CoppeliaSim scene, 
            % which performs a simulation step when the stepping mode is used.
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
            end
            obj.check_client_();
            obj.sim_.step();
        end

        function start_simulation(obj)
            % This method starts the CoppeliaSim simulation.
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
            end
            obj.check_client_();
            obj.sim_.startSimulation();
        end

        function stop_simulation(obj)
             % This method stops the CoppeliaSim simulation.
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
            end           
            obj.check_client_();
            obj.sim_.stopSimulation()
        end

        function handles = get_object_handles(obj,objectnames)
            % This method returns a cell that contains the handles
            % corresponding to the objectnames.
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                objectnames (1,:) {mustBeText} 
            end
            n = length(objectnames);
            handles = cell(1,n);
            for i=1:n
               handles{i} = obj.get_object_handle(objectnames{i});
            end
        end

        function handle = get_object_handle(obj,objectname)
            % get_object_handle gets the object handle from CoppeliaSim. 
            % If the handle is not included in the map, then the map is updated.
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                objectname (1,1) {mustBeText} 
            end
            % If the objectname does not start with a slash and the
            % deprecated name compatibility is not enabled, this method
            % will throw an exception. Therefore, additional information about the error is
            % provided to the user.
            additional_error_message = "";
            if (~obj.start_with_slash_(objectname) && ...
                    obj.enable_deprecated_name_compatibility_ == false)      
                additional_error_message = "Did you mean   " + char(34) + '/' +objectname +  char(34) + '   ?';
            end

            try
                obj.check_client_();
                standard_objectname = obj.get_standard_name_(objectname);
                handle = obj.sim_.getObject(standard_objectname);
                obj.update_map_(standard_objectname, handle);

            catch ME
                
                msg = "The object "  + char(34) + objectname  + char(34) + " does not exist in the " + ...
                      "current scene in CoppeliaSim. " + additional_error_message;
                obj.throw_runtime_error_(ME, msg)
            end  

        end

        function t = get_object_translation(obj,objectname)
            % This method returns the translation of an object in the CoppeliaSim scene.
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                objectname (1,1) {mustBeText} 
            end
            obj.check_client_();
            position = obj.sim_.getObjectPosition(obj.get_handle_from_map_(objectname), ...
            obj.sim_.handle_world);
            t = DQ(double([position{1},position{2},position{3}]));
        end

        function set_object_translation(obj, objectname, translation)
            % This method sets the translation of an object in the CoppeliaSim scene.
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                objectname (1,1) {mustBeText} 
                translation (1,1) DQ
            end
            obj.check_client_();
            vec_t = vec3(translation);
            position = {vec_t(1), vec_t(2), vec_t(3)};
            obj.sim_.setObjectPosition(obj.get_handle_from_map_(objectname), ...
                                       position,obj.sim_.handle_world);
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


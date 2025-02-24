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

        function rtn = get_handle_from_map_(obj, objectname)
            % This method searchs a handle in the map. If the handle is not found, 
            % it is taken from CoppeliaSim and the map is updated
            % by using the get_object_handle() method.
            if isConfigured(obj.handles_map_)
                if (isKey(obj.handles_map_ , objectname))
                  rtn = obj.handles_map_(objectname);
                else
                  rtn = obj.get_object_handle(objectname);
                end
            else
                rtn = obj.get_object_handle(objectname);
            end
        end

        function throw_runtime_error_(~, ME, msg)
            % This method throws an exception after showing a custom
            % message.
            disp(msg);
            rethrow(ME); 
        end

        function update_map_(obj, objectname, handle)
            % This method updates the map. The map is updated only if the objectname is not in the map.
            % In other words, it is not allowed to have an objectname twice in the map.
                obj.handles_map_ = insert(obj.handles_map_, objectname, handle);
        end

        function check_sizes_(~, v1, v2, message)
            % This method throws an exception with the desired message if
            % the sizes of v1 and v2 are different.
            if (length(v1) ~= length(v2))
                error(message);
            end
        end

        function set_joint_position_(obj, jointname, angle_rad)
           % This method sets the position of a joint in the CoppeliaSim scene.
           obj.check_client_();
           obj.sim_.setJointPosition(obj.get_handle_from_map_(jointname), angle_rad);
        end

        function theta = get_joint_position_(obj, jointname)
           % This method gets the position of a joint in the CoppeliaSim scene.
           obj.check_client_();
           theta = double(obj.sim_.getJointPosition(obj.get_handle_from_map_(jointname)));
        end

        function set_joint_target_position_(obj, jointname, angle_rad)
           % This method sets the target position of a joint in the CoppeliaSim scene. 
           obj.check_client_();
           obj.sim_.setJointTargetPosition(obj.get_handle_from_map_(jointname), angle_rad);
        end

        function theta_dot = get_joint_velocity_(obj, jointname)
            % This method gets the velocity of a joint in the CoppeliaSim scene.
            obj.check_client_();
            theta_dot = obj.sim_.getObjectFloatParam(obj.get_handle_from_map_(jointname), ...
                        obj.sim_.jointfloatparam_velocity);
        end

        function set_joint_target_velocity_(obj, jointname, angle_rad_dot) 
           % This method sets the target velocity of a joint in the CoppeliaSim scene. 
           obj.check_client_();
           obj.sim_.setJointTargetVelocity(obj.get_handle_from_map_(jointname), angle_rad_dot);
        end

    end

    methods

        function obj = DQ_CoppeliaSimInterfaceZMQ()
            obj.client_created_ = false;
            obj.handles_map_ = dictionary;
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
                host (1,1) {mustBeText} = "localhost"
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
                objectname (1,:) {mustBeText} 
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
            % This method returns a pure quaternion that represents the translation of an object 
            % in the CoppeliaSim scene.
            %
            % Usage:
            %    t = get_object_translation(objectname)
            %
            %       objectname The object name
            %       t A pure quaternion that represents the translation of
            %       the desired object in the CoppeliaSim scene.
            %
            % Example:
            %
            %      t = get_object_translation('DefaultCamera');
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
            %
            % Usage:
            %    set_object_translation(objectname)
            %
            %       objectname (string) The object name
            %       translation (pure quaternion) The desired translation
            % Example:
            %
            %      set_object_translation('DefaultCamera', t);  
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
            % This method returns a unit quaternion that represents the rotation of an object 
            % in the CoppeliaSim scene.
            %
            % Usage:
            %    r = get_object_rotation(objectname)
            %
            %       objectname The object name
            %       r A unit quaternion that represents the rotation of
            %       the desired object in the CoppeliaSim scene.
            %
            % Example:
            %
            %      r = get_object_rotation('DefaultCamera');
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                objectname (1,1) {mustBeText} 
            end
           obj.check_client_();
           rotation = obj.sim_.getObjectQuaternion(obj.get_handle_from_map_(objectname) ...
                        + obj.sim_.handleflag_wxyzquat, obj.sim_.handle_world);
           r = normalize(DQ(double([rotation{1},rotation{2},rotation{3}, rotation{4}])));
        end

        function set_object_rotation(obj,objectname, rotation)
            % This method sets the rotation of an object in the CoppeliaSim scene.
            %
            % Usage:
            %    set_object_rotation(objectname)
            %
            %       objectname (string) The object name
            %       rotation (unit quaternion) The desired rotation
            % Example:
            %
            %      set_object_translation('DefaultCamera', r);  
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                objectname (1,1) {mustBeText} 
                rotation (1,1) DQ
            end
            obj.check_client_();
            vec_r = vec4(rotation);
            rotation = {vec_r(1), vec_r(2), vec_r(3), vec_r(4)};
            obj.sim_.setObjectQuaternion(obj.get_handle_from_map_(objectname) ...
                        + obj.sim_.handleflag_wxyzquat, rotation, obj.sim_.handle_world);
        end

        function x = get_object_pose(obj,objectname)
            % This method returns a unit dual quaternion that represents the pose of an object 
            % in the CoppeliaSim scene.
            %
            % Usage:
            %    x = get_object_pose(objectname)
            %
            %       objectname The object name
            %       x A unit dual quaternion that represents the pose of
            %       the desired object in the CoppeliaSim scene.
            %
            % Example:
            %
            %      x = get_object_pose('DefaultCamera');
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                objectname (1,1) {mustBeText} 
            end
           t = obj.get_object_translation(objectname);
           r = obj.get_object_rotation(objectname);
           x = r + 0.5*DQ.E*t*r;
        end

        function set_object_pose(obj,objectname,pose)
            % This method sets the pose of an object in the CoppeliaSim scene.
            %
            % Usage:
            %    set_object_pose(objectname, pose)
            %
            %       objectname (string) The object name
            %       pose (unit dual quaternion) The desired pose
            % Example:
            %
            %      set_object_pose('DefaultCamera', pose);  
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                objectname (1,1) {mustBeText} 
                pose (1,1) DQ
            end
            if ~is_unit(pose)
                error('Bad call in DQ_CoppeliaSimInterfaceZMQ.set_object_pose. The pose must be a unit dual quaternion!');
            end
            obj.check_client_();
            vec_r = h.P().vec4();
            vec_t = h.translation().vec3(); 
            pose = {vec_t(1), vec_t(2), vec_t(3),vec_r(1), vec_r(2), vec_r(3), vec_r(4)};
            obj.sim_.setObjectPose(obj.get_handle_from_map_(objectname) + obj.sim_.handleflag_wxyzquat, ...
                     pose, obj.sim_.handle_world);
        end

        function set_joint_positions(obj,jointnames, joint_positions)
            % This method sets the joint positions in the CoppeliaSimscene.
            % It is required a dynamics disabled scene. 
            %
            % Usage:
            %    set_joint_positions(objectnames, joint_positions)
            %
            %       objectnames (cell of strings) The joint names
            %       joint_positions (vector) The joint positions
            %
            % Example:
            %      jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %       u = [0.1 0.1 0.1 0.1 0.1 0.1 0.1];
            %
            %       set_joint_positions(jointnames, u);
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                jointnames (1,:) {mustBeText} 
                joint_positions (1,:) {mustBeNumeric}
            end
            message = 'Bad call in DQ_CoppeliaSimInterfaceZMQ.set_joint_positions. jointnames and joint_positions have incompatible sizes';
            obj.check_sizes_(jointnames, joint_positions, message);
            n = length(jointnames);
            for i=1:n
               obj.set_joint_position_(jointnames{i}, joint_positions(i));
            end

        end

        function joint_positions = get_joint_positions(obj,jointnames)
            % This method gets the joint positions in the CoppeliaSimscene.
            % Usage:
            %    joint_positions = get_joint_positions(jointnames)
            %
            %         objectnames (cell of strings) The joint names
            %
            % Example:
            %      jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %
            %      joint_positions = get_joint_positions(jointnames);
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                jointnames (1,:) {mustBeText} 
            end
            n = length(jointnames);
            joint_positions = zeros(n,1);
            for i=1:n
                joint_positions(i) = obj.get_joint_position_(jointnames{i});
            end
        end

        function set_joint_target_positions(obj,jointnames, joint_target_positions)
            % This method sets the joint target positions in the CoppeliaSimscene.
            % It is required a dynamics enabled scene, and joints in dynamic mode 
            % with position control mode. 
            %
            % Usage:
            %    set_joint_positions(objectnames, joint_positions)
            %
            %       objectnames (cell of strings) The joint names
            %       joint_target_positions (vector) The joint target positions
            %
            % Example:
            %      jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %       u = [0.1 0.1 0.1 0.1 0.1 0.1 0.1];
            %
            %       set_joint_target_positions(jointnames, u);
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                jointnames (1,:) {mustBeText} 
                joint_target_positions (1,:) {mustBeNumeric}
            end
           message = 'Bad call in DQ_CoppeliaSimInterfaceZMQ.set_joint_target_positions. jointnames and joint_target_positions have incompatible sizes';
           obj.check_sizes_(jointnames, angles_rad, message);   
           n = length(jointnames);
           for i=1:n
               obj.set_joint_target_position_(jointnames{i}, joint_target_positions(i));
           end
        end

        function joint_velocities = get_joint_velocities(obj,jointnames)
            % This method gets the joint velocities in the CoppeliaSimscene.
            % Usage:
            %    joint_velocities = get_joint_velocities(jointnames)
            %
            %         objectnames (cell of strings) The joint names
            %
            % Example:
            %      jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %
            %      joint_velocities = get_joint_velocities(jointnames);
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                jointnames (1,:) {mustBeText} 
            end
           n = length(jointnames);
           joint_velocities = zeros(n,1);
           for i=1:n
               joint_velocities(i) = obj.get_joint_velocity_(jointnames{i});
           end
        end

        function set_joint_target_velocities(obj,jointnames,joint_target_velocities)
            % This method sets the joint target velocities in the CoppeliaSimscene.
            % It is required a dynamics enabled scene, and joints in dynamic mode 
            % with velocity control mode. 
            %
            % Usage:
            %    set_joint_target_velocities(objectnames, joint_target_velocities)
            %
            %       objectnames (cell of strings) The joint names
            %       joint_target_velocities (vector) The joint target velocities
            %
            % Example:
            %      jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %       joint_target_velocities = [0.1 0.1 0.1 0.1 0.1 0.1 0.1];
            %
            %       set_joint_target_velocities(jointnames, joint_target_velocities);
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                jointnames (1,:) {mustBeText} 
                joint_target_velocities (1,:) {mustBeNumeric}
            end
            message = 'Bad call in DQ_CoppeliaSimInterfaceZMQ.set_joint_target_velocities. jointnames and joint_target_velocities have incompatible sizes';
            obj.check_sizes_(jointnames, angles_rad, message);   
            n = length(jointnames);
            for i=1:n
               obj.set_joint_target_velocity_(jointnames{i}, joint_target_velocities(i));
            end
        end

        function set_joint_torques(obj,jointnames,torques)
            % This method sets the joint torques in the CoppeliaSimscene.
            % It is required a dynamics enabled scene, and joints in dynamic mode 
            % with velocity or force control mode. 
            %
            % Usage:
            %    set_joint_torques(objectnames, torques)
            %
            %       objectnames (cell of strings) The joint names
            %       torques (vector) The joint torques
            %
            % Example:
            %      jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %       torques = [0.1 0.1 0.1 0.1 0.1 0.1 0.1];
            %
            %       set_joint_torques(jointnames, torques);
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                jointnames (1,:) {mustBeText} 
                torques (1,:) {mustBeNumeric}
            end
            message = "Bad call in DQ_CoppeliaSimInterfaceZMQ.set_joint_torques: " + ...
                     "jointnames and torques have incompatible sizes";
            obj.check_sizes_(jointnames, torques, message);   
            n = length(jointnames);
            for i=1:n
                obj.set_joint_torque(jointnames{i}, torques(i));
            end 
        end

        function joint_torques = get_joint_torques(obj,jointnames)
            % This method gets the joint torques in the CoppeliaSimscene.
            % Usage:
            %    torques = get_joint_torques(jointnames)
            %
            %         objectnames (cell of strings) The joint names
            %
            % Example:
            %      jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %
            %      torques = get_joint_torques(jointnames);
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                jointnames (1,:) {mustBeText} 
            end        
           n = length(jointnames);
           joint_torques = zeros(n,1);
           for i=1:n
               joint_torques(i) = obj.get_joint_torque(jointnames{i});
           end 
        end

    end
end


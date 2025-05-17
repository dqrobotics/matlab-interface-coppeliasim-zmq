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
%    1. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
%         - Responsible for the original implementation, which is based on
%           https://github.com/dqrobotics/cpp-interface-coppeliasim-zmq/blob/main/src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.cpp
%
%
% Instructions:
%
%   1. Add the ZMQ remote API folder to your Matlab path, which is typically located on:
%      YOUR_COPPELIASIM_FOLDER/programming/zmqRemoteApi/clients/matlab
%
%   2. Add the DQ_CoppeliaSimInterface to your Matlab path. This file is
%      available on https://github.com/dqrobotics/matlab-interface-coppeliasim

classdef DQ_CoppeliaSimInterfaceZMQ < DQ_CoppeliaSimInterface

    properties (Access = private)
        client_;
        sim_;
        client_created_;
        handles_map_;
        enable_deprecated_name_compatibility_;
        use_dictionaries_for_maps_;
    end

    properties (Access=protected)
        host_;
        rpcPort_;
        cntPort_;
        verbose_;
        max_time_in_milliseconds_to_try_connection_;
        compatible_version_ = "4.7.0-rev4";
    end
    
    methods (Access = protected)
        function rtn = create_handle_container_using_dictionaries_(obj, flag)
            % This method returns an uninitialized dictionary by default or
            % a containers.Map if the flag is set to false. 
            %
            % For more information about containers.Maps and dictionaries:
            %       https://uk.mathworks.com/help/matlab/ref/containers.map.html
            %       https://uk.mathworks.com/help/matlab/ref/dictionary.html
            %
            % Usage: rtn = create_handle_container_using_dictionaries_(flag)
            %
            %     flag: Use false to return a containers.Maps. Use true
            %           (default) to return a dictionary.
            %     rtn: The desired  uninitialized container.
            %       
            % Example:
            %     % To return a containers.Map
            %     handles_map_ = create_handle_container_using_dictionaries_(false); 
            %
            %     % To return a dictionary
            %     handles_map_ = create_handle_container_using_dictionaries_();
            %
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                flag (1,1) {mustBeNumericOrLogical} = true
            end

            obj.use_dictionaries_for_maps_ = flag;

            if obj.use_dictionaries_for_maps_
                rtn = dictionary;
            else
                rtn = containers.Map;
            end
        end

        function rtn = connect_(obj, host, rpcPort, max_time_in_milliseconds_to_try_connection, cntPort, verbose)
            % This method initializes the remote API client. 
            %
            % Usage: rtn = connect_(host, rpcPort, max_time_in_milliseconds_to_try_connection, cntPort, verbose)
            %
            %        host: The IP address of the computer that hosts the CoppeliaSim simulation. If the client (your code)
            %             and the simulation are running in the same computer, you can use "localhost".
            %        rpcPort: The port to establish a connection (e.g. 23000, 23001, 23002, 23003...).
            %        max_time_in_milliseconds_to_try_connection: The timeout to establish the connection. 
            %                               However, this timeout feature is not implemented yet.
            %        cntPort: This parameter is not well documented on
            %                the remote API. A typical value is -1
            %        verbose: This parameter is not well documented on
            %                the remote API. A typical value is false.
            %        rtn: The connection status.
            %
            % Example:
            %     rtn = connect_("localhost", 23000, 1000, -1, false)
            %
            obj.host_ = host;
            obj.rpcPort_ = rpcPort;
            obj.cntPort_ = cntPort;
            obj.verbose_ = verbose;   
            obj.max_time_in_milliseconds_to_try_connection_ = max_time_in_milliseconds_to_try_connection; 

            try
                if ~obj.client_created_ 
                     obj.client_ = RemoteAPIClient('host', obj.host_,'port', obj.rpcPort_, 'cntPort', obj.cntPort_, 'verbose', obj.verbose_);
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
            %
            % Usage: set_status_bar_message_(message, verbosity_type)
            %     
            %      message: The desired message to show on the status bar.
            %      verbosity_type: The verbosity level, as explained in the
            %                     documentation:
            %                     https://manual.coppeliarobotics.com/en/apiConstants.htm#verbosity
            %
            % Example:
            %     set_status_bar_message_('DQ Robotics', obj.sim_.verbosity_warnings)
            %
            obj.sim_.addLog(verbosity_type, message);
        end

        function rtn = start_with_slash_(~, objectname)
            % This method returns true if the first character of the
            % objectname string is '/'. Returns false otherwise.
            %
            % Usage: rtn = start_with_slash_(objectname)
            %
            %      objectname: The object name.
            %      rtn: The flag that indicates if the first string character is a slash.
            %
            % Example: 
            %     start_with_slash_('/joint') % returns true.
            %     start_with_slash_('frame')  % returns false.
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


        function standard_str = get_standard_name_(obj, objectname)
            % This method returns a string that starts with a slash in the 
            % first position.
            %
            % Usage: standard_str = get_standard_name_(objectname)
            %
            %      objectname: The object name.
            %      standard_str: The standard object name.
            %
            % Example: 
            %     get_standard_name_('/robot') % returns '/robot'
            %     get_standard_name_('robot')  % returns '/robot'
            %
            arguments
                obj DQ_CoppeliaSimInterfaceZMQ
                objectname string
            end
            standard_str = objectname;
            if (~obj.start_with_slash_(objectname) && obj.enable_deprecated_name_compatibility_ == true)
               standard_str = '/'+objectname;
            end
        end

        function rtn = get_handle_from_map_(obj, objectname)
            % This method searchs a handle in the map. If the handle is not found, 
            % it is taken from CoppeliaSim and the map is updated
            % by using the get_object_handle() method.
            %
            % Usage: rtn = get_handle_from_map_(objectname)
            %
            %    objectname: The object name
            %    rtn: The object handle.
            %
            % Example:
            %     handle = get_handle_from_map_("/joint")
            %
            if (obj.use_dictionaries_for_maps_)
                % For dictionaries    
                if isConfigured(obj.handles_map_)
                    if (isKey(obj.handles_map_ , objectname))
                      rtn = obj.handles_map_(objectname);
                    else
                      rtn = obj.get_object_handle(objectname);
                    end
                else
                    rtn = obj.get_object_handle(objectname);
                end

            else
                % For containers.Map
                if (isKey(obj.handles_map_ , objectname))
                  rtn = obj.handles_map_(objectname);
                else
                  rtn = obj.get_object_handle(objectname);
                end
            end
        end

        function throw_runtime_error_(~, ME, msg)
            % This method throws an exception after showing a custom
            % message.
            %
            % Usage: throw_runtime_error_(ME, msg)
            %
            %     ME: MException object contains retrievable information about errors.
            %         For more information: https://uk.mathworks.com/help/matlab/ref/mexception.html
            %     msg: custom message to show before throwing the exception.
            %
            % Example:
            %
            %     try
            %        surf
            %     catch ME
            %        throw_runtime_error_(ME, "something went wrong!")
            %     end
            %
            disp(msg);
            rethrow(ME); 
        end

        function update_map_(obj, objectname, handle)
            % This method updates the map. The map is updated only if the objectname is not in the map.
            % In other words, it is not allowed to have an objectname twice in the map.
            %
            % Usage: update_map_(objectname, handle)
            %
            %    objectname: The object name.
            %    handle: The object handle.
            %
            % Example:
            %     update_map_("/joint", 10);
            %
            if (obj.use_dictionaries_for_maps_)
                % For dictionaries
                obj.handles_map_ = insert(obj.handles_map_, objectname, handle);     
            else          
                % For containers.Map
                obj.handles_map_(objectname) = handle;
            end
        end

        function rtn = get_port_from_deprecated_default_port_(~, port)
            % This method returns the default ZMQ port if the input port 
            % corresponds to a default port of the legacy API.
            %
            % Usage: rtn = get_port_from_deprecated_default_port_(port)
            %   
            %     port: The port to establish a connection.
            %     rtn: The ZMQ port.
            %
            % Example:
            %     port = get_port_from_deprecated_default_port_(19997) % returns 23000
            %     port = get_port_from_deprecated_default_port_(23000) % returns 23000
            %
            auxport = port;
            if port == 19997 || port == 19998 || port == 19999 || port == 20000
                auxport = 23000;
                warning("The port " + int2str(port) + " is commonly used in the legacy API. " + ...
                    "However it is not compatible with the ZMQ Remote API. I changed the port to " + int2str(auxport));
            end
            rtn = auxport;
        end

        function check_sizes_(~, v1, v2, message)
            % This method throws an exception with the desired message if
            % the sizes of v1 and v2 are different.
            %
            % Usage: check_sizes_(v1, v2, message)
            %
            % Example:
            %     check_sizes([0.1 0.2 0.3], {'joint1','joint2'}, "Wrong sizes")
            %
            if (length(v1) ~= length(v2))
                error(message);
            end
        end

        function set_joint_position_(obj, jointname, angle_rad)
            % This method sets the position of a joint in the CoppeliaSim scene.
            % 
            % Usage: set_joint_position_(jointname, angle_rad)
            %
            %     jointname: The joint name.
            %     angle_rad: The joint position.
            % 
            % Example:
            %     set_joint_position_("/joint", 0.5)
            %
            obj.check_client_();
            obj.sim_.setJointPosition(obj.get_handle_from_map_(jointname), angle_rad);
        end

        function theta = get_joint_position_(obj, jointname)
            % This method gets the position of a joint in the CoppeliaSim scene.
            %
            % Usage: theta = get_joint_position_(jointname)
            %
            %     jointname: The joint name.
            %     theta: The joint position.
            %
            % Example:
            %     p = get_joint_position_("/joint")
            %
            obj.check_client_();
            theta = double(obj.sim_.getJointPosition(obj.get_handle_from_map_(jointname)));
        end

        function set_joint_target_position_(obj, jointname, angle_rad)
            % This method sets the target position of a joint in the CoppeliaSim scene. 
            % 
            % Usage: set_joint_target_position_(jointname, angle_rad)
            %
            %     jointname: The joint name.
            %     angle_rad: The target joint position.
            %  
            % Example:
            %     set_joint_target_position_("/joint", 0.5)
            %
            obj.check_client_();
            obj.sim_.setJointTargetPosition(obj.get_handle_from_map_(jointname), angle_rad);
        end

        function theta_dot = get_joint_velocity_(obj, jointname)
            % This method gets the velocity of a joint in the CoppeliaSim scene.
            %
            % Usage: theta_dot = get_joint_velocity_(jointname)
            %
            %     jointname: The joint name.
            %     theta_dot: The joint velocity.
            %
            % Example:
            %     v = get_joint_velocity_("/joint")
            %
            obj.check_client_();
            theta_dot = obj.sim_.getObjectFloatParam(obj.get_handle_from_map_(jointname), ...
                        obj.sim_.jointfloatparam_velocity);
        end

        function set_joint_target_velocity_(obj, jointname, angle_rad_dot) 
            % This method sets the target velocity of a joint in the CoppeliaSim scene. 
            % 
            % Usage: set_joint_target_velocity_(jointname, angle_rad_dot)
            %
            %     jointname: The joint name.
            %     angle_rad_dot: The target joint velocity.
            % 
            % Example:
            %     set_joint_target_velocity_("/joint", 0.5)
            %
            obj.check_client_();
            obj.sim_.setJointTargetVelocity(obj.get_handle_from_map_(jointname), angle_rad_dot);
        end

        function force = get_joint_force_(obj, jointname)
            % This method gets the force of a joint in the CoppeliaSim scene.
            %
            % Usage: force = get_joint_force_(jointname)
            %
            %     jointname: The joint name.
            %     force: The joint force.
            %
            % Example:
            %     f = get_joint_force_("/joint")
            %
            obj.check_client_();
            
            % The getJointForce method retrieves the force or torque applied to a joint along/about its active axis. 
            % This method uses the joint perspective, which returns an inverted signal value. 
            % For instance, if the joint is set to a positive target force (e.g., using setJointTargetForce), 
            % the direction of motion will follow the right-hand rule. 
            % However, when using getJointForce, which uses the joint perspective, this motion is seen as clockwise, 
            % resulting in a negative value. 
            % 
            % More information:
            %  https://manual.coppeliarobotics.com/en/regularApi/simGetJointForce.htm
            %  https://forum.coppeliarobotics.com/viewtopic.php?p=41694&hilit=getJointForce#p41694
            % 
            % Therefore, to return a joint force using the same reference that 
            % setJointTargetForce uses, we multiply it by -1.
            force = -obj.sim_.getJointForce(obj.get_handle_from_map_(jointname));
        end

        function set_joint_target_force_(obj, jointname, force) 
            % This method sets the target force of a joint in the CoppeliaSim scene. 
            % 
            % Usage: set_joint_target_force_(jointname, torque)
            %
            %     jointname: The joint name.
            %     force: The target joint force.
            % 
            % Example:
            %     set_joint_target_force_("/joint", 0.5)
            %
            obj.check_client_();
            obj.sim_.setJointTargetForce(obj.get_handle_from_map_(jointname), force, true);
        end

    end

    methods
        function obj = DQ_CoppeliaSimInterfaceZMQ()
            % Constructor of the class. 
            

            % This property is set in false at the beginning and is used to
            % check if the connection is established given a timeout.
            % However, the timeout feature is not implemented yet.
            obj.client_created_ = false;

            % Enable name compatibility
            % If enable_deprecated_name_compatibility_ is set true, 
            % the class will accept names without the initial slash.
            % Example: The user can use: 
            %      get_objec t_pose("/Franka") or get_objec t_pose("Franka")
            %
            % If the flag is set to false, then the class only will accept
            % names in which the first character is a slash. This is the
            % default behavior of the ZeroMQ Remote API. 

            % Accept both styles in the objectnames (i.e., "/name" or "name")
            obj.enable_deprecated_name_compatibility_ = true;

            % Note for future developers:
            %
            % Matlab recommends the use of dictionaries over container maps
            % because it accepts more data types as keys and values and provides better performance.
            % However, dictionaries are available since Matlab R2022. Because of that, 
            % the maps are implemented using containers.Map
            %
            % Source:
            % https://uk.mathworks.com/help/matlab/ref/containers.map.html

            % create the containers.Map (or dictionary).
            % To force the use of a containers.Map, use:
            % create_handle_container_using_dictionaries_(false)
            %
            % To force the use of a dictionaries, use: 
            % create_handle_container_using_dictionaries_()
            obj.handles_map_ = obj.create_handle_container_using_dictionaries_(false); 

            disp("This version of DQ_CoppeliaSimInterfaceZMQ is compatible" + ...
                  " with CoppeliaSim " + obj.compatible_version_);
        end
        
        function status = connect(obj, host, port, timeout_in_milliseconds)
            % This method connects to the remote API server (i.e.CoppeliaSim). 
            % Returns true if the connection is established. False otherwise.
            % Calling this function is required before anything else can happen.
            %
            % Usage: status = connect(host, port, timeout_in_milliseconds)
            %
            %        host: The IP address of the computer that hosts the CoppeliaSim simulation. If the client (your code)
            %             and the simulation are running in the same
            %             computer, you can use "localhost" (default).
            %        port: The port to establish a connection. (e.g. 23000,
            %        23001, 23002, 23003...). The default port is 23000.
            %        timeout_in_milliseconds: The timeout to establish the connection. However, this feature is not implemented yet. 
            %        status: The connection status. The default value is 5000.
            %
            % Example:
            %     connect("localhost", 23000, 1000)
            %
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                host (1,:) {mustBeText} = "localhost" % The size is (1,:) to be compatible with strings and vector of characters.
                port (1,1) {mustBeNumeric} = 23000
                timeout_in_milliseconds (1,1) {mustBeNumeric} = 5000
            end     
            if nargin == 3 || nargin == 4
                port = obj.get_port_from_deprecated_default_port_(port);
            end
            if (nargin == 4)
                warning("Timeout has not been implemented yet and will be added in future releases. " + ...
                    "See discussion in https://github.com/dqrobotics/matlab-interface-coppeliasim-zmq/pull/1")
            end
            status = obj.connect_(host, port, timeout_in_milliseconds, -1, false);
        end

        function set_stepping_mode(obj, flag)
            % This method enables or disables the stepping mode (formerly known as synchronous mode).
            %
            % Usage: set_stepping_mode(flag)
            %        
            %        flag: Use true to enable the stepping mode. False otherwise 
            % 
            % Example: 
            %     set_stepping_mode(true)   % enables the stepping mode
            %     set_stepping_mode(false)  % disables the stepping mode
            %
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

        function handles = get_object_handles(obj, objectnames)
            % This method returns a cell that contains the handles
            % corresponding to the objectnames.
            %
            % Usage: handles = get_object_handles(objectnames)
            %
            %    objectnames: The object names.
            %    handles: The object handles.
            %
            % Example:
            %     handles = get_object_handles({'/joint1', '/joint2'})
            %
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

        function handle = get_object_handle(obj, objectname)
            % This method gets the object handle from CoppeliaSim. 
            % If the handle is not included in the map, then the map is updated.
            %
            % Usage: handle = get_object_handle(objectname)
            %
            %      objectname: The object name
            %      handle: The object handle.
            %
            % Example:
            %     handle = get_object_handle("/joint")
            %
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                objectname (1,:) {mustBeText} % The size is (1,:) to be compatible with strings and vector of characters.
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

        function translation = get_object_translation(obj, objectname)
            % This method returns a pure quaternion that represents the translation of an object 
            % in the CoppeliaSim scene.
            %
            % Usage: translation = get_object_translation(objectname)
            %
            %       objectname: The object name
            %       translation: The pure quaternion that represents the translation of
            %                    the desired object in the CoppeliaSim scene.
            %
            % Example:
            %     translation = get_object_translation('DefaultCamera');
            %      
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                objectname (1,:) {mustBeText} % The size is (1,:) to be compatible with strings and vector of characters.
            end
            obj.check_client_();
            position = obj.sim_.getObjectPosition(obj.get_handle_from_map_(objectname), ...
            obj.sim_.handle_world);
            translation = DQ(double([position{1},position{2},position{3}]));
        end

        function set_object_translation(obj, objectname, translation)
            % This method sets the translation of an object in the CoppeliaSim scene.
            %
            % Usage: set_object_translation(objectname, translation)
            %
            %       objectname: The object name
            %       translation: The desired translation represented by a
            %                    pure quaternion.
            %
            % Example:
            %     set_object_translation('DefaultCamera', 0.5*DQ.i);
            %        
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                objectname (1,:) {mustBeText} % The size is (1,:) to be compatible with strings and vector of characters. 
                translation (1,1) DQ
            end
            obj.check_client_();
            vec_t = vec3(translation);
            position = {vec_t(1), vec_t(2), vec_t(3)};
            obj.sim_.setObjectPosition(obj.get_handle_from_map_(objectname), ...
                                       position,obj.sim_.handle_world);
        end

        function rotation = get_object_rotation(obj, objectname)
            % This method returns a unit quaternion that represents the rotation of an object 
            % in the CoppeliaSim scene.
            %
            % Usage: rotation = get_object_rotation(objectname)
            %
            %       objectname: The object name.
            %       rotation: The unit quaternion that represents the rotation of
            %                 the desired object in the CoppeliaSim scene.
            %
            % Example:
            %     rotation = get_object_rotation('DefaultCamera');
            %      
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                objectname (1,:) {mustBeText} % The size is (1,:) to be compatible with strings and vector of characters.
            end
            obj.check_client_();
            r = obj.sim_.getObjectQuaternion(obj.get_handle_from_map_(objectname) ...
                        + obj.sim_.handleflag_wxyzquat, obj.sim_.handle_world);
            rotation = normalize(DQ(double([r{1},r{2},r{3}, r{4}])));
        end

        function set_object_rotation(obj, objectname, rotation)
            % This method sets the rotation of an object in the CoppeliaSim scene.
            %
            % Usage: set_object_rotation(objectname, rotation)
            %
            %       objectname: The object name.
            %       rotation: The desired rotation represented by a unit
            %                 quaternion.
            %
            % Example:
            %     set_object_rotation('DefaultCamera', cos(pi/4) + DQ.k*sin(pi/4));
            %        
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                objectname (1,:) {mustBeText} % The size is (1,:) to be compatible with strings and vector of characters.
                rotation (1,1) DQ
            end
            obj.check_client_();
            vec_r = vec4(rotation);
            rotation = {vec_r(1), vec_r(2), vec_r(3), vec_r(4)};
            obj.sim_.setObjectQuaternion(obj.get_handle_from_map_(objectname) ...
                        + obj.sim_.handleflag_wxyzquat, rotation, obj.sim_.handle_world);
        end

        function pose = get_object_pose(obj, objectname)
            % This method returns a unit dual quaternion that represents the pose of an object 
            % in the CoppeliaSim scene.
            %
            % Usage:
            %    pose = get_object_pose(objectname)
            %
            %       objectname: The object name.
            %       pose: The unit dual quaternion that represents the pose of
            %             the desired object in the CoppeliaSim scene.
            %
            % Example:
            %     pose = get_object_pose('DefaultCamera');
            %      
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                objectname (1,:) {mustBeText} % The size is (1,:) to be compatible with strings and vector of characters.
            end
            t = obj.get_object_translation(objectname);
            r = obj.get_object_rotation(objectname);
            pose = r + 0.5*DQ.E*t*r;
        end

        function set_object_pose(obj, objectname, pose)
            % This method sets the pose of an object in the CoppeliaSim scene.
            %
            % Usage: set_object_pose(objectname, pose)
            %
            %       objectname: The object name.
            %       pose: The desired pose represented by a unit dual
            %             quaternion.
            % Example:
            %     set_object_pose('DefaultCamera', DQ(1));  
            %      
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                objectname (1,:) {mustBeText} % The size is (1,:) to be compatible with strings and vector of characters.
                pose (1,1) DQ
            end
            if ~is_unit(pose)
                error('Bad call in DQ_CoppeliaSimInterfaceZMQ.set_object_pose. The pose must be a unit dual quaternion!');
            end
            obj.check_client_();
            vec_r = vec4(P(pose));
            vec_t = vec3(translation(pose)); 
            pose = {vec_t(1), vec_t(2), vec_t(3),vec_r(1), vec_r(2), vec_r(3), vec_r(4)};
            obj.sim_.setObjectPose(obj.get_handle_from_map_(objectname) + obj.sim_.handleflag_wxyzquat, ...
                     pose, obj.sim_.handle_world);
        end

        function set_joint_positions(obj, jointnames, joint_positions)
            % This method sets the joint positions in the CoppeliaSim scene.
            % It is required a dynamics disabled scene. 
            %
            % Usage: set_joint_positions(objectnames, joint_positions)
            %
            %       objectnames: The joint names.
            %       joint_positions: The desired joint positions.
            %
            % Example:
            %      jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %       u = [0.1 0.1 0.1 0.1 0.1 0.1 0.1];
            %
            %       set_joint_positions(jointnames, u);
            %
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

        function joint_positions = get_joint_positions(obj, jointnames)
            % This method gets the joint positions in the CoppeliaSim scene.
            %
            % Usage: joint_positions = get_joint_positions(jointnames)
            %
            %         objectnames: The joint names.
            %         joint_positions: The joint positions.
            %
            % Example:
            %      jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %
            %      joint_positions = get_joint_positions(jointnames);
            %
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

        function set_joint_target_positions(obj, jointnames, joint_target_positions)
            % This method sets the joint target positions in the CoppeliaSim scene.
            % It requires a dynamics-enabled scene and joints in dynamic mode with position control mode. 
            % For more information about joint modes:
            % https://www.coppeliarobotics.com/helpFiles/en/jointModes.htm
            %
            % Usage: set_joint_positions(objectnames, joint_positions)
            %
            %       objectnames: The joint names.
            %       joint_target_positions: The joint target positions.
            %
            % Example:
            %      jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %       u = [0.1 0.1 0.1 0.1 0.1 0.1 0.1];
            %
            %       set_joint_target_positions(jointnames, u);
            %
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                jointnames (1,:) {mustBeText} 
                joint_target_positions (1,:) {mustBeNumeric}
            end
           message = 'Bad call in DQ_CoppeliaSimInterfaceZMQ.set_joint_target_positions. jointnames and joint_target_positions have incompatible sizes';
           obj.check_sizes_(jointnames, joint_target_positions, message);   
           n = length(jointnames);
           for i=1:n
               obj.set_joint_target_position_(jointnames{i}, joint_target_positions(i));
           end
        end

        function joint_velocities = get_joint_velocities(obj, jointnames)
            % This method gets the joint velocities in the CoppeliaSim scene.
            %
            % Usage: joint_velocities = get_joint_velocities(jointnames)
            %
            %         objectnames: The joint names.
            %         joint_velocities: The joint velocities.
            %
            % Example:
            %      jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %
            %      joint_velocities = get_joint_velocities(jointnames);
            %
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

        function set_joint_target_velocities(obj, jointnames, joint_target_velocities)
            % This method sets the joint target velocities in the CoppeliaSim scene.
            % It requires a dynamics-enabled scene and joints in dynamic mode with velocity control mode. 
            % For more information about joint modes:
            % https://www.coppeliarobotics.com/helpFiles/en/jointModes.htm
            %
            % Usage: set_joint_target_velocities(objectnames, joint_target_velocities)
            %
            %       objectnames: The joint names.
            %       joint_target_velocities: The joint target velocities.
            %
            % Example:
            %      jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %       joint_target_velocities = [0.1 0.1 0.1 0.1 0.1 0.1 0.1];
            %
            %       set_joint_target_velocities(jointnames, joint_target_velocities);
            %
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                jointnames (1,:) {mustBeText} 
                joint_target_velocities (1,:) {mustBeNumeric}
            end
            message = 'Bad call in DQ_CoppeliaSimInterfaceZMQ.set_joint_target_velocities. jointnames and joint_target_velocities have incompatible sizes';
            obj.check_sizes_(jointnames, joint_target_velocities, message);   
            n = length(jointnames);
            for i=1:n
               obj.set_joint_target_velocity_(jointnames{i}, joint_target_velocities(i));
            end
        end

        function set_joint_target_forces(obj, jointnames, forces)
            % This method sets the joint target forces in the CoppeliaSim scene.
            % It requires a dynamics-enabled scene and joints in dynamic mode with force control mode. 
            % For more information about joint modes:
            % https://www.coppeliarobotics.com/helpFiles/en/jointModes.htm
            %
            % Usage: set_joint_torques(objectnames, forces)
            %
            %       objectnames: The joint names.
            %       forces: The target joint forces.
            %
            % Example:
            %      jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %      forces = [0.1 0.1 0.1 0.1 0.1 0.1 0.1];
            %
            %      set_joint_target_forces(jointnames, forces);
            %
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                jointnames (1,:) {mustBeText} 
                forces (1,:) {mustBeNumeric}
            end
            message = "Bad call in DQ_CoppeliaSimInterfaceZMQ.set_joint_torques: " + ...
                     "jointnames and forces have incompatible sizes";
            obj.check_sizes_(jointnames, forces, message);   
            n = length(jointnames);
            for i=1:n
                obj.set_joint_target_force_(jointnames{i}, forces(i));
            end 
        end

        function joint_forces = get_joint_forces(obj, jointnames)
            % This method gets the joint forces in the CoppeliaSim scene.
            %
            % Usage: forces = get_joint_forces(jointnames)
            %
            %         objectnames: The joint names.
            %
            % Example:
            %      jointnames={'LBR4p_joint1','LBR4p_joint2','LBR4p_joint3','LBR4p_joint4',...
            %                  'LBR4p_joint5','LBR4p_joint6','LBR4p_joint7'};
            %
            %      forces = get_joint_forces(jointnames);
            arguments
                obj  (1,1) DQ_CoppeliaSimInterfaceZMQ
                jointnames (1,:) {mustBeText} 
            end        
           n = length(jointnames);
           joint_forces = zeros(n,1);
           for i=1:n
               joint_forces(i) = obj.get_joint_force_(jointnames{i});
           end 
        end

    end

    % Exclusive ZMQ methods, which are not enforced by the
    % DQ_CoppeliaSimInterface class
    methods 
        function objectname = get_object_name(obj, handle)
            % This method gets the name of an object in the CoppeliaSim scene.
            %
            % Usage: objectname = get_object_name(handle)
            %
            %         handle: The object handle.
            %
            % Example:
            %      objectname = get_object_name(handle)
           arguments 
                obj (1,1) DQ_CoppeliaSimInterfaceZMQ
                handle (1,1) {mustBeNumeric}
           end
           obj.check_client_();
           objectname = obj.sim_.getObjectAlias(handle, 1);
           obj.update_map_(objectname, handle);
        end

        function objectnames = get_object_names(obj, handles)
            % This method gets the object names in the CoppeliaSim scene.
            %
            % Usage: objectnames = get_object_name(handles)
            %
            %         handles: The object handles.
            %
            % Example:
            %      objectnames = get_object_names(handles)
           arguments 
                obj (1,1) DQ_CoppeliaSimInterfaceZMQ
                handles cell
           end
           n = length(handles);
           objectnames = cell(1,n);
           for i=1:n
               objectnames{i}=obj.get_object_name(handles{i});
           end
        end

        function jointnames =  get_jointnames_from_object(obj, objectname)
            % This method gets all the names of the joints under the hierarchy tree, 
            % in which the object name describes its base.
            %
            % Usage: jointnames = get_jointnames_from_object(objectname);
            %
            %         objectname: The parent object name of the joints.
            %
            % Example:
            %      jointnames = get_jointnames_from_object('LBR4p');
           arguments 
                obj (1,1) DQ_CoppeliaSimInterfaceZMQ
                objectname (1,:) {mustBeText} 
           end
           obj.check_client_();
           base_handle = obj.get_handle_from_map_(objectname);
           jointhandles = obj.sim_.getObjectsInTree(base_handle,...
                                                obj.sim_.object_joint_type,0);
           jointnames = obj.get_object_names(jointhandles);
        end
    end

    methods % Deprecated methods to ensure backwards compatibility
        function disconnect_all(~)
            warning("The method disconnect_all is deprecated and is not required with ZeroMQ remote API.");
        end
        function disconnect(~)
            warning("The method disconnect is deprecated and is not required with ZeroMQ remote API.");
        end
        function set_synchronous(obj, flag)
            warning("The method set_synchronous is deprecated. Consider using set_stepping_mode(flag) instead.");
            obj.set_stepping_mode(flag);
        end
        function wait_for_simulation_step_to_end(~)
            warning("The method wait_for_simulation_step_to_end is deprecated and is not required with ZeroMQ remote API.");
        end
    end
end


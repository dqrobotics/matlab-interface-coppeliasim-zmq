
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
%           https://github.com/dqrobotics/cpp-interface-coppeliasim-zmq/blob/main/src/dqrobotics/interfaces/coppeliasim/internal/_zmq_wrapper.h

classdef zmq_wrapper < handle

    
    properties (Access = private)

        client_;
        sim_;
    end
    
    methods(Static, Access=public)
        function rtn = create_client(obj, host, rpcPort, cntPort, verbose)
            obj.client_ = RemoteAPIClient('host', host,'port',rpcPort, 'cntPort', cntPort, 'verbose', verbose);
            obj.sim_ = obj.client_.require('sim');
            rtn = true;
        end

        function rtn = get_sim(obj)
            rtn = obj.sim_;
        end

        
    end
end


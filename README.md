![GitHub License](https://img.shields.io/github/license/dqrobotics/matlab-interface-coppeliasim-zmq)![Static Badge](https://img.shields.io/badge/based_on-ZeroMQ_remote_API-blue)

# matlab-interface-coppeliasim-zmq

A DQ Robotics interface based on the ZeroMQ remote API to connect with CoppeliaSim. 

### How to use

```Matlab
clear;
close all;
clc;

cs = DQ_CoppeliaSimInterfaceZMQ();

try
    cs.connect(); 
    cs.start_simulation();
    x = cs.get_object_pose("/Floor")
    cs.stop_simulation();
catch ME
    rethrow(ME)
end
```

Check more examples [here.](https://github.com/dqrobotics/matlab-examples)

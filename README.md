# Gas-Source-Localization
Implementations of several GSL methods for ROS.

To use one of the implemented algorithms, simply launch gsl_actionserver_node and create an ActionClient to send a message with the name of the chosen algorithm (see gsl_server_call.cpp for an example).
Currently available algorithms include:

"surge_cast"
"surge_spiral"
"spiral"
"particle_filter"
(See paper: https://dl.acm.org/doi/10.1145/3378184.3378220)

"grid"
(See paper: https://ieeexplore.ieee.org/document/9347683)

Make sure to set move_base to use the global_planner/GlobalPlanner plugin rather than navfn, since the implementations of the algorithms utilize the make_plan service.

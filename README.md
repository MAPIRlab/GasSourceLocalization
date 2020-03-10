# Gas-Source-Localization
Implementations of several GSL methods for ROS.

To use one of the implemented algorithms, simply launch gsl_actionserver_node and create an ActionClient to send a message with the name of the chosen algorithm (see gsl_server_call.cpp for an example).
Currently available algorithms include:

"surge_cast"

"surge_spiral"

"spiral"

"particle_filter"

Make sure to set move_base to use the global_planner/GlobalPlanner plugin rather than navfn, since the implementation of the algorithms utilize the make_plan service.

# Gas-Source-Localization
Implementations of several GSL methods for ROS.

To use one of the implemented algorithms, simply launch gsl_actionserver_node and create an ActionClient to send a message with the name of the chosen algorithm (see gsl_server_call.cpp for an example).
Currently available algorithms include:


### (currently under review)
- "PMFS"

### (See paper: https://ieeexplore.ieee.org/document/9347683)
- "GrGSL"

### (See paper: https://dl.acm.org/doi/10.1145/3378184.3378220)
- "surge_cast"
- "surge_spiral"
- "spiral"
- "particle_filter"

# Installing and building
You can download this package by navigating to your catkin workspace and running:

`git clone --recursive git@github.com:MAPIRlab/Gas-Source-Localization.git GSL`

This will get you the source code of the GSL algorithms and some additional dependencies (under gsl_actionserver/third_party). However, due to the nature of how ROS code is organized in semi-independent packages, it does not quite make sense for this repository to include other catkin packages (which you might already have in the same workspace for something else) as submodules. You will need to install those separately:

## Dependencies

- [GMRF-wind](https://github.com/MAPIRlab/GMRF-wind)
- [Nav-assistant](https://github.com/MAPIRlab/navigation-assistant)
- [Olfaction msgs](https://github.com/MAPIRlab/olfaction_msgs)
- (Optional)[Gaden](https://github.com/MAPIRlab/gaden)

If you don't want to use Gaden for your experiments you can omit it and set the `USE_GADEN` option in the [CMakeLists.txt](gsl_actionserver/CMakeLists.txt) file to `OFF` to compile without the corresponding dependency.
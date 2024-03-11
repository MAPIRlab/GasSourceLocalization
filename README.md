# Gas Source Localization
Implementations of several GSL methods for ROS.

To use one of the implemented algorithms, simply launch the `gsl_actionserver_node` node and create an ActionClient to send a goal with the name of the chosen algorithm (see [gsl_server_call.cpp](gsl_server/src/gsl_server_call.cpp) for an example).
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
You can download this package by navigating to your ROS workspace and running:

`git clone --recursive git@github.com:MAPIRlab/Gas-Source-Localization.git GSL`

Due to the nature of how ROS code is organized in semi-independent packages, it does not quite make sense for this repository to include other ament packages (which you might already have in the same workspace for something else) as submodules. You will need to install those separately:

## Dependencies

- [GMRF-wind](https://github.com/MAPIRlab/GMRF-wind)
- [Olfaction msgs](https://github.com/MAPIRlab/olfaction_msgs)

- (Optional) [Nav-assistant](https://github.com/MAPIRlab/navigation-assistant)
- (Optional) [Gaden](https://github.com/MAPIRlab/gaden)
- (Optional) [ament_imgui](https://github.com/PepeOjeda/ament_imgui)


If you don't want to use Gaden for your experiments you can omit it and set the `USE_GADEN` option in the [CMakeLists.txt](gsl_server/CMakeLists.txt) file to `OFF` to compile without the corresponding dependency.

Similarly, the dependency with `ament_imgui` can be disabled using the `USE_GUI` option, and the dependency with `Nav-Assistant` can be disabled with `USE_NAV_ASSISTANT` in the [CMakeLists.txt](gsl_server/CMakeLists.txt).
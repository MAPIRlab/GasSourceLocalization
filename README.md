# Gas Source Localization
Implementations of several GSL methods for ROS.

To use one of the implemented algorithms, simply launch the `gsl_actionserver_node` node and create an ActionClient to send a goal with the name of the chosen algorithm (see [gsl_server_call.cpp](gsl_server/src/gsl_server_call.cpp) for an example).
Currently available algorithms include:


- `"PMFS"` (See paper [here](https://ieeexplore.ieee.org/document/10592836))
- `"GrGSL"` (See paper: [here](https://ieeexplore.ieee.org/document/9347683))
- `"surge_cast"`, `"surge_spiral"`, `"spiral"`, `"particle_filter"` (See paper [here](https://dl.acm.org/doi/10.1145/3378184.3378220))
- `"SemanticPMFS"`, `"SemanticGrGSL"` (See paper [here](https://arxiv.org/abs/2501.12812))

# Installing and building
You can download this package by navigating to your ROS workspace and running:

`git clone --recurse-submodules git@github.com:MAPIRlab/Gas-Source-Localization.git GSL`

## Dependencies
Due to the nature of how ROS code is organized in semi-independent packages, it does not quite make sense for this repository to include other ament packages (which you might already have in the same workspace for something else) as submodules. You will need to install those separately:

- [GMRF-wind](https://github.com/MAPIRlab/GMRF-wind)
- [Olfaction msgs](https://github.com/MAPIRlab/olfaction_msgs)

There are also some additional, optional dependecies that you might not need, depending on which algorithms you choose to compile (see next section).

- (Optional) [Nav-assistant](https://github.com/MAPIRlab/navigation-assistant)
- (Optional) [Gaden](https://github.com/MAPIRlab/gaden)
- (Optional) [ament_imgui](https://github.com/PepeOjeda/ament_imgui)
- (Optional) [Voxeland](https://github.com/MAPIRlab/Voxeland)

## Compiling specific algorithms
While this project contains several different GSL algorithms, you might only be interested in using one of them. Since some of these algorithms carry additional dependencies, you might want to completely disable the compilation of algorithms you are not going to use, to avoid having to install their dependencies. You can do that by commenting out the [corresponding line in the main CMakeLists.txt](https://github.com/MAPIRlab/GasSourceLocalization/blob/f24ea320986eea2972bddf0ef847cb8c793950ac/gsl_server/CMakeLists.txt#L86)


## Disabling optional dependencies
Even if you are compiling all algorithms, there are some dependencies that can just be turned off by disabling the corresponding features.

If you don't want to use Gaden for your experiments you can omit it and set the `USE_GADEN` option in the [CMakeLists.txt](gsl_server/CMakeLists.txt) file to `OFF` to compile without the corresponding dependency.

Similarly, the dependency with `ament_imgui` can be disabled using the `USE_GUI` option, and the dependency with `Nav-Assistant` can be disabled with `USE_NAV_ASSISTANT` in the [CMakeLists.txt](gsl_server/CMakeLists.txt).


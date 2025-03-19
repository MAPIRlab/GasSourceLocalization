#include "SemanticPMFS.hpp"

#if ENABLE_CLASSMAP2D

#include "gsl_server/algorithms/Semantics/Semantics/2D/ClassMap2D.hpp"
void GSL::SemanticPMFS::createClassMap2D()
{
    semantics = std::make_unique<ClassMap2D>(gridMetadata, simulationOccupancy, tfBuffer, currentRobotPose); 
}

#else

void GSL::SemanticPMFS::createClassMap2D()
{
    GSL_ERROR("Selected semantics type ClassMap2D was not compiled!");
}

#endif

#if ENABLE_CLASSMAP_VOXELAND

#include "gsl_server/algorithms/Semantics/Semantics/Voxeland/ClassMapVoxeland.hpp"
void GSL::SemanticPMFS::createClassMapVoxeland()
{
    semantics = std::make_unique<ClassMapVoxeland>(gridMetadata, simulationOccupancy, tfBuffer, node);
}

#else

void GSL::SemanticPMFS::createClassMapVoxeland()
{
    GSL_ERROR("Selected semantics type ClassMapVoxeland was not compiled!");
}

#endif
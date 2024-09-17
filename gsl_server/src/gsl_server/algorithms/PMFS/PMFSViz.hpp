#pragma once
#include <gsl_server/algorithms/Common/Grid2D.hpp>
#include <gsl_server/algorithms/PMFS/internal/HitProbability.hpp>
#include <gsl_server/algorithms/PMFS/internal/Settings.hpp>
#include <gsl_server/algorithms/PMFS/internal/PublishersAndSubscribers.hpp>
#include <gsl_server/algorithms/Common/Utils/NQAQuadtree.hpp>

namespace GSL
{
    //Functions to publish markers for Rviz
    class PMFSViz
    {
    public:
        static void ShowHitProb(Grid2D<PMFS_internal::HitProbability> grid, const PMFS_internal::VisualizationSettings& settings,
                                const PMFS_internal::PublishersAndSubscribers& pubs);
        static void ShowSourceProb(Grid2D<double> sourceProb, const PMFS_internal::VisualizationSettings& settings,
                                   const PMFS_internal::PublishersAndSubscribers& pubs);
        static void DebugMapSegmentation(const std::vector<Utils::NQA::Node>& QTleaves, const PMFS_internal::PublishersAndSubscribers& pubs,
                                         const Grid2DMetadata& gridMetadata);
        static void PlotWindVectors(Grid2D<Vector2> estimatedWindVectors, const PMFS_internal::VisualizationSettings& settings,
                                    const PMFS_internal::PublishersAndSubscribers& pubs);
    };
} // namespace GSL
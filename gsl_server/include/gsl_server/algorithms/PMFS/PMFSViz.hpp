#pragma once
#include <gsl_server/algorithms/Common/Grid.hpp>
#include <gsl_server/algorithms/PMFS/internal/HitProbability.hpp>
#include <gsl_server/algorithms/PMFS/internal/Settings.hpp>
#include <gsl_server/algorithms/PMFS/internal/PublishersAndSubscribers.hpp>
#include <gsl_server/Utils/NQAQuadtree.hpp>

namespace GSL
{
    class PMFSViz
    {
    public:
        static void ShowHitProb(Grid<PMFS_internal::HitProbability> grid, const PMFS_internal::VisualizationSettings& settings,
                                const PMFS_internal::PublishersAndSubscribers& pubs);
        static void ShowSourceProb(Grid<double> sourceProb, const PMFS_internal::VisualizationSettings& settings,
                                   const PMFS_internal::PublishersAndSubscribers& pubs);
        static void DebugMapSegmentation(const std::vector<Utils::NQA::Node>& QTleaves, const PMFS_internal::PublishersAndSubscribers& pubs,
                                         const GridMetadata& gridMetadata);
        static void PlotWindVectors(Grid<Vector2> estimatedWindVectors, const PMFS_internal::VisualizationSettings& settings,
                                    const PMFS_internal::PublishersAndSubscribers& pubs);
    };
} // namespace GSL
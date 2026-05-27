#include "SimulationSystem.hpp"
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/Common/Utils/Time.hpp>
#include <gsl_server/algorithms/PMFS/PMFS.hpp>
#include <gsl_server/algorithms/PMFS/PMFSLib.hpp>
#include <gsl_server/core/Logging.hpp>

#include <opencv2/core/hal/interface.h>
#include <opencv2/core/saturate.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <DDA/DDA.h>
#include <gsl_server/core/Profiling.hpp>

namespace GSL::PMFS_internal
{
    using HashSet = std::unordered_set<Vector2Int>;

    // create the occupancy Quadtree
    void SimulationSystem::initializeMap(const Grid2D<Occupancy> occupancyMap)
    {
        ZoneScoped;
        quadtree = std::make_unique<NQA::Quadtree>(occupancyMap);
        QTleaves = quadtree->fusedLeaves(settings.maxRegionSize);

        mapSegmentation.resize(occupancyMap.metadata.dimensions.x, std::vector<NQA::Node*>(occupancyMap.metadata.dimensions.y, nullptr));

        GSL_INFO("Number of cells after fusing quadtree: {0}", QTleaves.size());
        // generate the image of indices so you can map a cell in the map to the corresponding leaf of the quatree
        for (int i = 0; i < QTleaves.size(); i++)
        {
            NQA::Node& node = QTleaves[i];
            Vector2Int start = node.origin;
            Vector2Int end = node.origin + node.size;

            for (int r = start.x; r < end.x; r++)
            {
                for (int c = start.y; c < end.y; c++)
                {
                    GSL_ASSERT_MSG(mapSegmentation[r][c] == nullptr, "fused cells are overlapping");
                    mapSegmentation[r][c] = &node;
                }
            }
        }
        sourceProbInternal.resize(sourceProb.data.size(), 0.0);
    }

    void SimulationSystem::updateSourceProbability(float refineFraction)
    {
        ZoneScoped;
        GSL_INFO_COLOR(fmt::terminal_color::yellow, "Started simulations. Might take a while!");
        Utils::Time::Stopwatch stopwatch;
        std::vector<NQA::Node> localCopyLeaves = QTleaves;

        // first, coarse simulation based on the quadtree decomposition of the map
        //------------------------------------------------------------------------
        //------------------------------------------------------------------------

        // store the score of each region to figure out which ones are worth subdividing for finer simulation
        std::vector<LeafScore> scores(localCopyLeaves.size());
        for (int leafIndex = 0; leafIndex < scores.size(); leafIndex++)
            scores[leafIndex].leaf = &localCopyLeaves[leafIndex];

        // this is used to calculate how much the state of this cell depends on where the source is. It is used for the movemente strategy
        std::vector<Utils::RunningVariance> varianceCalculationData(measuredHitProb.data.size());

        int numberOfSimulations = 0;
        resultsFirstLevel.clear();
        resultsFirstLevel.reserve(scores.size());
// iterate over the leaves of the quadtree, doing one simulation for each and calculating how well it fits our measured gas map
#pragma omp parallel for schedule(dynamic)
        for (int leafIndex = 0; leafIndex < scores.size(); leafIndex++)
        {
            SimulationResult result = runSimulation(scores, leafIndex);
            if (!result.valid)
                continue;

// update the information for the variance calulation
#pragma omp critical
            {
                resultsFirstLevel.push_back(result);
                numberOfSimulations++;
                for (int cell = 0; cell < result.hitMap.size(); cell++)
                {
                    auto& var = varianceCalculationData[cell];
                    var.Update(result.hitMap[cell],
                               result.sourceProb);
                }
            }
        }

// update the variance thing (for the movement strategy)
#pragma omp parallel for
        for (int cellI = 0; cellI < measuredHitProb.data.size(); cellI++)
        {
            if (measuredHitProb.occupancy[cellI] == Occupancy::Free)
                varianceOfHitProb[cellI] = varianceCalculationData[cellI].variance / varianceCalculationData[cellI].weight_sum;
        }

        GSL_TRACE("First simulation level done");
        // now, finer simulation where it is deemed relevant
        //------------------------------------------------------
        //------------------------------------------------------

        int numberOfLevelsSimulated = 1;
        // Choose the most interesting quadtree leaves (the ones with the best result in the previous iteration) and subdivide them to do more
        // simulations. Keep going until none of the leaves can be subdivided any more
        while (scores.size() > 0)
        {
            std::sort(scores.begin(), scores.end(), [](LeafScore result1, LeafScore result2)
                      {
                          return result1.score > result2.score;
                      });

            // subdivide the good cells and add the children to the list of cells to simulate
            std::vector<LeafScore> newLevel;
            for (int leafIndex = 0; leafIndex < scores.size() * refineFraction; leafIndex++)
            {
                NQA::Node* leaf = scores[leafIndex].leaf;
                leaf->ForceSubdivide();
                for (int childI = 0; childI < 4; childI++)
                    if (leaf->children[childI])
                        newLevel.push_back({0, (leaf->children[childI]).get()});
            }
            scores = newLevel;

            numberOfLevelsSimulated++;
            numberOfSimulations += scores.size();

// run the simulations of the new level and get scores for each node
#pragma omp parallel for schedule(dynamic)
            for (int leafIndex = 0; leafIndex < scores.size(); leafIndex++)
                SimulationResult result = runSimulation(scores, leafIndex);

            GSL_TRACE("Simulation level {} done", numberOfLevelsSimulated);
        }

        GSL_INFO("Number of levels in the simulation: {0}", numberOfLevelsSimulated);
        GSL_INFO("Total number of simulations: {0}", numberOfSimulations);

        Utils::NormalizeDistribution<long double>(
            sourceProbInternal,
            sourceProb.occupancy);

        for (size_t i = 0; i < sourceProb.data.size(); i++)
            sourceProb.data[i] = (double)sourceProbInternal[i];

        GSL_INFO("Time ellapsed in simulation = {} s", stopwatch.ellapsed());
    }

    SimulationSystem::SimulationResult SimulationSystem::runSimulation(std::vector<LeafScore>& scores, size_t index)
    {
        SimulationResult result{.valid = false};
        NQA::Node* node = scores[index].leaf;
        if (node->value != 1)
            return result;

        result.valid = true;
        result.hitMap.resize(measuredHitProb.data.size(), 0.0);

        Simulation sim{
            .source = SimulationSource(AABB2D(
                measuredHitProb.metadata.indicesToCoordinates(node->origin, false),
                measuredHitProb.metadata.indicesToCoordinates(node->origin + node->size, false))),
            .warmup = true,
            .timesteps = settings.iterationsToRecord,
            .deltaTime = (float)settings.deltaTime,
            .noiseSTDev = (float)settings.noiseSTDev,
            .wind = wind,
            .visibilityMap = *visibilityMap};
        sim.Run(result.hitMap);

        Simulation::blurHitMap(result.hitMap, settings.blurSigma, wind.AsOccupancy(), blurredMask);

        result.sourceProb = sourceProbFromMaps(measuredHitProb, result.hitMap);

        scores[index].score = result.sourceProb;

        // assign this probability to all cells that fall inside this region
        for (int cellI = node->origin.x; cellI < (node->origin.x + node->size.x); cellI++)
            for (int cellJ = node->origin.y; cellJ < (node->origin.y + node->size.y); cellJ++)
                sourceProbInternal[sourceProb.metadata.indexOf({cellI, cellJ})] = result.sourceProb;
        return result;
    }

    long double SimulationSystem::sourceProbFromMaps(const Grid2D<HitProbability>& measuredHitProb, const std::vector<float>& hitMap) const
    {
        ZoneScoped;
        long double total = 1;
        for (int i = 0; i < measuredHitProb.data.size(); i++)
        {
            if (measuredHitProb.occupancy[i] != Occupancy::Free)
                continue;

            const double& simulated = hitMap[i];
            double sourceGivenThisCell = probabilityFromSingleCell(measuredHitProb.data[i], simulated);
            total *= sourceGivenThisCell;
            GSL_ASSERT(!std::isnan(total));
        }
        return total;
    }

    double SimulationSystem::probabilityFromSingleCell(HitProbability hitProb, double simulated) const
    {
        return Utils::lerp(1, probabilitySingleFrequency(hitProb.probability(), simulated), hitProb.confidence);
    }

    double SimulationSystem::probabilitySingleFrequency(double measured, double simulated) const
    {
        return 1 - std::abs(measured - simulated) * settings.sourceDiscriminationPower;
    }

    void SimulationSystem::makeSimulationImage(const SimulationSource& source)
    {
        std::vector<float> hitMap(measuredHitProb.data.size(), 0.0);

        Simulation sim{
            .source = source,
            .warmup = true,
            .timesteps = settings.iterationsToRecord,
            .deltaTime = (float)settings.deltaTime,
            .noiseSTDev = (float)settings.noiseSTDev,
            .wind = wind,
            .visibilityMap = *visibilityMap};
        sim.Run(hitMap);

        Simulation::displayImage(Grid2D<float>(hitMap, wind));
    }
} // namespace GSL::PMFS_internal
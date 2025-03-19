#include "gsl_server/algorithms/PMFS/internal/HitProbability.hpp"
#include <cstddef>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/Common/Utils/Time.hpp>
#include <gsl_server/algorithms/PMFS/PMFS.hpp>
#include <gsl_server/algorithms/PMFS/PMFSLib.hpp>
#include <gsl_server/algorithms/PMFS/internal/Simulations.hpp>
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
    namespace NQA = Utils::NQA;
    using HashSet = std::unordered_set<Vector2Int>;

    // We have a long list of pre-calculated random values for Speeeeeeeeeeeeeeeeed
    static thread_local Utils::PrecalculatedGaussian<2500> gaussian;

    static void weighted_incremental_variance(double value, double weight, double& mean, double& weight_sum, double& weight_squared_sum,
                                              double& variance)
    {
        // Updating Mean and Variance Estimates: An Improved Method D.H.D. West 1979
        weight_sum = weight_sum + weight;
        weight_squared_sum = weight_squared_sum + weight * weight;
        double mean_old = mean;
        mean = mean_old + (weight / weight_sum) * (value - mean_old);
        variance = variance + weight * (value - mean_old) * (value - mean);
    }

    // create the occupancy Quadtree
    void Simulations::initializeMap(const std::vector<std::vector<uint8_t>>& occupancyMap)
    {
        ZoneScoped;
        quadtree = std::make_unique<Utils::NQA::Quadtree>(occupancyMap);
        QTleaves = quadtree->fusedLeaves(settings.maxRegionSize);

        mapSegmentation.resize(occupancyMap.size(), std::vector<Utils::NQA::Node*>(occupancyMap[0].size(), nullptr));

        GSL_INFO("Number of cells after fusing quadtree: {0}", QTleaves.size());
        // generate the image of indices so you can map a cell in the map to the corresponding leaf of the quatree
        for (int i = 0; i < QTleaves.size(); i++)
        {
            Utils::NQA::Node& node = QTleaves[i];
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

        // we want to have the occupancy mask as a cv image so we can correct the blur later
        freeSpaceMask = cv::Mat(
            cv::Size(measuredHitProb.metadata.dimensions.x, measuredHitProb.metadata.dimensions.y),
            CV_32F,
            cv::Scalar(0, 0, 0));

        for (int i = 0; i < measuredHitProb.metadata.dimensions.y; i++)
            for (int j = 0; j < measuredHitProb.metadata.dimensions.x; j++)
                if (measuredHitProb.freeAt(i, j))
                    freeSpaceMask.at<float>(i, j) = 1;
    }

    void Simulations::updateSourceProbability(float refineFraction)
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
        struct VarianceCalculationData
        {
            double mean = 0;
            double weight_sum = 0;
            double weight_squared_sum = 0;
            double variance = 0;
        };
        std::vector<VarianceCalculationData> varianceCalculationData(measuredHitProb.data.size());

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
                    weighted_incremental_variance(result.hitMap[cell],
                                                  result.sourceProb,
                                                  var.mean,
                                                  var.weight_sum,
                                                  var.weight_squared_sum,
                                                  var.variance);
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
                bool hasChildren = leaf->subdivide();
                if (hasChildren)
                {
                    for (int childI = 0; childI < 4; childI++)
                        if (leaf->children[childI])
                            newLevel.push_back({0, (leaf->children[childI]).get()});
                }
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

        Utils::NormalizeDistributionLong(
            sourceProbInternal,
            sourceProb.occupancy);

        for (size_t i = 0; i < sourceProb.data.size(); i++)
            sourceProb.data[i] = (double)sourceProbInternal[i];

        GSL_INFO("Time ellapsed in simulation = {} s", stopwatch.ellapsed());
    }

    Simulations::SimulationResult Simulations::runSimulation(std::vector<LeafScore>& scores, size_t index)
    {
        SimulationResult result{.valid = false};
        NQA::Node* node = scores[index].leaf;
        if (node->value != 1)
            return result;

        result.valid = true;
        result.hitMap.resize(measuredHitProb.data.size(), 0.0);

        SimulationSource source(node, measuredHitProb.metadata);
        simulateSourceInPosition(source, result.hitMap, true, settings.iterationsToRecord, settings.deltaTime,
                                 settings.noiseSTDev);

        if (settings.blurSigmaX > 0 || settings.blurSigmaY > 0)
        {
            cv::Mat asImage(result.hitMap);
            asImage = asImage.reshape(1, measuredHitProb.metadata.dimensions.y);
            blurHitMap(asImage);
        }

        result.sourceProb = sourceProbFromMaps(measuredHitProb, result.hitMap);

        scores[index].score = result.sourceProb;

        // assign this probability to all cells that fall inside this region
        for (int cellI = node->origin.x; cellI < (node->origin.x + node->size.x); cellI++)
            for (int cellJ = node->origin.y; cellJ < (node->origin.y + node->size.y); cellJ++)
                sourceProbInternal[sourceProb.metadata.indexOf({cellI, cellJ})] = result.sourceProb;
        return result;
    }

    long double Simulations::sourceProbFromMaps(const Grid2D<HitProbability>& measuredHitProb, const std::vector<float>& hitMap) const
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

    double Simulations::probabilityFromSingleCell(HitProbability hitProb, double simulated) const
    {
#define FREQUENCY_DISTRIBUTION_METHOD 0
#if FREQUENCY_DISTRIBUTION_METHOD
        auto frequencyDistribution = hitProb.frequencyDistribution();
        double result = 0;
        for (int freqIndex = 0; freqIndex < frequencyDistribution.size(); freqIndex++)
        {
            float measured = HitProbability::frequencyOfBucket(freqIndex);
            result += frequencyDistribution[freqIndex] * probabilitySingleFrequency(measured, simulated);
            GSL_ASSERT(!std::isnan(result));
        }
        return result;
#else
        return Utils::lerp(1, probabilitySingleFrequency(hitProb.probability(), simulated), hitProb.confidence);
#endif
    }

    double Simulations::probabilitySingleFrequency(double measured, double simulated) const
    {
        return 1 - std::abs(measured - simulated) * settings.sourceDiscriminationPower;
    }

    void Simulations::moveFilament(Filament& filament, Vector2Int& indices, float deltaTime, float noiseSTDev) const
    {
        Vector2 velocity = wind.dataAt(indices.x, indices.y) + Vector2(gaussian.nextValue(0, noiseSTDev), gaussian.nextValue(0, noiseSTDev));

        Vector2 newPos = filament.position + deltaTime * velocity;
        moveAlongPath(filament.position, newPos);
    }

    bool Simulations::filamentIsOutside(const Filament& filament) const
    {
        Vector2Int newIndices = measuredHitProb.metadata.coordinatesToIndices(filament.position.x, filament.position.y);
        return !measuredHitProb.metadata.indicesInBounds(newIndices);
    }

    void Simulations::simulateSourceInPosition(const SimulationSource& source, std::vector<float>& hitMap, bool warmup,
                                               int timesteps, float deltaTime, float noiseSTDev) const
    {
        constexpr int numFilamentsIteration = 5;
        size_t max_filaments = settings.maxWarmupIterations * numFilamentsIteration + timesteps * numFilamentsIteration;

        // To avoid having to delete filaments from the middle of the vector, which is quite slow, we will ping-pong the active filaments between two vectors
        // at the start of any iteration, one vector (active) will contain all the released filaments and the other one will be empty
        // after each filament has been moved, if it is still active, it will be copied to the other vector
        // then, the active vector changes and the old one is cleared
        std::vector<Filament> filaments1;
        std::vector<Filament> filaments2;
        filaments1.reserve(max_filaments);
        filaments2.reserve(max_filaments);
        std::vector<Filament>* activeFilamentVec = &filaments1;
        std::vector<Filament>* otherFilamentVec = &filaments2;

        std::vector<uint16_t> updated(hitMap.size(), 0); // index of the last iteration in which this cell was updated, to avoid double-counting

        // warm-up: we don't want to start recording frequency of hits until the shape of the plume has stabilized. Wait until a filament exits the
        // environment through an outlet, or a maximum number of steps
        {
            ZoneScopedN("Warmup");

            bool stable = false;
            int iterationCount = 0;
            while (iterationCount < settings.minWarmupIterations || (!stable && iterationCount < settings.maxWarmupIterations))
            {
                for (size_t i = 0; i < numFilamentsIteration; i++)
                {
                    activeFilamentVec->emplace_back();
                    activeFilamentVec->back().position = source.getPoint();
                }

                for (Filament& filament : *activeFilamentVec)
                {
                    auto indices = measuredHitProb.metadata.coordinatesToIndices(filament.position.x, filament.position.y);

                    // move active filaments
                    moveFilament(filament, indices, deltaTime * 2, noiseSTDev);

                    // remove filaments
                    if (filamentIsOutside(filament))
                        stable = true;
                    else
                        otherFilamentVec->push_back(filament);
                }
                iterationCount++;

                // "other" now contains the list of all the filaments that are still available, so swap the vectors and remove the old list
                activeFilamentVec->clear();
                std::swap(activeFilamentVec, otherFilamentVec);
            }
        }

        ZoneScopedN("Recording");
        // now, we do the thing
        for (int t = 1; t < timesteps + 1; t++)
        {
            for (int i = 0; i < numFilamentsIteration; i++)
            {
                activeFilamentVec->emplace_back();
                activeFilamentVec->back().position = source.getPoint();
            }

            for (Filament& filament : *activeFilamentVec)
            {
                // update map
                auto indices = measuredHitProb.metadata.coordinatesToIndices(filament.position.x, filament.position.y);
                size_t index = measuredHitProb.metadata.indexOf(indices);
                GSL_ASSERT(measuredHitProb.metadata.indicesInBounds(indices));
                // mark as updated so it doesn't count multiple filaments in the same timestep
                if (updated[index] < t)
                {
                    hitMap[index]++;
                    updated[index] = t;
                }

                // move active filaments
                moveFilament(filament, indices, deltaTime, noiseSTDev);

                // remove filaments
                if (!filamentIsOutside(filament))
                    otherFilamentVec->push_back(filament);
            }
            activeFilamentVec->clear();
            std::swap(activeFilamentVec, otherFilamentVec);
        }

        // convert the total hit count into relative frequency
        for (int i = 0; i < measuredHitProb.data.size(); i++)
        {
            if (measuredHitProb.occupancy[i] == Occupancy::Free)
                hitMap[i] = hitMap[i] / timesteps;
        }
    }

    Vector2 SimulationSource::getPoint() const
    {
        if (mode == Mode::Point)
            return point;

        Vector2 start = metadata.indicesToCoordinates(nqaNode->origin.x, nqaNode->origin.y, false);
        Vector2 end = metadata.indicesToCoordinates(nqaNode->origin.x + nqaNode->size.x, nqaNode->origin.y + nqaNode->size.y, false);

        Vector2 randP(Utils::uniformRandomF(start.x, end.x), Utils::uniformRandomF(start.y, end.y));
        GSL_ASSERT(randP.x >= start.x && randP.x < end.x && randP.y >= start.y && randP.y < end.y);
        return randP;
    }

    bool Simulations::moveAlongPath(Vector2& currentPosition, const Vector2& end) const
    {
        Vector2Int indexEnd = measuredHitProb.metadata.coordinatesToIndices(end.x, end.y);
        Vector2Int indexOrigin = measuredHitProb.metadata.coordinatesToIndices(currentPosition.x, currentPosition.y);

        if (!measuredHitProb.freeAt(indexOrigin.x, indexOrigin.y))
        {
            return false;
        }

        // try to avoid doing the raycast by looking at the pre-computed visibilityMap
        if (indexOrigin == indexEnd || (measuredHitProb.metadata.indicesInBounds(indexEnd) && measuredHitProb.freeAt(indexEnd.x, indexEnd.y) &&
                                        visibilityMap->isVisible(indexOrigin, indexEnd) == Visibility::Visible))
        {
            currentPosition = end;
            return true;
        }

#define USE_DDA 0
#if USE_DDA
        Vector2 movement = end - currentPosition;
        DDA::_2D::RayCastInfo raycastInfo =
            DDA::_2D::castRay<GSL::Occupancy>(currentPosition, movement, vmath::length(movement),
                                              DDA::_2D::Map<GSL::Occupancy>(measuredHitProb.occupancy, measuredHitProb.metadata.origin,
                                                                            measuredHitProb.metadata.cellSize, measuredHitProb.metadata.dimensions),
                                              [](const GSL::Occupancy& occ)
                                              {
                                                  return occ == GSL::Occupancy::Free;
                                              });
        // This is a completely hacky arbitrary value to try and stop filaments from getting stuck right next to a wall
        // ideally, we should implement a "deflection" instead so they move along the wall a bit rather than stopping dead
        constexpr float wallStoppingProportion = 0.7;
        currentPosition += movement * raycastInfo.distance * wallStoppingProportion;

        return true;
#else
        const auto& metadata = measuredHitProb.metadata;
        bool pathIsFree = true;

        Vector2 vector = end - currentPosition;
        float travelDistance = vmath::length(vector);
        float stepSize = std::min(travelDistance, metadata.cellSize * 0.5f);
        Vector2 increment = vmath::normalized(vector) * stepSize;
        int steps = travelDistance / stepSize;

        int index = 0;
        while (index < steps && pathIsFree)
        {
            currentPosition += increment;
            index++;
            Vector2Int pair = metadata.coordinatesToIndices(currentPosition.x, currentPosition.y);
            pathIsFree = !metadata.indicesInBounds(pair) || measuredHitProb.freeAt(pair.x, pair.y);
            if (!pathIsFree)
                currentPosition -= increment;
        }
        return pathIsFree;
#endif
    }

    void Simulations::makeSimulationImage(const SimulationSource& source)
    {
        std::vector<float> hitMap(measuredHitProb.data.size(), 0.0);
        simulateSourceInPosition(source, hitMap, true, settings.iterationsToRecord, settings.deltaTime,
                                 settings.noiseSTDev);
        displayImage(hitMap);
    }

    void Simulations::displayImage(const std::vector<float>& hitMap, const std::string& imageName) const
    {
        cv::Mat asImage(hitMap);
        asImage = asImage.reshape(1, measuredHitProb.metadata.dimensions.y);
        if (settings.blurSigmaX > 0 || settings.blurSigmaY > 0)
        {
            blurHitMap(asImage);
        }

        cv::Mat inColor;
        cv::cvtColor(asImage, inColor, cv::COLOR_GRAY2BGR);

        for (int j = 0; j < measuredHitProb.metadata.dimensions.y; j++)
        {
            for (int i = 0; i < measuredHitProb.metadata.dimensions.x; i++)
            {
                if (!measuredHitProb.freeAt(i, j))
                    inColor.at<cv::Vec3f>(j, i) = cv::Vec3f(0, 0, 1);
            }
        }

#if 0
        cv::flip(inColor, inColor, 0);
        inColor *= 255;
        cv::imwrite(fmt::format("{}.png", imageName), inColor);
        GSL_WARN("hitMap image saved");
#else
        cv::flip(inColor, inColor, 0);
        cv::Mat resized;
        cv::resize(inColor, resized, cv::Size(inColor.size[1] * 10, inColor.size[0] * 10), 0, 0, cv::INTER_NEAREST);
        cv::imshow(imageName, resized);
        cv::waitKey();
        cv::destroyAllWindows();
#endif
    }

    void Simulations::blurHitMap(cv::Mat& asImage) const
    {
        cv::GaussianBlur(asImage, asImage, cv::Size(0, 0), settings.blurSigmaX, settings.blurSigmaY);
        // divide by the blurred mask to correct the edges always getting lower
        // TODO measure performance of doing this repeatedly. Is it worth it to pre-compute and only redo it if the blur sigma has changed?
        cv::Mat blurredMask;
        cv::GaussianBlur(freeSpaceMask, blurredMask, cv::Size(0, 0), settings.blurSigmaX, settings.blurSigmaY);

        for (int i = 0; i < measuredHitProb.metadata.dimensions.y; i++)
            for (int j = 0; j < measuredHitProb.metadata.dimensions.x; j++)
                if (blurredMask.at<float>(i, j) != 0)
                    asImage.at<float>(i, j) = Utils::clamp(asImage.at<float>(i, j) / blurredMask.at<float>(i, j), 0, 1);
    }

} // namespace GSL::PMFS_internal
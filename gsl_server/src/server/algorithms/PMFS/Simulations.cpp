#include <gsl_server/algorithms/PMFS/internal/Simulations.hpp>
#include <gsl_server/algorithms/PMFS/PMFS.hpp>
#include <gsl_server/algorithms/PMFS/PMFSLib.hpp>
#include <gsl_server/Utils/Math.hpp>
#include <gsl_server/Utils/Time.hpp>
#include <gsl_server/core/Logging.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

//TODO make it so you don't need this file
#include <tracy/Tracy.hpp>

namespace GSL::PMFS_internal
{
    namespace NQA = Utils::NQA;
    using HashSet = std::unordered_set<Vector2Int>;

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

    // create the Quadtree
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
    }

    void Simulations::updateSourceProbability(float refineFraction)
    {
        ZoneScoped;
        GSL_INFO_COLOR(fmt::terminal_color::yellow, "Started simulations. Might take a while!");
        Utils::Time::Stopwatch stopwatch;
        std::vector<NQA::Node> localCopyLeaves = QTleaves;

        // first, coarse simulation based on the quadtree decomposition of the map

        // store the score of each region to figure out which ones are worth subdividing for finer simulation
        struct SimulationResult
        {
            float score;
            NQA::Node* leaf;
        };
        std::vector<SimulationResult> scores(localCopyLeaves.size());

        struct VarianceCalculationData
        {
            double mean = 0;
            double weight_sum = 0;
            double weight_squared_sum = 0;
            double variance = 0;
        };
        std::vector<VarianceCalculationData> varianceCalculationData(measuredHitProb.data.size());

        int numberOfSimulations = 0;
        #pragma omp parallel for schedule(dynamic)
        for (int leafIndex = 0; leafIndex < localCopyLeaves.size(); leafIndex++)
        {
            NQA::Node* node = &localCopyLeaves[leafIndex];
            if (node->value != 1)
                continue;
            SimulationSource source(node, measuredHitProb.metadata);

            std::vector<float> hitMap(measuredHitProb.data.size(), 0.0);
            simulateSourceInPosition(source, hitMap, true, settings.maxWarmupIterations, settings.iterationsToRecord, settings.deltaTime, settings.noiseSTDev);

            double diff = weightedDifference(measuredHitProb, hitMap);
            scores[leafIndex].score = diff;
            scores[leafIndex].leaf = node;

            // assign this probability to all cells that fall inside this region
            for (int cellI = node->origin.x; cellI < (node->origin.x + node->size.x); cellI++)
            {
                for (int cellJ = node->origin.y; cellJ < (node->origin.y + node->size.y); cellJ++)
                {
                    sourceProb.dataAt(cellI, cellJ) = diff;
                }
            }

            // update the information for the variance calulation
            #pragma omp critical
            {
                numberOfSimulations++;
                for (int cell = 0; cell < hitMap.size(); cell++)
                {
                    auto& var = varianceCalculationData[cell];
                    weighted_incremental_variance(hitMap[cell], diff, var.mean, var.weight_sum, var.weight_squared_sum, var.variance);
                }
            }
        }

        // update the variance thing
        #pragma omp parallel for
        for (int cellI = 0; cellI < measuredHitProb.data.size(); cellI++)
        {
            if (measuredHitProb.occupancy[cellI] == Occupancy::Free)
                varianceOfHitProb[cellI] =
                    varianceCalculationData[cellI].variance / varianceCalculationData[cellI].weight_sum;
        }

        // now, finer simulation where it is deemed relevant

        int numberOfLevelsSimulated = 1;
        while (scores.size() > 0)
        {
            std::sort(scores.begin(), scores.end(), [](SimulationResult result1, SimulationResult result2) { return result1.score > result2.score; });

            // subdivide the good cells and add the children to the list of cells to simulate
            std::vector<SimulationResult> newLevel;
            for (int leafIndex = 0; leafIndex < scores.size() * refineFraction; leafIndex++)
            {
                NQA::Node* leaf = scores[leafIndex].leaf;
                bool hasChildren = leaf->subdivide();
                if (hasChildren)
                {
                    for (int childI = 0; childI < 4; childI++)
                    {
                        if (leaf->children[childI])
                        {
                            newLevel.push_back({0, (leaf->children[childI]).get()});
                        }
                    }
                }
            }
            scores = newLevel;

            numberOfLevelsSimulated++;
            numberOfSimulations += scores.size();

            // run the simulations of the new level and get scores for each node
            #pragma omp parallel for schedule(dynamic)
            for (int leafIndex = 0; leafIndex < scores.size(); leafIndex++)
            {
                NQA::Node* node = scores[leafIndex].leaf;
                if (node->value != 1)
                    continue;
                SimulationSource source(node, measuredHitProb.metadata);

                std::vector<float> hitMap(measuredHitProb.data.size(), 0.0);
                simulateSourceInPosition(source, hitMap, true, settings.maxWarmupIterations, settings.iterationsToRecord, settings.deltaTime,
                                         settings.noiseSTDev);

                double diff = weightedDifference(measuredHitProb, hitMap);
                scores[leafIndex].score = diff;

                // assign this probability to all cells that fall inside this region
                for (int cellI = node->origin.x; cellI < (node->origin.x + node->size.x); cellI++)
                {
                    for (int cellJ = node->origin.y; cellJ < (node->origin.y + node->size.y); cellJ++)
                    {
                        sourceProb.dataAt(cellI,cellJ) = diff;
                    }
                }
            }
        }

        GSL_INFO("Number of levels in the simulation: {0}", numberOfLevelsSimulated);
        GSL_INFO("Total number of simulations: {0}", numberOfSimulations);

        PMFSLib::normalizeSourceProb(sourceProb);
        GSL_INFO("Time ellapsed in simulation = {} s", stopwatch.ellapsed());
    }

    double Simulations::weightedDifference(const Grid<HitProbability>& hitRandomVariable, const std::vector<float>& hitMap) const
    {
        ZoneScoped;
        double total = 1;
        for (int i = 0; i < measuredHitProb.data.size(); i++)
        {
            if (measuredHitProb.occupancy[i] != Occupancy::Free)
                continue;
            double measured = Utils::logOddsToProbability(measuredHitProb.data[i].logOdds);
            const double& simulated = hitMap[i];
            double val = Utils::lerp(1, (1 - std::abs(measured - simulated) * settings.sourceDiscriminationPower),
                                        measuredHitProb.data[i].confidence);
            total *= val;
            GSL_ASSERT(!std::isnan(total));
        }
        return total;
    }

    void Simulations::moveFilament(Filament& filament, Vector2Int& indices, float deltaTime, float noiseSTDev) const
    {
        ZoneScoped;

        Vector2 velocity = wind.dataAt(indices.x, indices.y) +
                                Vector2(Utils::randomFromGaussian(0, noiseSTDev), Utils::randomFromGaussian(0, noiseSTDev));
        
        Vector2 newPos = filament.position + deltaTime * velocity;
        moveAlongPath(filament.position, newPos);
    }

    //moves exactly one cell at a time, independently of the magnitude of the vector. For warmup.
    void Simulations::moveFilamentDiscretePosition(Filament& filament, Vector2Int& indices, float noiseSTDev) const
    {
        Vector2 velocity = wind.dataAt(indices.x, indices.y) +
                                             Vector2(Utils::randomFromGaussian(0, noiseSTDev), Utils::randomFromGaussian(0, noiseSTDev));
        
        float angle = atan2(velocity.x, velocity.y);
        Vector2Int next = indices;
        if(angle >= 0)
        {

            if(angle < M_PI/8)
                next.x += 1;
            else if(angle < M_PI/4 + M_PI/8)
            {
                next.x += 1;
                next.y += 1;
            }
            else if(angle < M_PI/2 + M_PI/8)
                next.y += 1;
            else if (angle < 3*M_PI/4 + M_PI/8)
            {
                next.x -= 1;
                next.y += 1;
            }
            else
                next.x -= 1;
        }
        else
        {
            angle = -angle;

            if(angle < M_PI/8)
                next.x += 1;
            else if(angle < M_PI/4 + M_PI/8)
            {
                next.x += 1;
                next.y -= 1;
            }
            else if(angle < M_PI/2 + M_PI/8)
                next.y -= 1;
            else if (angle < 3*M_PI/4 + M_PI/8)
            {
                next.x -= 1;
                next.y -= 1;
            }
            else
                next.x -= 1;
        }
        
        if(measuredHitProb.metadata.indicesInBounds(next) || measuredHitProb.freeAt(next.x, next.y))
            filament.position = measuredHitProb.metadata.indexToCoordinates(next.x, next.y);
    }

    bool Simulations::filamentIsOutside(const Filament& filament) const
    {
        ZoneScoped;
        Vector2Int newIndices = measuredHitProb.metadata.coordinatesToIndex(filament.position.x, filament.position.y);
        return !measuredHitProb.metadata.indicesInBounds(newIndices);
    }

    void Simulations::simulateSourceInPosition(const SimulationSource& source, std::vector<float>& hitMap, bool warmup, int warmupLimit,
                                      int timesteps, float deltaTime, float noiseSTDev) const
    {
        ZoneScoped;

        constexpr int numFilamentsIteration = 5;
        std::vector<Filament> filaments(warmupLimit * numFilamentsIteration + timesteps * numFilamentsIteration);

        std::vector<uint16_t> updated(hitMap.size(), 0); //index of the last iteration in which this cell was updated, to avoid double-counting

        int lastActivated = 0;

        // warm-up: we don't want to start recording frequency of hits until the shape of the plume has stabilized. Wait until a filament exits the
        // environment through an outlet, or a maximum number of steps
        {
            bool stable = false;
            int count = 0;
            while (!stable && count < warmupLimit)
            {
                for (int i = 0; i < numFilamentsIteration; i++)
                {
                    filaments[lastActivated].position = source.getPoint();
                    filaments[lastActivated].active = true;
                    lastActivated = (lastActivated + 1) % filaments.size();
                }

                for (Filament& filament : filaments)
                {
                    if (!filament.active)
                        continue;
                    auto indices = measuredHitProb.metadata.coordinatesToIndex(filament.position.x, filament.position.y);

                    // move active filaments

#define WARMUP_DISCRETE_MOVEMENT 0 
#if WARMUP_DISCRETE_MOVEMENT
                    //TODO this seems to work a fair bit better when the source is out of the main airflow current, but causes some problems in other places
                    //TODO should run some testing with different weights for the noise and maybe selective application of the discrete movement based on wind speed
                    //moveFilamentDiscretePosition(filament, indices, noiseSTDev * 0.2);
#else
                    moveFilament(filament, indices, deltaTime * 2, noiseSTDev);
#endif

                    // remove filaments
                    if (filamentIsOutside(filament))
                    {
                        filament.active = false;
                        stable = true;
                        break;
                    }
                }
                count++;
            }
        }

        // now, we do the thing
        for (int t = 1; t < timesteps+1; t++)
        {
            // spawn new ones
            for (int i = 0; i < numFilamentsIteration; i++)
            {
                filaments[lastActivated].position = source.getPoint();
                filaments[lastActivated].active = true;
                lastActivated = (lastActivated + 1) % filaments.size();
            }

            for (Filament& filament : filaments)
            {
                if (!filament.active)
                    continue;

                // update map
                auto indices = measuredHitProb.metadata.coordinatesToIndex(filament.position.x, filament.position.y);
                size_t index = measuredHitProb.metadata.indexOf(indices.x, indices.y);
                // mark as updated so it doesn't count multiple filaments in the same timestep
                if (updated[index] < t)
                {
                    hitMap[index]++;
                    updated[index] = t;
                }

                // move active filaments
                moveFilament(filament, indices, deltaTime, noiseSTDev);

                // remove filaments
                if (filamentIsOutside(filament))
                {
                    filament.active = false;
                }
            }
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

        Vector2 start = metadata.indexToCoordinates(nqaNode->origin.x, nqaNode->origin.y, false);
        Vector2 end = metadata.indexToCoordinates(nqaNode->origin.x + nqaNode->size.x, nqaNode->origin.y + nqaNode->size.y, false);

        Vector2 randP(Utils::uniformRandom(start.x, end.x), Utils::uniformRandom(start.y, end.y));

        return randP;
    }

    bool Simulations::moveAlongPath(Vector2& currentPosition, const Vector2& end) const
    {
        Vector2Int indexEnd = measuredHitProb.metadata.coordinatesToIndex(end.x, end.y);
        Vector2Int indexOrigin = measuredHitProb.metadata.coordinatesToIndex(currentPosition.x, currentPosition.y);

        if (!measuredHitProb.freeAt(indexOrigin.x, indexOrigin.y))
        {
            return false;
        }

        if(visibilityMap)
        {
            if (visibilityMap->isVisible(indexOrigin, indexEnd) == Visibility::Visible)
            {
                currentPosition = end;
                return true;
            }
        }

        bool pathIsFree = true;
        Vector2 vector = end - currentPosition;
        Vector2 increment = glm::normalize(vector) * (measuredHitProb.metadata.cellSize);
        int steps = glm::length(vector) / (measuredHitProb.metadata.cellSize);

        int index = 0;
        while (index < steps && pathIsFree)
        {
            currentPosition += increment;
            index++;
            Vector2Int pair = measuredHitProb.metadata.coordinatesToIndex(currentPosition.x, currentPosition.y);
            pathIsFree = measuredHitProb.metadata.indicesInBounds(pair) || measuredHitProb.freeAt(pair.x, pair.y);
            if (!pathIsFree)
                currentPosition -= increment;
        }

        return pathIsFree;
    }

    void Simulations::printImage(const SimulationSource& source)
    {
        std::vector<float> hitMap(measuredHitProb.data.size(), 0.0);
        simulateSourceInPosition(source, hitMap, false, settings.maxWarmupIterations, settings.iterationsToRecord, settings.deltaTime, settings.noiseSTDev);

        cv::Mat image(cv::Size(measuredHitProb.metadata.width, measuredHitProb.metadata.height), CV_32FC3, cv::Scalar(0, 0, 0));

        for (int i = 0; i < measuredHitProb.metadata.height; i++)
        {
            for (int j = 0; j < measuredHitProb.metadata.width; j++)
            {
                if (measuredHitProb.freeAt(i,j))
                {
                    float v = hitMap[measuredHitProb.metadata.indexOf(i,j)] * 255;
                    image.at<cv::Vec3f>(hitMap.size() - 1 - i, j) = cv::Vec3f(v, v, v);
                }
                else
                    image.at<cv::Vec3f>(hitMap.size() - 1 - i, j) = cv::Vec3f(0, 0, 255);
            }
        }

        std::string filename;
        if (source.mode == SimulationSource::Mode::Quadtree)
            filename = fmt::format("leaf_{}.png", source.nqaNode->origin);
        else
            filename = fmt::format("point_{}.png", source.point);

        cv::imwrite(filename, image);

        GSL_WARN("hitMap image saved");
    }

} // namespace GSL::PMFS_internal
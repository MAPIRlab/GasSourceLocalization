#include <gsl_server/algorithms/PMFS/internal/Simulations.hpp>
#include <gsl_server/algorithms/PMFS/PMFS.hpp>
#include <gsl_server/Utils/Math.hpp>
#include <gsl_server/Utils/Time.hpp>
#include <gsl_server/core/logging.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace GSL::PMFS_internal
{
    namespace NQA = Utils::NQA;
    using hashSet = std::unordered_set<Vector2Int>;

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

    void Simulations::updateSourceProbability(float refineFraction)
    {
        GSL_INFO_COLOR(fmt::terminal_color::yellow, "Started simulations. Might take a while!");
        Utils::Time::Stopwatch stopwatch;
        const Settings::SimulationSettings& setts = pmfs->settings.simulation;
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
        std::vector<std::vector<VarianceCalculationData>> varianceCalculationData(pmfs->grid.size(),
                                                                                  std::vector<VarianceCalculationData>(pmfs->grid[0].size()));

        int numberOfSimulations = 0;
        #pragma omp parallel for
        for (int leafIndex = 0; leafIndex < localCopyLeaves.size(); leafIndex++)
        {
            NQA::Node* node = &localCopyLeaves[leafIndex];
            if (node->value != 1)
                continue;
            SimulationSource source(node, pmfs);

            std::vector<std::vector<float>> hitMap(pmfs->grid.size(), std::vector<float>(pmfs->grid[0].size(), 0.0));
            simulateSourceInPosition(source, hitMap, true, setts.maxWarmupIterations, setts.iterationsToRecord, setts.deltaTime, setts.noiseSTDev);

            double diff = weightedDifference(pmfs->grid, hitMap);
            scores[leafIndex].score = diff;
            scores[leafIndex].leaf = node;

            // assign this probability to all cells that fall inside this region
            for (int cellI = node->origin.x; cellI < (node->origin.x + node->size.x); cellI++)
            {
                for (int cellJ = node->origin.y; cellJ < (node->origin.y + node->size.y); cellJ++)
                {
                    pmfs->grid[cellI][cellJ].sourceProbability = diff;
                }
            }

            // update the information for the variance calulation
            #pragma omp critical
            {
                numberOfSimulations++;
                for (int cellI = 0; cellI < hitMap.size(); cellI++)
                {
                    for (int cellJ = 0; cellJ < hitMap[0].size(); cellJ++)
                    {
                        auto& var = varianceCalculationData[cellI][cellJ];
                        weighted_incremental_variance(hitMap[cellI][cellJ], diff, var.mean, var.weight_sum, var.weight_squared_sum, var.variance);
                    }
                }
            }
        }

        // update the variance thing
        #pragma omp parallel for collapse(2)
        for (int cellI = 0; cellI < pmfs->grid.size(); cellI++)
        {
            for (int cellJ = 0; cellJ < pmfs->grid[0].size(); cellJ++)
            {
                if (pmfs->grid[cellI][cellJ].free)
                    varianceOfHitProb[cellI][cellJ] =
                        varianceCalculationData[cellI][cellJ].variance / varianceCalculationData[cellI][cellJ].weight_sum;
            }
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
            #pragma omp parallel for
            for (int leafIndex = 0; leafIndex < scores.size(); leafIndex++)
            {
                NQA::Node* node = scores[leafIndex].leaf;
                if (node->value != 1)
                    continue;
                SimulationSource source(node, pmfs);

                std::vector<std::vector<float>> hitMap(pmfs->grid.size(), std::vector<float>(pmfs->grid[0].size(), 0.0));
                simulateSourceInPosition(source, hitMap, true, setts.maxWarmupIterations, setts.iterationsToRecord, setts.deltaTime,
                                         setts.noiseSTDev);

                double diff = weightedDifference(pmfs->grid, hitMap);
                scores[leafIndex].score = diff;

                // assign this probability to all cells that fall inside this region
                for (int cellI = node->origin.x; cellI < (node->origin.x + node->size.x); cellI++)
                {
                    for (int cellJ = node->origin.y; cellJ < (node->origin.y + node->size.y); cellJ++)
                    {
                        pmfs->grid[cellI][cellJ].sourceProbability = diff;
                    }
                }
            }
        }

        GSL_INFO("Number of levels in the simulation: {0}", numberOfLevelsSimulated);
        GSL_INFO("Total number of simulations: {0}", numberOfSimulations);

        pmfs->normalizeSourceProb(pmfs->grid);
        GSL_INFO("Time ellapsed in simulation = {} s", stopwatch.ellapsed());
    }

    double Simulations::weightedDifference(const std::vector<std::vector<Cell>>& grid, const std::vector<std::vector<float>>& hitMap)
    {
        double total = 1;
        for (int i = 0; i < grid.size(); i++)
        {
            for (int j = 0; j < grid[0].size(); j++)
            {
                if (!grid[i][j].free)
                    continue;
                double measured = Utils::logOddsToProbability(grid[i][j].hitProbability.logOdds);
                const double& simulated = hitMap[i][j];
                double val = Utils::lerp(1, (1 - std::abs(measured - simulated) * pmfs->settings.simulation.sourceDiscriminationPower),
                                         grid[i][j].hitProbability.confidence);
                total *= val;
                GSL_ASSERT(!std::isnan(total));
            }
        }
        return total;
    }

    void Simulations::moveFilament(Filament& filament, Vector2Int& indices, float deltaTime, float noiseSTDev)
    {
        Vector2 velocity = pmfs->estimatedWindVectors[indices.x][indices.y] +
                                             Vector2(Utils::randomFromGaussian(0, noiseSTDev), Utils::randomFromGaussian(0, noiseSTDev));
        
        Vector2 newPos = filament.position + deltaTime * velocity;
        moveAlongPath(filament.position, newPos);
    }

    //moves exactly one cell at a time, independently of the magnitude of the vector. For warmup.
    void Simulations::moveFilamentDiscretePosition(Filament& filament, Vector2Int& indices, float noiseSTDev)
    {
        Vector2 velocity = pmfs->estimatedWindVectors[indices.x][indices.y] +
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
        
        bool isOutside = next.x < 0 || next.x >= pmfs->grid.size() || next.y < 0 || next.y >= pmfs->grid[0].size();
        if(isOutside || pmfs->grid[next.x][next.y].free)
            filament.position = pmfs->gridMetaData.indexToCoordinates(next.x, next.y);
    }

    bool Simulations::filamentIsOutside(Filament& filament)
    {

        Vector2Int newIndices = pmfs->gridMetaData.coordinatesToIndex(filament.position.x, filament.position.y);

        return (newIndices.x < 0 || newIndices.x >= pmfs->grid.size() || newIndices.y < 0 || newIndices.y >= pmfs->grid[0].size());
    }

    void Simulations::simulateSourceInPosition(const SimulationSource& source, std::vector<std::vector<float>>& hitMap, bool warmup, int warmupLimit,
                                               int timesteps, float deltaTime, float noiseSTDev)
    {

        constexpr int numFilamentsIteration = 5;
        std::vector<Filament> filaments(warmupLimit * numFilamentsIteration + timesteps * numFilamentsIteration);

        std::vector<std::vector<uint16_t>> updated(hitMap.size(), std::vector<uint16_t>(hitMap[0].size(), 0));

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
                    auto indices = pmfs->gridMetaData.coordinatesToIndex(filament.position.x, filament.position.y);

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
        for (int t = 0; t < timesteps; t++)
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
                auto indices = pmfs->gridMetaData.coordinatesToIndex(filament.position.x, filament.position.y);

                // mark as updated so it doesn't count multiple filaments in the same timestep
                if (updated[indices.x][indices.y] < t)
                {
                    hitMap[indices.x][indices.y]++;
                    updated[indices.x][indices.y] = t;
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
        for (int i = 0; i < pmfs->grid.size(); i++)
        {
            for (int j = 0; j < pmfs->grid[0].size(); j++)
            {
                if (pmfs->grid[i][j].free)
                    hitMap[i][j] = hitMap[i][j] / timesteps;
            }
        }
    }

    Vector2 SimulationSource::getPoint() const
    {
        if (mode == Mode::Point)
            return point;

        Vector2 start = pmfs->gridMetaData.indexToCoordinates(nqaNode->origin.x, nqaNode->origin.y, false);
        Vector2 end = pmfs->gridMetaData.indexToCoordinates(nqaNode->origin.x + nqaNode->size.x, nqaNode->origin.y + nqaNode->size.y, false);

        Vector2 randP(Utils::uniformRandom(start.x, end.x), Utils::uniformRandom(start.y, end.y));

        GSL_ASSERT(pmfs->isPointInsideMapBounds(randP));
        return randP;
    }

    bool Simulations::moveAlongPath(Vector2& beginning, const Vector2& end)
    {

        Vector2Int indexEnd = pmfs->gridMetaData.coordinatesToIndex(end.x, end.y);
        Vector2Int indexOrigin = pmfs->gridMetaData.coordinatesToIndex(beginning.x, beginning.y);

        if (!pmfs->grid[indexOrigin.x][indexOrigin.y].free)
        {
            return false;
        }

        hashSet& set = pmfs->visibilityMap.at(indexOrigin);
        if (set.find(indexEnd) != set.end())
        {
            beginning = end;
            return true;
        }

        bool pathIsFree = true;
        Vector2 vector = end - beginning;
        Vector2 increment = glm::normalize(vector) * (pmfs->gridMetaData.cellSize);
        int steps = glm::length(vector) / (pmfs->gridMetaData.cellSize);

        int index = 0;
        while (index < steps && pathIsFree)
        {
            beginning += increment;
            index++;
            Vector2Int pair = pmfs->gridMetaData.coordinatesToIndex(beginning.x, beginning.y);
            pathIsFree = !pmfs->indicesInBounds(pair) || pmfs->grid[pair.x][pair.y].free;
            if (!pathIsFree)
                beginning -= increment;
        }

        return pathIsFree;
    }

    void Simulations::printImage(const SimulationSource& source)
    {
        const PMFS_internal::Settings::SimulationSettings& setts = pmfs->settings.simulation;
        std::vector<std::vector<float>> hitMap(pmfs->grid.size(), std::vector<float>(pmfs->grid[0].size(), 0.0));
        simulateSourceInPosition(source, hitMap, false, setts.maxWarmupIterations, setts.iterationsToRecord, setts.deltaTime, setts.noiseSTDev);

        cv::Mat image(cv::Size(hitMap[0].size(), hitMap.size()), CV_32FC3, cv::Scalar(0, 0, 0));

        for (int i = 0; i < hitMap.size(); i++)
        {
            for (int j = 0; j < hitMap[0].size(); j++)
            {
                if (pmfs->grid[i][j].free)
                    image.at<cv::Vec3f>(hitMap.size() - 1 - i, j) = cv::Vec3f((hitMap[i][j]) * 255, (hitMap[i][j]) * 255, (hitMap[i][j]) * 255);
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
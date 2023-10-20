#include <algorithms/PMFS/PMFS.h>
namespace PMFS
{

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
        spdlog::info(fmt::format(fmt::fg(fmt::color::orange), "Started simulations. Might take a while!"));
        Utils::Time::Stopwatch stopwatch;
        const GridSettings::SimulationSettings& setts = grid->settings.simulation;
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
        std::vector<std::vector<VarianceCalculationData>> varianceCalculationData(grid->cells.size(),
                                                                                  std::vector<VarianceCalculationData>(grid->cells[0].size()));

        int numberOfSimulations = 0;
#pragma omp parallel for
        for (int leafIndex = 0; leafIndex < localCopyLeaves.size(); leafIndex++)
        {
            NQA::Node* node = &localCopyLeaves[leafIndex];
            if (node->value != 1)
                continue;
            SimulationSource source(node, grid);

            std::vector<std::vector<float>> hitMap(grid->cells.size(), std::vector<float>(grid->cells[0].size(), 0.0));
            simulateSourceInPosition(source, hitMap, true, setts.maxWarmupIterations, setts.iterationsToRecord, setts.deltaTime, setts.noiseSTDev);

            double diff = weightedDifference(grid->cells, hitMap);
            scores[leafIndex].score = diff;
            scores[leafIndex].leaf = node;

            // assign this probability to all cells that fall inside this region
            for (int cellI = node->origin.x; cellI < (node->origin.x + node->size.x); cellI++)
            {
                for (int cellJ = node->origin.y; cellJ < (node->origin.y + node->size.y); cellJ++)
                {
                    grid->cells[cellI][cellJ].sourceProbability = diff;
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
        for (int cellI = 0; cellI < grid->cells.size(); cellI++)
        {
            for (int cellJ = 0; cellJ < grid->cells[0].size(); cellJ++)
            {
                if (grid->cells[cellI][cellJ].free)
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
                SimulationSource source(node, grid);

                std::vector<std::vector<float>> hitMap(grid->cells.size(), std::vector<float>(grid->cells[0].size(), 0.0));
                simulateSourceInPosition(source, hitMap, true, setts.maxWarmupIterations, setts.iterationsToRecord, setts.deltaTime,
                                         setts.noiseSTDev);

                double diff = weightedDifference(grid->cells, hitMap);
                scores[leafIndex].score = diff;

                // assign this probability to all cells that fall inside this region
                for (int cellI = node->origin.x; cellI < (node->origin.x + node->size.x); cellI++)
                {
                    for (int cellJ = node->origin.y; cellJ < (node->origin.y + node->size.y); cellJ++)
                    {
                        grid->cells[cellI][cellJ].sourceProbability = diff;
                    }
                }
            }
        }

        spdlog::info("Number of levels in the simulation: {0}", numberOfLevelsSimulated);
        spdlog::info("Total number of simulations: {0}", numberOfSimulations);

        grid->normalizeSourceProb(grid->cells);
        spdlog::info("Time ellapsed in simulation = {} s", stopwatch.ellapsed());
    }

    double Simulations::weightedDifference(const std::vector<std::vector<Cell>>& cells, const std::vector<std::vector<float>>& hitMap)
    {
        double total = 1;
        for (int i = 0; i < cells.size(); i++)
        {
            for (int j = 0; j < cells[0].size(); j++)
            {
                if (!cells[i][j].free)
                    continue;
                double measured = Utils::logOddsToProbability(cells[i][j].hitProbability.logOdds);
                const double& simulated = hitMap[i][j];
                double val = Utils::lerp(1, (1 - std::abs(measured - simulated) * grid->settings.simulation.sourceDiscriminationPower),
                                         cells[i][j].hitProbability.confidence);
                total *= val;
                GSL_ASSERT(!std::isnan(total));
            }
        }
        return total;
    }

    void Simulations::moveFilament(Filament& filament, Vector2Int& indices, float deltaTime, float noiseSTDev)
    {

        Utils::Vector2 newPos =
            filament.position + deltaTime * (grid->estimatedWindVectors[indices.x][indices.y] +
                                             Utils::Vector2(Utils::randomFromGaussian(0, noiseSTDev), Utils::randomFromGaussian(0, noiseSTDev)));

        moveAlongPath(filament.position, newPos);
    }

    bool Simulations::filamentIsOutside(Filament& filament)
    {

        Vector2Int newIndices = grid->coordinatesToIndex(filament.position.x, filament.position.y);

        return (newIndices.x < 0 || newIndices.x >= grid->cells.size() || newIndices.y < 0 || newIndices.y >= grid->cells[0].size());
    }

    void Simulations::simulateSourceInPosition(const SimulationSource& source, std::vector<std::vector<float>>& hitMap, bool warmup, int warmupLimit,
                                               int timesteps, float deltaTime, float noiseSTDev)
    {

        const int numFilamentsIteration = 5;
        std::vector<Filament> filaments(warmupLimit * numFilamentsIteration + timesteps * numFilamentsIteration);

        std::vector<std::vector<bool>> updated(hitMap.size(), std::vector<bool>(hitMap[0].size(), false));

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
                    auto indices = grid->coordinatesToIndex(filament.position.x, filament.position.y);

                    // move active filaments
                    moveFilament(filament, indices, deltaTime * 2, noiseSTDev);

                    // remove filaments
                    if (filamentIsOutside(filament))
                    {
                        filament.active = false;
                        if (warmup)
                        {
                            stable = true;
                            break;
                        }
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
                auto indices = grid->coordinatesToIndex(filament.position.x, filament.position.y);

                // mark as updated so it doesn't count multiple filaments in the same timestep
                if (!updated[indices.x][indices.y])
                {
                    hitMap[indices.x][indices.y]++;
                    updated[indices.x][indices.y] = true;
                }

                // move active filaments
                moveFilament(filament, indices, deltaTime, noiseSTDev);

                // remove filaments
                if (filamentIsOutside(filament))
                {
                    filament.active = false;
                }
            }

            // reset the updated status
            for (int i = 0; i < updated.size(); i++)
            {
                for (int j = 0; j < updated[0].size(); j++)
                {
                    updated[i][j] = false;
                }
            }
        }

        // convert the total hit count into relative frequency
        for (int i = 0; i < grid->cells.size(); i++)
        {
            for (int j = 0; j < grid->cells[0].size(); j++)
            {
                if (grid->cells[i][j].free)
                    hitMap[i][j] = hitMap[i][j] / timesteps;
            }
        }
    }

    void Simulations::compareRefineFractions()
    {
        spdlog::warn("COMPARING THE SIMULATION RESULTS FOR SEVERAL REFINE FRACTIONS. THIS IS ONLY FOR EVALUATION. FOR ACTUAL USE OF THE ALGORITHM, "
                     "CALL updateSourceProbability() DIRECTLY");
        Utils::Time::Clock clock;

        std::ofstream outfile;
        outfile.open("/home/pepe/catkin_ws/sim_comparisons", std::ios_base::app);
        outfile << grid->iterationsCounter << "\n";

        std::vector<std::vector<double>> fullprob(grid->cells.size(), std::vector<double>(grid->cells[0].size()));
        std::vector<std::vector<double>> testedprob = fullprob;

        auto logResult = [&](float fraction)
        {
            Utils::Time::TimePoint start = clock.now();
            updateSourceProbability(fraction);
            for (int i = 0; i < grid->cells.size(); i++)
            {
                for (int j = 0; j < grid->cells[0].size(); j++)
                {
                    testedprob[i][j] = grid->cells[i][j].sourceProbability;
                }
            }
            outfile << fmt::format("{} -> time:{}, KLD:{}\n", fraction, Utils::Time::toSeconds(clock.now() - start),
                                   Utils::KLD(fullprob, testedprob));
        };

        Utils::Time::TimePoint start = clock.now();
        updateSourceProbability(1);
        for (int i = 0; i < grid->cells.size(); i++)
        {
            for (int j = 0; j < grid->cells[0].size(); j++)
            {
                fullprob[i][j] = grid->cells[i][j].sourceProbability;
            }
        }
        outfile << fmt::format("1 -> time:{}, KLD:0\n", Utils::Time::toSeconds(clock.now() - start));

        logResult(0.5);
        logResult(0.25);
        logResult(0.1);
        logResult(0.01);

        outfile.close();
    }

    Utils::Vector2 SimulationSource::getPoint() const
    {
        if (mode == Mode::Point)
            return point;

        Utils::Vector2 start = grid->indexToCoordinates(nqaNode->origin.x, nqaNode->origin.y, false);
        Utils::Vector2 end = grid->indexToCoordinates(nqaNode->origin.x + nqaNode->size.x, nqaNode->origin.y + nqaNode->size.y, false);

        Utils::Vector2 randP(Utils::uniformRandom(start.x, end.x), Utils::uniformRandom(start.y, end.y));

        GSL_ASSERT(grid->isPointInsideMapBounds(randP));
        return randP;
    }

    bool Simulations::moveAlongPath(Utils::Vector2& beginning, const Utils::Vector2& end)
    {

        Vector2Int indexEnd = grid->coordinatesToIndex(end.x, end.y);
        Vector2Int indexOrigin = grid->coordinatesToIndex(beginning.x, beginning.y);

        if (!grid->cells[indexOrigin.x][indexOrigin.y].free)
        {
            return false;
        }

        hashSet& set = grid->visibilityMap.at(indexOrigin);
        if (set.find(indexEnd) != set.end())
        {
            beginning = end;
            return true;
        }

        bool pathIsFree = true;
        Utils::Vector2 vector = end - beginning;
        Utils::Vector2 increment = vector.normalized() * (grid->settings.scale * grid->map_.info.resolution);
        int steps = vector.norm() / (grid->settings.scale * grid->map_.info.resolution);

        int index = 0;
        while (index < steps && pathIsFree)
        {
            beginning += increment;
            index++;
            Vector2Int pair = grid->coordinatesToIndex(beginning.x, beginning.y);
            pathIsFree = !grid->indicesInBounds(pair) || grid->cells[pair.x][pair.y].free;
            if (!pathIsFree)
                beginning -= increment;
        }

        return pathIsFree;
    }

    void Simulations::printImage(const SimulationSource& source)
    {
        const GridSettings::SimulationSettings& setts = grid->settings.simulation;
        std::vector<std::vector<float>> hitMap(grid->cells.size(), std::vector<float>(grid->cells[0].size(), 0.0));
        simulateSourceInPosition(source, hitMap, false, setts.maxWarmupIterations, setts.iterationsToRecord, setts.deltaTime, setts.noiseSTDev);

        cv::Mat image(cv::Size(hitMap[0].size(), hitMap.size()), CV_32FC3, cv::Scalar(0, 0, 0));

        for (int i = 0; i < hitMap.size(); i++)
        {
            for (int j = 0; j < hitMap[0].size(); j++)
            {
                if (grid->cells[i][j].free)
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

        spdlog::warn("hitMap image saved");
    }

} // namespace PMFS
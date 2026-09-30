#include "EulerianSimulation.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
#include <tracy/Tracy.hpp>

namespace GSL
{

    void EulerianSimulation::Run(std::vector<float>& gasMap, float lowerThr)
    {
        outlets->concentrationExitingDoorway.resize(outlets->enabled.size(), 0);

        ZoneScopedN("RunEulerian");
        Grid2D<float> totalGasGrid{gasMap, wind.occupancy, wind.metadata};
        gasMap.assign(gasMap.size(), 0.0f);

        // this is an optimization: instead of tracking each branching path separately,
        // group all the ones that end up at the same cell and continue a single branch from there
        // markov, baby!
        std::vector<float> currentGasVec(gasMap.size(), 0.0f);
        Grid2D<float> currentGasGrid{currentGasVec, wind.occupancy, wind.metadata};

        std::queue<Vector2Int> activeStates;

        if (source.mode == SimulationSource::AABB)
        {
            AABB2DInt sourceIndices = wind.metadata.coordinatesToIndices(*source.aabb);
            for(auto ind : sourceIndices)
            {
                activeStates.push(ind);
                currentGasGrid.dataAt(ind) = 1.0f;
            }
        }
        else
        {
            Vector2 sourcePoint = source.getPoint();
            Vector2Int sourceIndices = wind.metadata.coordinatesToIndices(sourcePoint);
            activeStates.push(sourceIndices);
            currentGasGrid.dataAt(sourceIndices) = 1.0f;
        }

        size_t numIterations = 0;
        constexpr size_t maxIterations = 1e7;

        // todo this can be persisted between simulations as long as we have the same wind map (and are working in the same room, of course)
        struct TransitionData
        {
            float newProbs[9];
            Vector2Int neighborsIndicesArr[9];
        };
        std::vector<std::optional<TransitionData>> transitionDataCache(currentGasGrid.data.size(), std::nullopt);

        while (!activeStates.empty() && numIterations < maxIterations)
        {
            Vector2Int currentIndices = activeStates.front();
            size_t currentIndex = currentGasGrid.metadata.indexOf(currentIndices.x, currentIndices.y);
            activeStates.pop();
            float currentGasAmount = currentGasGrid.data.at(currentIndex);
            totalGasGrid.data.at(currentIndex) += currentGasAmount;
            currentGasGrid.data.at(currentIndex) = 0;

            if (currentIndex >= totalGasGrid.data.size() || currentGasAmount < lowerThr)
                continue;

            // clang-format off
            const Vector2 segmentAngles [3][3] = {
                {{4.5f/8, 5.5f/8},  {3.5f/8, 4.5f/8},   {2.5f/8, 3.5f/8}},
                {{5.5f/8, 6.5f/8},  {0,0},              {1.5f/8, 2.5f/8}},
                {{6.5f/8, 7.5f/8},  {-0.5f/8, 0.5f/8},  {0.5f/8, 1.5f/8}},
            };
            // clang-format on

            constexpr int BLOCKED = -INT_MAX;
            TransitionData tData;
            if (transitionDataCache.at(currentIndex))
                tData = *transitionDataCache.at(currentIndex);
            else
            {
                constexpr float maxSpeed = 0.2;
                Vector2 windVec = wind.data.at(currentIndex);
                float windSpeed = vmath::length(windVec);
                float windAngle = vmath::angle_fast(windVec);

                float sum = 0;
                for (int i = -1; i <= 1; i++)
                    for (int j = -1; j <= 1; j++)
                    {
                        Vector2Int neighborInd = currentIndices + Vector2Int{i, j};
                        size_t oneDIndex = (i + 1) * 3 + (j + 1);

                        bool inBounds = wind.metadata.indicesInBounds(neighborInd);
                        if (inBounds && wind.occupancyAt(neighborInd) == Occupancy::Obstacle)
                        {
                            tData.neighborsIndicesArr[oneDIndex] = {BLOCKED, BLOCKED};
                            tData.newProbs[oneDIndex] = 0;
                        }
                        else
                        {
                            Vector2 angleLimits = segmentAngles[i + 1][j + 1] * 2 * M_PI;
                            float proportion = Utils::CauchyIntervalProb(angleLimits.x,
                                                                         angleLimits.y,
                                                                         windAngle,
                                                                         std::lerp(0.f, 0.85f, std::clamp(windSpeed / maxSpeed, 0.f, 1.f)));
                            tData.newProbs[oneDIndex] = proportion;
                            sum += proportion;
                            tData.neighborsIndicesArr[oneDIndex] = neighborInd;

                            if (outlets && (!inBounds || wind.occupancyAt(neighborInd) == Occupancy::Unknown))
                            {
                                int outletInd = outlets->mask.data.at(currentIndex);
                                if (outletInd >= 0)
                                    outlets->concentrationExitingDoorway.at(outletInd) += proportion;
                            }
                        }
                    }

                // if neighbor was blocked, the corresponding gas proportion should stay in this cell
                // while this seems a little arbitrary, removing it causes a noticeable artifact on cells adjacent to obstacles, so...
                tData.newProbs[4] = 1.f - sum;

                transitionDataCache[currentIndex] = tData;
            }

            for (size_t k = 0; k < 9; k++)
            {
                if (tData.newProbs[k] < 5e-3 || tData.neighborsIndicesArr[k].x == BLOCKED)
                    continue;

                size_t neighborIndex = currentGasGrid.metadata.indexOf(tData.neighborsIndicesArr[k]);
                if (neighborIndex >= currentGasGrid.data.size())
                    continue;

                float prob = tData.newProbs[k] * currentGasAmount;
                if (currentGasGrid.data.at(neighborIndex) == 0)
                    activeStates.push(tData.neighborsIndicesArr[k]);
                currentGasGrid.data.at(neighborIndex) += prob;
            }

            numIterations++;
        }

        if (numIterations == maxIterations)
            GSL_WARN("Reached {} iterations!", numIterations);

        float rawMax = *std::max_element(gasMap.begin(), gasMap.end());
        Utils::PowerMaxNormalize(gasMap, totalGasGrid.occupancy);

        if (outlets)
            for (size_t i = 0; i < outlets->concentrationExitingDoorway.size(); i++)
            {
                outlets->concentrationExitingDoorway.at(i) /= rawMax * outlets->numCellsOutlet.at(i);
            }
    }
} // namespace GSL
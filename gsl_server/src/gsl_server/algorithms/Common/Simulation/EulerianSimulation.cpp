#include "EulerianSimulation.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
#include "gsl_server/algorithms/Common/Utils/Synchronization.hpp"
#include "gsl_server/core/Profiling.hpp"
#include <stack>

namespace GSL
{
    struct TransitionData
    {
        float newProbs[9];
        Vector2Int neighborsIndicesArr[9];
    };
    static std::map<std::string, std::vector<std::optional<TransitionData>>> transitionDataCaches;

    struct SubSimulation
    {
        bool complete = false;
        std::vector<float> gas;
    };
    static std::map<std::string, std::vector<SubSimulation>> simulationCache;

    void EulerianSimulation::Run(std::vector<float>& gasMap, std::string roomID)
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

        struct State
        {
            Vector2Int indices;
            size_t index;
            bool done;
        };
        std::stack<State> activeStates;

        if (source.mode == SimulationSource::AABB)
        {
            AABB2DInt sourceIndices = wind.metadata.coordinatesToIndices(*source.aabb);
            for (auto ind : sourceIndices)
            {
                activeStates.push({ind, wind.metadata.indexOf(ind), false});
                currentGasGrid.dataAt(ind) = 1.0f;
            }
        }
        else
        {
            Vector2 sourcePoint = source.getPoint();
            Vector2Int sourceIndices = wind.metadata.coordinatesToIndices(sourcePoint);
            activeStates.push({sourceIndices, wind.metadata.indexOf(sourceIndices), false});
            currentGasGrid.dataAt(sourceIndices) = 1.0f;
        }

        size_t numIterations = 0;

        if (!transitionDataCaches.contains(roomID))
            transitionDataCaches[roomID] = std::vector<std::optional<TransitionData>>(currentGasGrid.data.size(), std::nullopt);
        SYNCED_REF(transitionDataCaches.at(roomID), transitionDataCache);

        // SubSimulation caching optimization
        // todo we are probably going to need to store the gas amount for each of these upstream states so we can calculate the value of the cell in the subsim
        // todo what happens when we loop :S
        std::stack<size_t> expandedStates;
        simulationCache.at(roomID).resize(currentGasGrid.data.size(), SubSimulation{.complete = false, .gas = std::vector<float>(currentGasGrid.data.size())});

        while (!activeStates.empty() && numIterations < options.maxIterations)
        {
            State& currentState = activeStates.top();
            GSL_ASSERT(currentGasGrid.occupancy.at(currentState.index) == Occupancy::Free);

            expandedStates.push(currentState.index); // TODO this probably needs to be done later in the loop body

            // we only pop states once we are done with all their children (so, the second time they are at the top of the stack)
            // that way, we know their sub-simulation is complete and can be used
            if (currentState.done)
            {
                activeStates.pop();
                expandedStates.pop();
                simulationCache.at(roomID).at(currentState.index).complete = true;
                continue;
            }
            currentState.done = true;

            float currentGasAmount = currentGasGrid.data.at(currentState.index);
            totalGasGrid.data.at(currentState.index) += currentGasAmount;
            currentGasGrid.data.at(currentState.index) = 0;

            if (currentState.index >= totalGasGrid.data.size() || currentGasAmount < options.lowerThr)
                continue;

            constexpr int BLOCKED = -INT_MAX;
            constexpr int OUT = -INT_MAX + 1;
            TransitionData tData;
            if (SYNC(transitionDataCache).at(currentState.index))
                tData = *SYNC(transitionDataCache).at(currentState.index);
            else
            {
                Vector2 windVec = wind.data.at(currentState.index);
                float windSpeed = vmath::length(windVec);
                float windAngle = vmath::angle_fast(windVec);

                // clang-format off
                const Vector2 segmentAngles [3][3] = {
                    {{4.5f/8, 5.5f/8},  {3.5f/8, 4.5f/8},   {2.5f/8, 3.5f/8}},
                    {{5.5f/8, 6.5f/8},  {0,0},              {1.5f/8, 2.5f/8}},
                    {{6.5f/8, 7.5f/8},  {-0.5f/8, 0.5f/8},  {0.5f/8, 1.5f/8}},
                };
                // clang-format on

                float sum = 0;
                for (int i = -1; i <= 1; i++)
                    for (int j = -1; j <= 1; j++)
                    {
                        Vector2Int neighborInd = currentState.indices + Vector2Int{i, j};
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
                                                                         std::lerp(options.minRho, options.maxRho, std::clamp(windSpeed / options.maxWindSpeed, 0.f, 1.f)));
                            tData.newProbs[oneDIndex] = proportion;
                            sum += proportion;
                            tData.neighborsIndicesArr[oneDIndex] = neighborInd;

                            if (!inBounds || wind.occupancyAt(neighborInd) == Occupancy::Unknown)
                            {
                                tData.neighborsIndicesArr[oneDIndex] = {OUT, OUT};
                                if (outlets)
                                    tData.neighborsIndicesArr[oneDIndex].y = outlets->mask.data.at(currentState.index);
                            }
                        }
                    }

                // if neighbor was blocked, the corresponding gas proportion should stay in this cell
                // while this seems a little arbitrary, removing it causes a noticeable artifact on cells adjacent to obstacles, so...
                tData.newProbs[4] = 1.f - sum;

                SYNC(transitionDataCache)
                [currentState.index] = tData;
            }

            for (size_t k = 0; k < 9; k++)
            {
                float prob = tData.newProbs[k] * currentGasAmount;
                if (tData.neighborsIndicesArr[k].x == OUT)
                {
                    if (tData.neighborsIndicesArr[k].y >= 0)
                        outlets->concentrationExitingDoorway.at(tData.neighborsIndicesArr[k].y) += prob;
                    continue;
                }

                if (tData.newProbs[k] < 5e-3 || tData.neighborsIndicesArr[k].x == BLOCKED)
                    continue;

                size_t neighborIndex = currentGasGrid.metadata.indexOf(tData.neighborsIndicesArr[k]);
                if (neighborIndex >= currentGasGrid.data.size())
                    continue;

                if (currentGasGrid.data.at(neighborIndex) == 0)
                    activeStates.push(State{.indices = tData.neighborsIndicesArr[k], .index = neighborIndex, .done = false});
                currentGasGrid.data.at(neighborIndex) += prob;
            }

            numIterations++;
        }

        if (numIterations == options.maxIterations)
            GSL_WARN("Reached {} iterations!", numIterations);

        float rawMax = *std::max_element(gasMap.begin(), gasMap.end());
        Utils::PowerMaxNormalize(gasMap, totalGasGrid.occupancy);

        if (outlets)
            for (size_t i = 0; i < outlets->concentrationExitingDoorway.size(); i++)
            {
                outlets->concentrationExitingDoorway.at(i) /= rawMax * outlets->numCellsOutlet.at(i);
            }
    }

    void EulerianSimulation::ClearAllCaches()
    {
        transitionDataCaches.clear();
    }

    void EulerianSimulation::ClearCacheRoom(std::string roomID)
    {
        transitionDataCaches.erase(roomID);
    }
} // namespace GSL
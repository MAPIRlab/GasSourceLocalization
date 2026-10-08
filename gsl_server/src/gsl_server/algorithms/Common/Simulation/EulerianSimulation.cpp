#include "EulerianSimulation.hpp"
#include "gsl_server/algorithms/Common/Utils/Images.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
#include "gsl_server/algorithms/Common/Utils/Synchronization.hpp"
#include "gsl_server/core/Profiling.hpp"

namespace GSL
{
    struct TransitionData
    {
        float newProbs[9];
        Vector2Int neighborsIndicesArr[9];
    };
    static std::map<std::string, std::vector<std::optional<TransitionData>>> transitionDataCaches;

    void EulerianSimulation::Run(std::vector<float>& gasMap, std::string roomID, std::optional<std::reference_wrapper<std::vector<float>>> uncertainty)
    {
        outlets->concentrationExitingDoorway.resize(outlets->enabled.size(), 0);
        std::vector<Utils::RunningWeightedMean> age(uncertainty ? uncertainty.value().get().size() : 0);

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
            for (auto ind : sourceIndices)
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

        if (!transitionDataCaches.contains(roomID))
            transitionDataCaches[roomID] = std::vector<std::optional<TransitionData>>(currentGasGrid.data.size(), std::nullopt);
        SYNCED_REF(transitionDataCaches.at(roomID), transitionDataCache);

        while (!activeStates.empty() && numIterations < options.maxIterations)
        {
            Vector2Int currentIndices = activeStates.front();
            size_t currentIndex = currentGasGrid.metadata.indexOf(currentIndices.x, currentIndices.y);
            GSL_ASSERT(currentGasGrid.occupancy.at(currentIndex) == Occupancy::Free);
            activeStates.pop();
            float currentGasAmount = currentGasGrid.data.at(currentIndex);
            totalGasGrid.data.at(currentIndex) += currentGasAmount;
            currentGasGrid.data.at(currentIndex) = 0;

            if (currentIndex >= totalGasGrid.data.size() || currentGasAmount < options.lowerThr)
                continue;

            // clang-format off
            const Vector2 segmentAngles [3][3] = {
                {{4.5f/8, 5.5f/8},  {3.5f/8, 4.5f/8},   {2.5f/8, 3.5f/8}},
                {{5.5f/8, 6.5f/8},  {0,0},              {1.5f/8, 2.5f/8}},
                {{6.5f/8, 7.5f/8},  {-0.5f/8, 0.5f/8},  {0.5f/8, 1.5f/8}},
            };
            // clang-format on

            constexpr int BLOCKED = -INT_MAX;
            constexpr int OUT = -INT_MAX + 1;
            TransitionData tData;
            if (SYNC(transitionDataCache).at(currentIndex))
                tData = *SYNC(transitionDataCache).at(currentIndex);
            else
            {
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
                                                                         std::lerp(0.f, 0.85f, std::clamp(windSpeed / options.maxWindSpeed, 0.f, 1.f)));
                            tData.newProbs[oneDIndex] = proportion;
                            sum += proportion;
                            tData.neighborsIndicesArr[oneDIndex] = neighborInd;

                            if (!inBounds || wind.occupancyAt(neighborInd) == Occupancy::Unknown)
                            {
                                tData.neighborsIndicesArr[oneDIndex] = {OUT, OUT};
                                if (outlets)
                                    tData.neighborsIndicesArr[oneDIndex].y = outlets->mask.data.at(currentIndex);
                            }
                        }
                    }

                // if neighbor was blocked, the corresponding gas proportion should stay in this cell
                // while this seems a little arbitrary, removing it causes a noticeable artifact on cells adjacent to obstacles, so...
                tData.newProbs[4] = 1.f - sum;
                SYNC(transitionDataCache).at(currentIndex) = tData;
            }

            float rejectedByObstacles = tData.newProbs[4];
            float extraAmount = (1.f / (1.f - rejectedByObstacles) - 1.f) * currentGasAmount;
            currentGasAmount += extraAmount;
            totalGasGrid.data.at(currentIndex) += extraAmount;

            for (size_t k = 0; k < 9; k++)
            {
                // skip self
                if (k == 4)
                    continue;

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
                    activeStates.push(tData.neighborsIndicesArr[k]);
                currentGasGrid.data.at(neighborIndex) += prob;
                
                if (uncertainty)
                    age.at(neighborIndex).Update(age.at(currentIndex).Mean() + 1.f, prob);
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

        float max = 0;
        if (uncertainty)
        {
            for (size_t i = 0; i < uncertainty.value().get().size(); i++)
                if (totalGasGrid.occupancy.at(i))
                    max = std::max(max, age.at(i).Mean());
            for (size_t i = 0; i < uncertainty.value().get().size(); i++)
                if (totalGasGrid.occupancy.at(i))
                    uncertainty.value().get().at(i) = age.at(i).Mean() / max;

            std::optional<Utils::Image::BlurMask> blurMask;
            Utils::Image::Blur(uncertainty.value(), options.uncertaintyBlurSigma, totalGasGrid.AsOccupancy(), blurMask);
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
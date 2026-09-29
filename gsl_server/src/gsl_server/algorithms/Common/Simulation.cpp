#include "Simulation.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
#include <angles/angles.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace GSL
{
    // We have a long list of pre-calculated random values for Speeeeeeeeeeeeeeeeed
    static thread_local Utils::PrecalculatedGaussian<2500> gaussian;

    bool Simulation::moveFilament(Filament& filament, Vector2Int& indices, float deltaTime, float noiseSTDev) const
    {
        filament.mostRecentOutlet = outlets->mask.dataAt(indices);
        Vector2 velocity = wind.dataAt(indices.x, indices.y) + Vector2(gaussian.nextValue(0, noiseSTDev), gaussian.nextValue(0, noiseSTDev));

        Vector2 newPos = filament.position + deltaTime * velocity;
        return moveAlongPath(filament.position, indices, newPos);
    }

    template <typename UpdateFunc>
    bool Simulation::filamentIsOutside(Filament& filament, Vector2 oldPos, size_t currentTimestep, UpdateFunc updateFunc)
    {
        Vector2Int newIndices = wind.metadata.coordinatesToIndices(filament.position.x, filament.position.y);

        if (!wind.metadata.indicesInBounds(newIndices) || wind.occupancyAt(newIndices) == Occupancy::Unknown)
        {
            // if we have manually defined outlets
            if (outlets)
            {
                int outletNum = filament.mostRecentOutlet;

                // keep track of how many filaments exit through each outlet
                if (outletNum >= 0 && outlets->enabled.at(outletNum))
                {
                    updateFunc.OnReachOutlet(outlets, outletNum, currentTimestep);
                    return true;
                }
                else
                {
                    // if the outlet is disabled, don't allow this filament to exit
                    // filament.position = oldPos;
                    return false;
                }
            }
            else
                return true;
        }

        return false;
    }

    void Simulation::Run(std::vector<float>& hitMap, Type type)
    {
        // we have this as a function template so the update process can be inlined and we have a single branch per simulation (here)
        // rather than every filament update
        // using functors instead of lambdas to facilitate inlining inside the template

        struct HitFreqFunc
        {
            void Update(std::vector<float>& hitMap, std::vector<uint16_t>& updated, size_t index, size_t t) const
            {
                // mark as updated so it doesn't count multiple filaments in the same timestep
                if (updated.at(index) < t)
                {
                    hitMap.at(index)++;
                    updated.at(index) = t;
                }
            }

            void OnReachOutlet(std::optional<SimulationOutlets>& outlets, size_t outletNum, size_t currentTimestep)
            {
                outlets->totalExitCount++;
                if (outlets->lastUpdateTime.at(outletNum) < currentTimestep)
                {
                    outlets->exitsPerOutlet.at(outletNum)++;
                    outlets->lastUpdateTime.at(outletNum) = currentTimestep;
                }
            }
        };

        struct CummulativeFunc
        {
            void Update(std::vector<float>& hitMap, std::vector<uint16_t>& updated, size_t index, size_t t) const
            {
                hitMap.at(index)++;
            }

            void OnReachOutlet(std::optional<SimulationOutlets>& outlets, size_t outletNum, size_t currentTimestep)
            {
                outlets->totalExitCount++;
                outlets->exitsPerOutlet.at(outletNum)++;
            }
        };

        if (type == Type::HitFrequency)
            _Run(hitMap, HitFreqFunc{}, type);
        else
            _Run(hitMap, CummulativeFunc{}, type);
    }

    void Simulation::RunEulerian(std::vector<float>& gasMap)
    {
        ZoneScopedN("RunEulerian");
        Grid2D<float> totalGasGrid{gasMap, wind.occupancy, wind.metadata};
        gasMap.assign(gasMap.size(), 0.0f);
        Vector2 sourcePoint = source.getPoint();
        Vector2Int sourceIndices = wind.metadata.coordinatesToIndices(sourcePoint);

        // this is an optimization: instead of tracking each branching path separately,
        // group all the ones that end up at the same cell and continue a single branch from there
        // markov, baby!
        std::vector<float> currentGasVec(gasMap.size(), 0.0f);
        Grid2D<float> currentGasGrid{currentGasVec, wind.occupancy, wind.metadata};

        // this can be persisted between simulations as long as we have the same wind map (and are working in the same room, of course)
        struct TransitionData
        {
            float newProbs[9] = {};
            Vector2Int neighborsIndicesArr[9] = {};
        };
        std::unordered_map<Vector2Int, TransitionData> transitionDataCache;

        std::queue<Vector2Int> activeStates;
        activeStates.push(sourceIndices);
        currentGasGrid.dataAt(sourceIndices) = 1.0f;

        size_t numIterations = 0;
        constexpr size_t maxIterations = 50000;
        constexpr float lowerThr = 1e-4;

        while (!activeStates.empty()) //&& numIterations < maxIterations)
        {
            Vector2Int currentIndices = activeStates.front();
            size_t currentIndex = currentGasGrid.metadata.indexOf(currentIndices.x, currentIndices.y);
            activeStates.pop();
            float currentGasAmount = currentGasGrid.data.at(currentIndex);
            totalGasGrid.data.at(currentIndex) += currentGasAmount;
            currentGasGrid.data.at(currentIndex) = 0;

            if (!totalGasGrid.metadata.indicesInBounds(currentIndices) || currentGasAmount < lowerThr)
                continue;

            constexpr float maxSpeed = 0.15;
            Vector2 windVec = wind.data.at(currentIndex);
            float windSpeed = vmath::length(windVec);
            float windAngle = vmath::angle_fast(windVec);

            // clang-format off
            const Vector2 segmentAngles [3][3] = {
                {{4.5f/8, 5.5f/8},  {3.5f/8, 4.5f/8},   {2.5f/8, 3.5f/8}},
                {{5.5f/8, 6.5f/8},  {0,0},              {1.5f/8, 2.5f/8}},
                {{6.5f/8, 7.5f/8},  {-0.5f/8, 0.5f/8},  {0.5f/8, 1.5f/8}},
            };
            // clang-format on

            constexpr int BLOCKED = -INT_MAX;
            TransitionData tData;
            if (transitionDataCache.contains(currentIndices))
                tData = transitionDataCache.at(currentIndices);
            else
            {
                for (int i = -1; i <= 1; i++)
                    for (int j = -1; j <= 1; j++)
                    {
                        Vector2Int neighborInd = currentIndices + Vector2Int{i, j};
                        size_t oneDIndex = (i + 1) * 3 + (j + 1);

                        if (wind.metadata.indicesInBounds(neighborInd) && !wind.occupancyAt(neighborInd))
                            tData.neighborsIndicesArr[oneDIndex] = {BLOCKED, BLOCKED};
                        else
                        {
                            Vector2 angleLimits = segmentAngles[i + 1][j + 1] * 2 * M_PI;
                            float proportion = Utils::CauchyIntervalProb(angleLimits.x,
                                                                         angleLimits.y,
                                                                         windAngle,
                                                                         std::lerp(0.f, 0.9f, std::clamp(windSpeed / maxSpeed, 0.f, 1.f)));
                            tData.newProbs[oneDIndex] = proportion;
                            tData.neighborsIndicesArr[oneDIndex] = neighborInd;
                        }
                    }

                // tData.newProbs[4] = std::lerp(0.2f, 0.f, std::clamp(windSpeed / maxSpeed, 0.f, 1.f));
                transitionDataCache[currentIndices] = tData;
            }

            // if some of the neighbors were blocked, normalize the probs to make sure it all still adds up to 1
            float sum = std::accumulate(tData.newProbs, tData.newProbs + 9, 0.0f);
            for (size_t k = 0; k < 9; k++)
            {
                if (tData.newProbs[k] < 5e-3 || tData.neighborsIndicesArr[k].x == BLOCKED)
                    continue;

                size_t neighborIndex = currentGasGrid.metadata.indexOf(tData.neighborsIndicesArr[k]);
                if (neighborIndex >= currentGasGrid.data.size())
                    continue;

                float prob = (tData.newProbs[k] / sum) * currentGasAmount;
                if (currentGasGrid.data.at(neighborIndex) == 0)
                    activeStates.push(tData.neighborsIndicesArr[k]);
                currentGasGrid.data.at(neighborIndex) += prob;
            }

            numIterations++;
        }

        GSL_INFO("{} iterations", numIterations);
    }

    template <typename UpdateFunc>
    void Simulation::_Run(std::vector<float>& hitMap, UpdateFunc updateFunc, Type type)
    {
        size_t max_filaments = maxWarmupIterations * source.numFilamentsSecond * deltaTime * warmupAcceleration // max filaments in warmup
                               + timesteps * source.numFilamentsSecond * deltaTime;                             // max filaments when recording

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

        outlets->lastUpdateTime.resize(outlets->enabled.size(), 0);

        std::vector<uint16_t> updated(hitMap.size(), 0); // index of the last iteration in which this cell was updated, to avoid double-counting

        // reset the count of how many filaments took each outlet
        if (outlets)
            std::fill(outlets->exitsPerOutlet.begin(), outlets->exitsPerOutlet.end(), 0);

        // warm-up: we don't want to start recording frequency of hits until the shape of the plume has stabilized. Wait until a filament exits the
        // environment through an outlet, or a maximum number of steps
        {
            ZoneScopedN("Warmup");

            bool stable = false;
            size_t iterationCount = 0;
            while (iterationCount < minWarmupIterations || (!stable && iterationCount < maxWarmupIterations))
            {
                size_t emitCount = source.FilamentsToEmit(deltaTime * warmupAcceleration);
                for (int i = 0; i < emitCount; i++)
                {
                    activeFilamentVec->emplace_back();
                    activeFilamentVec->back().position = source.getPoint();
                }

                for (Filament& filament : *activeFilamentVec)
                {
                    auto indices = wind.metadata.coordinatesToIndices(filament.position.x, filament.position.y);

                    // this can happen as a result of sources with imprecisely defined shapes. Don't worry about performance, we would have had to check later anyways
                    if (!wind.metadata.indicesInBounds(indices) || !wind.occupancyAt(indices.x, indices.y))
                        continue;

                    // move active filaments
                    Vector2 oldPos = filament.position;
                    moveFilament(filament, indices, deltaTime * warmupAcceleration, noiseSTDev / warmupAcceleration);

                    filament.age++;
                    // remove filaments
                    if (filamentIsOutside(filament, oldPos, 0, updateFunc))
                        stable = true;
                    else
                        otherFilamentVec->push_back(filament);
                }
                iterationCount++;

                // "other" now contains the list of all the filaments that are still available, so swap the vectors and remove the old list
                activeFilamentVec->clear();
                std::swap(activeFilamentVec, otherFilamentVec);
            }
            // GSL_INFO("Warmup complete ({} iterations)", iterationCount);
        }

        ZoneScopedN("Recording");
        // now, we do the thing
        for (size_t t = 1; t < timesteps + 1; t++)
        {
            size_t emitCount = source.FilamentsToEmit(deltaTime);
            for (size_t i = 0; i < emitCount; i++)
            {
                activeFilamentVec->emplace_back();
                activeFilamentVec->back().position = source.getPoint();
                totalEmittedFilaments++;
            }

            for (Filament& filament : *activeFilamentVec)
            {
                // update map
                auto indices = wind.metadata.coordinatesToIndices(filament.position.x, filament.position.y);
                size_t index = wind.metadata.indexOf(indices);

                // this can happen as a result of sources with imprecisely defined shapes. Don't worry about performance, we would have had to check later anyways
                if (!wind.metadata.indicesInBounds(indices) || !wind.occupancyAt(indices.x, indices.y))
                    continue;

                GSL_ASSERT(wind.metadata.indicesInBounds(indices));
                updateFunc.Update(hitMap, updated, index, t);

                // move active filaments
                Vector2 oldPos = filament.position;
                moveFilament(filament, indices, deltaTime, noiseSTDev);
                filament.age++;

                // remove filaments
                if (!filamentIsOutside(filament, oldPos, t, updateFunc))
                    otherFilamentVec->push_back(filament);
            }
            activeFilamentVec->clear();
            std::swap(activeFilamentVec, otherFilamentVec);
        }

        if (type == Type::HitFrequency)
        {
            float normalizationVal = timesteps;
            // convert the total hit count into relative frequency
            for (int i = 0; i < wind.occupancy.size(); i++)
                hitMap.at(i) = hitMap.at(i) / normalizationVal;
        }
    }

    Vector2 SimulationSource::getPoint() const
    {
        if (mode == Mode::Point)
            return point;

        const Vector2& start = aabb->min;
        const Vector2& end = aabb->max;

        Vector2 randP(Utils::uniformRandomF(start.x, end.x), Utils::uniformRandomF(start.y, end.y));
        GSL_ASSERT(randP.x >= start.x && randP.x < end.x && randP.y >= start.y && randP.y < end.y);
        return randP;
    }

    size_t SimulationSource::FilamentsToEmit(float deltaT)
    {
        emissionCounter += numFilamentsSecond * deltaT;
        size_t num = emissionCounter;
        emissionCounter -= num;
        return num;
    }

    bool Simulation::moveAlongPath(Vector2& currentPosition, const Vector2Int& indexOrigin, const Vector2& end) const
    {
        Vector2Int indexEnd = wind.metadata.coordinatesToIndices(end.x, end.y);

        // try to avoid doing the raycast by looking at the pre-computed visibilityMap
        if (indexOrigin == indexEnd || //
            (wind.metadata.indicesInBounds(indexEnd) && wind.occupancyAt(indexEnd.x, indexEnd.y) &&
             visibilityMap.has_value() &&
             visibilityMap->get().isVisible(indexOrigin, indexEnd) == Visibility::Visible))
        {
            currentPosition = end;
            return true;
        }

#define USE_DDA 0
#if USE_DDA
        Vector2 movement = end - currentPosition;
        DDA::_2D::RayCastInfo raycastInfo =
            DDA::_2D::castRay<GSL::Occupancy>(currentPosition, movement, vmath::length(movement),
                                              DDA::_2D::Map<GSL::Occupancy>(wind.occupancy, wind.metadata.origin,
                                                                            wind.metadata.cellSize, wind.metadata.dimensions),
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
        const auto& metadata = wind.metadata;
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
            bool isOutside = !metadata.indicesInBounds(pair);
            bool freeBecauseOutside = isOutside || wind.occupancyAt(pair.x, pair.y) == GSL::Occupancy::Unknown;
            pathIsFree = freeBecauseOutside || (!isOutside && wind.occupancyAt(pair.x, pair.y));
            if (!pathIsFree)
                currentPosition -= increment;
        }
        return pathIsFree;
#endif
    }

    static void show(const cv::Mat& mat, std::string name)
    {
        cv::Mat resized;
        cv::resize(mat, resized, cv::Size(mat.size[1] * 10, mat.size[0] * 10), 0, 0, cv::INTER_NEAREST);
        cv::imshow(name, resized);
        cv::setWindowProperty(name, cv::WND_PROP_TOPMOST, 1); // force focus
        while (cv::getWindowProperty(name, cv::WindowPropertyFlags::WND_PROP_VISIBLE) && cv::waitKey(30) == -1)
            ;
        cv::destroyWindow(name);
    }

    void Simulation::makeSimulationImage()
    {
        std::vector<float> hitMap(wind.data.size(), 0.0);
        Run(hitMap);

        displayImage(Grid2D<float>(hitMap, wind.occupancy, wind.metadata));
    }

    void Simulation::displayImage(const Grid2D<float>& hitMap, const std::string& imageName, float raisePower)
    {
        std::vector<float> hitMapCopy = hitMap.data;
        for (float& f : hitMapCopy)
            f = std::pow(f, raisePower);
        cv::Mat asImage(hitMapCopy);
        asImage = asImage.reshape(1, hitMap.metadata.dimensions.y);

        cv::Mat inColor;
        asImage.convertTo(asImage, CV_8UC1, 255);
        cv::applyColorMap(asImage, inColor, cv::COLORMAP_VIRIDIS);

        for (int j = 0; j < hitMap.metadata.dimensions.y; j++)
        {
            for (int i = 0; i < hitMap.metadata.dimensions.x; i++)
            {
                if (!hitMap.occupancyAt(i, j))
                    inColor.at<cv::Vec3b>(j, i) = cv::Vec3b(80, 80, 80);
            }
        }

#if 0
        cv::flip(inColor, inColor, 0);
        inColor *= 255;
        cv::imwrite(fmt::format("{}.png", imageName), inColor);
        GSL_WARN("hitMap image saved");
#else
        cv::flip(inColor, inColor, 0);
        show(inColor, imageName);
#endif
    }

    void Simulation::blurHitMap(std::vector<float>& hitMap, float blurSigma, Grid2D<Occupancy> occupancy, std::optional<SimulationBlurMask>& blurredMask)
    {
        if (blurSigma == 0)
            return;
        cv::Mat asImage(hitMap, false); // copyData=false, so changes to the matrix will affect the hitMap vector
        asImage = asImage.reshape(1, occupancy.metadata.dimensions.y);

        cv::GaussianBlur(asImage, asImage, cv::Size(0, 0), blurSigma, blurSigma);

        // divide by the blurred mask to correct the edges always getting lower
        if (!blurredMask || blurredMask->sigma != blurSigma)
        {
            blurredMask.emplace();
            blurredMask->sigma = blurSigma;
            cv::Mat freeSpaceMask(
                cv::Size(occupancy.metadata.dimensions.x, occupancy.metadata.dimensions.y),
                CV_32F,
                cv::Scalar(0, 0, 0));

            for (int j = 0; j < occupancy.metadata.dimensions.y; j++)
            {
                for (int i = 0; i < occupancy.metadata.dimensions.x; i++)
                {
                    if (occupancy.occupancyAt(i, j))
                        freeSpaceMask.at<float>(j, i) = 1;
                }
            }

            cv::GaussianBlur(freeSpaceMask, blurredMask->mask, cv::Size(0, 0), blurSigma, blurSigma);
        }

        for (int i = 0; i < occupancy.metadata.dimensions.y; i++)
            for (int j = 0; j < occupancy.metadata.dimensions.x; j++)
            {
                if (!occupancy.occupancyAt(j, i))
                    asImage.at<float>(i, j) = 0;
                else if (blurredMask->mask.at<float>(i, j) > 0)
                    asImage.at<float>(i, j) = Utils::clamp(asImage.at<float>(i, j) / blurredMask->mask.at<float>(i, j), 0, 1);
            }
    }
} // namespace GSL

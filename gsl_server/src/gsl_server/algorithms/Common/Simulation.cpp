#include "Simulation.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
#include "gsl_server/core/Profiling.hpp"

namespace GSL
{
    // We have a long list of pre-calculated random values for Speeeeeeeeeeeeeeeeed
    static thread_local Utils::PrecalculatedGaussian<2500> gaussian;

    void Simulation::moveFilament(Filament& filament, Vector2Int& indices, float deltaTime, float noiseSTDev) const
    {
        Vector2 velocity = wind.dataAt(indices.x, indices.y) + Vector2(gaussian.nextValue(0, noiseSTDev), gaussian.nextValue(0, noiseSTDev));

        Vector2 newPos = filament.position + deltaTime * velocity;
        moveAlongPath(filament.position, newPos);
    }

    bool Simulation::filamentIsOutside(const Filament& filament) const
    {
        Vector2Int newIndices = wind.metadata.coordinatesToIndices(filament.position.x, filament.position.y);
        return !wind.metadata.indicesInBounds(newIndices);
    }

    void Simulation::Run(std::vector<float>& hitMap)
    {
        constexpr int numFilamentsIteration = 5;
        size_t max_filaments = maxWarmupIterations * numFilamentsIteration + timesteps * numFilamentsIteration;

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
            while (iterationCount < minWarmupIterations || (!stable && iterationCount < maxWarmupIterations))
            {
                for (size_t i = 0; i < numFilamentsIteration; i++)
                {
                    activeFilamentVec->emplace_back();
                    activeFilamentVec->back().position = source.getPoint();
                }

                for (Filament& filament : *activeFilamentVec)
                {
                    auto indices = wind.metadata.coordinatesToIndices(filament.position.x, filament.position.y);

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
                auto indices = wind.metadata.coordinatesToIndices(filament.position.x, filament.position.y);
                size_t index = wind.metadata.indexOf(indices);
                GSL_ASSERT(wind.metadata.indicesInBounds(indices));
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
        for (int i = 0; i < wind.occupancy.size(); i++)
        {
            if (wind.occupancy[i] == Occupancy::Free)
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

    bool Simulation::moveAlongPath(Vector2& currentPosition, const Vector2& end) const
    {
        Vector2Int indexEnd = wind.metadata.coordinatesToIndices(end.x, end.y);
        Vector2Int indexOrigin = wind.metadata.coordinatesToIndices(currentPosition.x, currentPosition.y);

        if (!wind.freeAt(indexOrigin.x, indexOrigin.y))
        {
            return false;
        }

        // try to avoid doing the raycast by looking at the pre-computed visibilityMap
        if (indexOrigin == indexEnd || //
            (wind.metadata.indicesInBounds(indexEnd) && wind.freeAt(indexEnd.x, indexEnd.y) &&
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
            pathIsFree = !metadata.indicesInBounds(pair) || wind.freeAt(pair.x, pair.y);
            if (!pathIsFree)
                currentPosition -= increment;
        }
        return pathIsFree;
#endif
    }
} // namespace GSL

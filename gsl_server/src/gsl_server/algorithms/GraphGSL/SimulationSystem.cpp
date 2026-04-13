#include "SimulationSystem.hpp"
#include "Node.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
#include "gsl_server/algorithms/Common/Utils/Pointers.hpp"

namespace GSL::Graph_internal
{
    SimulationSystem::SimWithResult SimulationSystem::SimulateFromPoint(const std::shared_ptr<RealNode> realNode, Vector2 point)
    {
        ScopedStopwatch s("sims");
        SimWithResult result;
        result.hitMap = std::make_shared<std::vector<float>>(realNode->GetOccupancy().data.size(), 0.);
        result.simulation = std::shared_ptr<Simulation>(new Simulation{
            .source = SimulationSource(point),
            .noiseSTDev = options.noiseSTDev,
            .minWarmupIterations = options.minWarmupIterations,
            .maxWarmupIterations = options.maxWarmupIterations,
            .wind = realNode->GetWindMap(),
            .outlets = SimulationOutlets{
                .mask = realNode->GetOutletsMask(),
                .exitsCount = std::vector<size_t>(realNode->arcs.size(), 0),
            },
        });

        result.simulation->outlets->exitsCount.resize(realNode->arcs.size(), 0);
        result.simulation->outlets->enabled.resize(realNode->arcs.size(), true);

        Simulation::Type type = options.cummulativeMap ? Simulation::Type::Cummulative : Simulation::Type::HitFrequency;
        result.simulation->Run(*result.hitMap, type);

        Utils::Windsorize(*result.hitMap, 5);
        Utils::PowerMaxNormalize(*result.hitMap, realNode->GetOccupancy().occupancy, options.normalizationPower);
        Simulation::blurHitMap(*result.hitMap, options.blurSigma, realNode->GetOccupancy(), blurMasks[realNode]);
        Utils::PowerMaxNormalize(*result.hitMap, realNode->GetOccupancy().occupancy, 1);
        return result;
    }

    SimulationSystem::SimWithResult SimulationSystem::SimulateFromArc(const Arc& arc)
    {
        SimWithResult result;
        auto realNode = As<RealNode>(arc.from.lock());
        Grid2DMetadata nodeMetadata = realNode->GetOccupancy().metadata;
        AABB2D sourceAABB(arc.aabb.min,
                          arc.aabb.max);
        Vector2 maxCoords = nodeMetadata.indicesToCoordinates(nodeMetadata.dimensions, false) - Vector2{0.001, 0.001};
        sourceAABB.min.x = std::clamp(sourceAABB.min.x, nodeMetadata.origin.x, maxCoords.x);
        sourceAABB.min.y = std::clamp(sourceAABB.min.y, nodeMetadata.origin.y, maxCoords.y);
        sourceAABB.max.x = std::clamp(sourceAABB.max.x, nodeMetadata.origin.x, maxCoords.x);
        sourceAABB.max.y = std::clamp(sourceAABB.max.y, nodeMetadata.origin.y, maxCoords.y);

        // configure the simulation
        result.hitMap = std::make_shared<std::vector<float>>(realNode->GetOccupancy().data.size(), 0.);
        result.simulation = std::shared_ptr<Simulation>(new Simulation{
            .source = SimulationSource(sourceAABB),
            .timesteps = options.iterationLimit,
            .noiseSTDev = options.noiseSTDev,
            .minWarmupIterations = options.minWarmupIterations,
            .maxWarmupIterations = options.maxWarmupIterations,
            .wind = realNode->GetWindMap(),
            .outlets = SimulationOutlets{
                .mask = realNode->GetOutletsMask(),
                .exitsCount = std::vector<size_t>(realNode->arcs.size(), 0),
                .numCellsOutlet = realNode->GetOutletsCellCount(),
            },
        });

        result.simulation->outlets->exitsCount.resize(realNode->arcs.size(), 0);
        result.simulation->outlets->enabled.resize(realNode->arcs.size(), true);

        for (size_t i = 0; i < realNode->arcs.size(); i++)
            if (&realNode->arcs.at(i) == &arc)
                result.simulation->outlets->enabled.at(i) = false;

        Simulation::Type type = options.cummulativeMap ? Simulation::Type::Cummulative : Simulation::Type::HitFrequency;
        result.simulation->Run(*result.hitMap, type);

        if (options.cummulativeMap)
        {
            Utils::Windsorize(*result.hitMap, 5);
            Utils::PowerMaxNormalize(*result.hitMap, realNode->GetOccupancy().occupancy, options.normalizationPower);
            Simulation::blurHitMap(*result.hitMap, options.blurSigma, realNode->GetOccupancy(), blurMasks[realNode]);
            Utils::PowerMaxNormalize(*result.hitMap, realNode->GetOccupancy().occupancy, 1);
        }
        else
            Simulation::blurHitMap(*result.hitMap, options.blurSigma, realNode->GetOccupancy(), blurMasks[realNode]);

        // store the simulation result in the cache
        //-------------------
        simulationCache[arc.getUID()] = result;

        return result;
    }

} // namespace GSL::Graph_internal
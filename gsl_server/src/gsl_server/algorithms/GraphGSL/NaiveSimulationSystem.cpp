#include "NaiveSimulationSystem.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
#include "gsl_server/algorithms/Common/Utils/Pointers.hpp"

namespace GSL::Graph_internal
{
    NaiveSimulationSystem::NaiveSimulationSystem(Options& options)
        : options(options) {}
        
    SimWithResult NaiveSimulationSystem::SimulateSourceFromPoint(const std::shared_ptr<PlaceNode> entireMap, const Vector2& sourcePoint)
    {
        auto roomNode = As<RoomNode>(entireMap);

        SimWithResult result;
        result.hitMap = std::make_shared<std::vector<float>>(roomNode->GetOccupancy().data.size(), 0.);
        result.simulation = std::shared_ptr<Simulation>(new Simulation{
            .source = SimulationSource(sourcePoint),
            .warmupAcceleration = options.warmupTimeAcc,
            .timesteps = options.iterationLimit,
            .deltaTime = options.deltaTime,
            .noiseSTDev = options.noiseSTDev,
            .minWarmupIterations = options.minWarmupIterations,
            .maxWarmupIterations = options.maxWarmupIterations,
            .wind = roomNode->GetWindMap(),
            .outlets = SimulationOutlets{
                .mask = roomNode->GetOutletsMask(),
                .exitsPerOutlet = std::vector<size_t>(roomNode->doorways.size(), 0),
                .numCellsOutlet = roomNode->GetOutletsCellCount(),
            },
        });
        result.simulation->source.numFilamentsSecond = options.filamentsPerSecond;

        result.simulation->outlets->exitsPerOutlet.resize(roomNode->doorways.size(), 0);
        result.simulation->outlets->enabled.resize(roomNode->doorways.size(), true);

        Simulation::Type type = options.cummulativeMap ? Simulation::Type::Cummulative : Simulation::Type::HitFrequency;
        result.simulation->Run(*result.hitMap, type);

        Utils::Winsorize(*result.hitMap, 5);
        Utils::PowerMaxNormalize(*result.hitMap, roomNode->GetOccupancy().occupancy, options.normalizationPower);
        Simulation::blurHitMap(*result.hitMap, options.blurSigma, roomNode->GetOccupancy(), blurMasks[roomNode]);
        Utils::PowerMaxNormalize(*result.hitMap, roomNode->GetOccupancy().occupancy, 1);
        return result;
    }

    CompleteMap NaiveSimulationSystem::AsCompleteMap(const std::shared_ptr<PlaceNode> entireMap, SimWithResult result)
    {
        CompleteMap completeMap{
            .source = std::make_shared<Graph_internal::PointSource>(result.simulation->source.getPoint()),
            .gasMaps = {
                {As<RoomNode>(entireMap), *result.hitMap}}};
        return completeMap;
    }
} // namespace GSL::Graph_internal
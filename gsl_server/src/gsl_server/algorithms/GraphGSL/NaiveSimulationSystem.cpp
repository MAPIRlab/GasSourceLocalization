#include "NaiveSimulationSystem.hpp"
#include "gsl_server/algorithms/Common/Simulation/FilamentSimulation.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
#include "gsl_server/algorithms/Common/Utils/Pointers.hpp"

namespace GSL::Graph_internal
{
    NaiveSimulationSystem::NaiveSimulationSystem(FilamentSimOptions& options)
        : options(options) {}
        
    SimWithResult NaiveSimulationSystem::SimulateSourceFromPoint(const std::shared_ptr<PlaceNode> entireMap, const Vector2& sourcePoint)
    {
        auto roomNode = As<RoomNode>(entireMap);

        SimWithResult result;
        result.hitMap = std::make_shared<std::vector<float>>(roomNode->GetOccupancy().data.size(), 0.);
        std::shared_ptr<FilamentSimulation> filamentSim(new FilamentSimulation(SimulationSource(sourcePoint),
                                                                               roomNode->GetWindMap(),
                                                                               SimulationOutlets{
                                                                                   .mask = roomNode->GetOutletsMask(),
                                                                                   .numCellsOutlet = roomNode->GetOutletsCellCount(),
                                                                               }));
        result.simulation = filamentSim;

        filamentSim->warmupAcceleration = options.warmupTimeAcc;
        filamentSim->timesteps = options.iterationLimit;
        filamentSim->deltaTime = options.deltaTime;
        filamentSim->noiseSTDev = options.noiseSTDev;
        filamentSim->minWarmupIterations = options.minWarmupIterations;
        filamentSim->maxWarmupIterations = options.maxWarmupIterations;
        filamentSim->numFilamentsSecond = options.filamentsPerSecond;
        // result.simulation->visibilityMap.emplace(roomNode->GetVisibilityMap());

        filamentSim->filamentOutlets->exitsPerOutlet.resize(roomNode->doorways.size(), 0);
        result.simulation->outlets->enabled.resize(roomNode->doorways.size(), true);

        FilamentSimulation::Type type = options.cummulativeMap ? FilamentSimulation::Type::Cummulative : FilamentSimulation::Type::HitFrequency;
        filamentSim->Run(*result.hitMap, type);


        Utils::Winsorize(*result.hitMap, 5);
        Utils::PowerMaxNormalize(*result.hitMap, roomNode->GetOccupancy().occupancy, options.normalizationPower);
        Utils::Image::Blur(*result.hitMap, options.blurSigma, roomNode->GetOccupancy(), blurMasks[roomNode]);
        Utils::PowerMaxNormalize(*result.hitMap, roomNode->GetOccupancy().occupancy, 1.f);
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
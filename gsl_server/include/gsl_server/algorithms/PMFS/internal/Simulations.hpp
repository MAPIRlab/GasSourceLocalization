#pragma once
#include <gsl_server/algorithms/PMFS/internal/Settings.hpp>
#include <gsl_server/Utils/NQAQuadtree.hpp>
#include <gsl_server/algorithms/PMFS/internal/HitProbability.hpp>
#include <gsl_server/algorithms/PMFS/internal/VisibilityMap.hpp>

namespace GSL
{
    class PMFS;
}

namespace GSL::PMFS_internal
{

    struct Filament
    {
        Vector2 position;
        bool active = false;
    };

    struct SimulationSource
    {
        enum Mode
        {
            Quadtree,
            Point
        };

        const Mode mode;
        const Utils::NQA::Node* nqaNode;
        const Vector2 point;
        const GridMetadata& metadata;

        SimulationSource(const Vector2& _point, const GridMetadata& _metadata) : nqaNode(nullptr), point(_point), mode(Mode::Point), metadata(_metadata)
        {}
        SimulationSource(const Utils::NQA::Node* _node, const GridMetadata& _metadata) : point(0, 0), nqaNode(_node), mode(Mode::Quadtree), metadata(_metadata)
        {}

        Vector2 getPoint() const;
    };

    class Simulations
    {
        using HashSet = std::unordered_set<Vector2Int>;
    public:
        Simulations(Grid<HitProbability> _measuredHitProb, Grid<double> _sourceProb, Grid<Vector2> _wind, const PMFS_internal::SimulationSettings& _settings)
            : measuredHitProb(_measuredHitProb), sourceProb(_sourceProb), wind(_wind), settings(_settings)
        {}

        void initializeMap(const std::vector<std::vector<uint8_t>>& occupancyMap);
        void updateSourceProbability(float refineFraction);
        void printImage(const SimulationSource& source);

        std::vector<std::vector<Utils::NQA::Node*>> mapSegmentation;
        std::unique_ptr<Utils::NQA::Quadtree> quadtree;
        std::vector<Utils::NQA::Node> QTleaves;
        std::vector<double> varianceOfHitProb; // calculated from the simulations, used for movement
        VisibilityMap* visibilityMap;
    protected:
        const PMFS_internal::SimulationSettings& settings;
        Grid<HitProbability> measuredHitProb;
        Grid<double> sourceProb;
        Grid<Vector2> wind;
        void moveFilament(Filament& filament, Vector2Int& indices, float deltaTime, float noiseSTDev);
        void moveFilamentDiscretePosition(Filament& filament, Vector2Int& indices, float noiseSTDev);
        void simulateSourceInPosition(const SimulationSource& source, std::vector<float>& hitMap, bool warmup, int warmupLimit,
                                      int timesteps, float deltaTime, float noiseSTDev);
        bool filamentIsOutside(Filament& filament);
        double weightedDifference(const Grid<HitProbability>& hitRandomVariable, const std::vector<float>& hitMap);
        bool moveAlongPath(Vector2& beginning, const Vector2& end);
    };
} // namespace GSL::PMFS_internal
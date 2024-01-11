#pragma once
#include <gsl_server/Utils/NQAQuadtree.hpp>
#include <gsl_server/algorithms/PMFS/internal/Cell.hpp>

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

        const PMFS* pmfs;

        SimulationSource(const Vector2& _point, const PMFS* _pmfs) : nqaNode(nullptr), point(_point), mode(Mode::Point), pmfs(_pmfs)
        {}
        SimulationSource(const Utils::NQA::Node* _node, const PMFS* _pmfs) : point(0, 0), nqaNode(_node), mode(Mode::Quadtree), pmfs(_pmfs)
        {}

        Vector2 getPoint() const;
    };

    class Simulations
    {
    public:
        Simulations(PMFS* _pmfs) : pmfs(_pmfs){}
        void updateSourceProbability(float refineFraction);
        void printImage(const SimulationSource& source);

        std::vector<std::vector<Utils::NQA::Node*>> mapSegmentation;
        std::unique_ptr<Utils::NQA::Quadtree> quadtree;
        std::vector<Utils::NQA::Node> QTleaves;
        std::vector<std::vector<double>> varianceOfHitProb; // calculated from the simulations, used for movement
    protected:
        PMFS* pmfs;
        void moveFilament(Filament& filament, Vector2Int& indices, float deltaTime, float noiseSTDev);
        void moveFilamentDiscretePosition(Filament& filament, Vector2Int& indices, float noiseSTDev);
        void simulateSourceInPosition(const SimulationSource& source, std::vector<std::vector<float>>& hitMap, bool warmup, int warmupLimit,
                                      int timesteps, float deltaTime, float noiseSTDev);
        bool filamentIsOutside(Filament& filament);
        double weightedDifference(const std::vector<std::vector<Cell>>& hitRandomVariable, const std::vector<std::vector<float>>& hitMap);
        bool moveAlongPath(Vector2& beginning, const Vector2& end);
    };
} // namespace GSL::PMFS_internal
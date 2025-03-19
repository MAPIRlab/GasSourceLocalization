#pragma once
#include <gsl_server/algorithms/Common/Utils/NQAQuadtree.hpp>
#include <gsl_server/algorithms/PMFS/internal/HitProbability.hpp>
#include <gsl_server/algorithms/PMFS/internal/Settings.hpp>
#include <gsl_server/algorithms/PMFS/internal/VisibilityMap.hpp>
#include <opencv2/core.hpp>

namespace GSL
{
    class PMFS;
}

namespace GSL::PMFS_internal
{

    struct Filament
    {
        Vector2 position;
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
        const Grid2DMetadata& metadata;

        SimulationSource(const Vector2& _point, const Grid2DMetadata& _metadata)
            : mode(Mode::Point), nqaNode(nullptr), point(_point), metadata(_metadata)
        {}
        SimulationSource(const Utils::NQA::Node* _node, const Grid2DMetadata& _metadata)
            : mode(Mode::Quadtree), nqaNode(_node), point(0, 0), metadata(_metadata)
        {}

        Vector2 getPoint() const;
    };

    class Simulations
    {
        using HashSet = std::unordered_set<Vector2Int>;

    public:
        struct SimulationResult
        {
            bool valid = false;
            std::vector<float> hitMap;
            long double sourceProb;
        };

    public:
        Simulations(Grid2D<HitProbability> _measuredHitProb, Grid2D<double> _sourceProb, Grid2D<Vector2> _wind,
                    const PMFS_internal::SimulationSettings& _settings)
            : settings(_settings), measuredHitProb(_measuredHitProb), sourceProb(_sourceProb), wind(_wind)
        {}

        void initializeMap(const std::vector<std::vector<uint8_t>>& occupancyMap);
        void updateSourceProbability(float refineFraction);
        void makeSimulationImage(const SimulationSource& source);
        double probabilitySingleFrequency(double measured, double simulated) const;
        double probabilityFromSingleCell(HitProbability measured, double simulated) const;
        long double sourceProbFromMaps(const Grid2D<HitProbability>& hitRandomVariable, const std::vector<float>& hitMap) const;

        std::vector<std::vector<Utils::NQA::Node*>> mapSegmentation;
        std::unique_ptr<Utils::NQA::Quadtree> quadtree;
        std::vector<Utils::NQA::Node> QTleaves;
        std::vector<double> varianceOfHitProb; // calculated from the simulations, used for movement
        std::vector<SimulationResult> resultsFirstLevel;
        VisibilityMap* visibilityMap;

    protected:

        struct LeafScore
        {
            long double score;
            Utils::NQA::Node* leaf;
        };

        std::vector<long double> sourceProbInternal; // calculated from the simulations, used for movement
        const PMFS_internal::SimulationSettings& settings;
        Grid2D<HitProbability> measuredHitProb;
        Grid2D<double> sourceProb;
        Grid2D<Vector2> wind;
        cv::Mat freeSpaceMask;

        SimulationResult runSimulation(std::vector<LeafScore>& nodes, size_t index);
        void moveFilament(Filament& filament, Vector2Int& indices, float deltaTime, float noiseSTDev) const;
        void simulateSourceInPosition(const SimulationSource& source, std::vector<float>& hitMap, bool warmup,
                                      int timesteps, float deltaTime, float noiseSTDev) const;
        bool filamentIsOutside(const Filament& filament) const;
        bool moveAlongPath(Vector2& beginning, const Vector2& end) const;

        void blurHitMap(cv::Mat& asImage) const;
        void displayImage(const std::vector<float>& hitMap, const std::string& imageName = "simResult") const;
    };
} // namespace GSL::PMFS_internal
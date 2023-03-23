#pragma once 
#include <pmfs_helper_classes.h>

namespace PMFS
{
    class PMFS_GSL;

    struct Filament{
        Utils::Vector2 position;
        bool active = false;
    };

    struct SimulationSource{
        enum Mode {Quadtree, Point};


        const Mode mode;
        const NQA::Node* nqaNode;
        const Utils::Vector2 point;

        const PMFS_GSL* grid; 

        SimulationSource(const Utils::Vector2& _point, const PMFS_GSL* _grid): point(_point), mode(Mode::Point), grid(_grid){}
        SimulationSource(const NQA::Node* _node, const PMFS_GSL* _grid): nqaNode(_node), mode(Mode::Quadtree), grid(_grid){}

        Utils::Vector2 getPoint() const;
    };
    
    class Simulations
    {
        public:
        PMFS_GSL* grid;

        void updateSourceProbability(float refineFraction);
        void compareRefineFractions();
        std::vector<std::vector<NQA::Node*> > mapSegmentation;
        std::unique_ptr<NQA::NQAQuadtree> quadtree;
        std::vector<NQA::Node> QTleaves;
        std::vector<std::vector<double> > varianceOfHitProb; //calculated from the simulations, used for movement

        void moveFilament(Filament& filament, Vector2Int& indices, float deltaTime, float noiseSTDev);
        void simulateSourceInPosition(const SimulationSource& source, std::vector<std::vector<float> >& hitMap, bool warmup, int warmupLimit, int timesteps, float deltaTime, float noiseSTDev);
        bool filamentIsOutside(Filament& filament);
        double weightedDifference(const std::vector<std::vector<Cell> >& hitRandomVariable, const std::vector<std::vector<float> >& hitMap);
        void printImage(const SimulationSource& source);
        bool moveAlongPath(Utils::Vector2& beginning, const Utils::Vector2& end);

    };
}
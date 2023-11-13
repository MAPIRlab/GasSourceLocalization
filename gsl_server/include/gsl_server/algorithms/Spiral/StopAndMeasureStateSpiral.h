#pragma once
#include <gsl_server/algorithms/Common/StopAndMeasureState.h>

namespace GSL
{
    class StopAndMeasureStateSpiral : public StopAndMeasureState
    {
    public:
        void OnEnterState(State* previous) override;
        StopAndMeasureStateSpiral(Algorithm* _algorithm);

        void addGasReading(double concentration) override;
        double getSumOfLocalMaxima();

        double intervalLength;

    private:
        class Spiral* spiral;

        struct MeasureInterval
        {
            std::vector<double> gasMeasurements;
        };
        std::vector<MeasureInterval> intervals;
    };
} // namespace GSL
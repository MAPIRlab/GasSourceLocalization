#pragma once
#include <gsl_server/algorithms/Algorithm.h>

namespace GSL
{
    class Spiral : public Algorithm
    {
        friend class StopAndMeasureStateSpiral;

    public:
        Spiral(std::shared_ptr<rclcpp::Node> _node) : Algorithm(_node)
        {
        }
        ~Spiral()
        {
        }

        void initialize() override;

    protected:
        void declareParameters() override;
        void processGasAndWindMeasurements(double concentration, double wind_speed, double wind_direction) override;
        double getProximityIndex();

        void doSpiral();
        void resetSpiral();
        NavigateToPose::Goal nextGoalSpiral(const Pose& initial);

        double spiralStepSize;
        double initSpiralStepSize;
        int spiral_iter;
        double spiralStep_increment;

        double stepIncrement;
        double Kmu;
        double Kp;
    };
} // namespace GSL
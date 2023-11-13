#include <gsl_server/algorithms/Spiral/StopAndMeasureStateSpiral.h>
#include <gsl_server/algorithms/Spiral/Spiral.h>

namespace GSL
{
    void StopAndMeasureStateSpiral::OnEnterState(State* previous)
    {
        StopAndMeasureState::OnEnterState(previous);
        intervals.clear();
    }

    StopAndMeasureStateSpiral::StopAndMeasureStateSpiral(Algorithm* _algorithm) : StopAndMeasureState(_algorithm)
    {
        spiral = dynamic_cast<Spiral*>(algorithm);
        intervalLength = spiral->getParam<double>("intervalLength", 0.5);
    }

    void StopAndMeasureStateSpiral::addGasReading(double concentration)
    {
        StopAndMeasureState::addGasReading(concentration);
        static rclcpp::Time timeStampLastInterval = rclcpp::Time(-100);

        if ((spiral->node->now() - timeStampLastInterval).seconds() > intervalLength)
        {
            intervals.back().gasMeasurements.push_back(concentration);
            timeStampLastInterval = spiral->node->now();
        }
    }

    double StopAndMeasureStateSpiral::getSumOfLocalMaxima()
    {
        double sum = 0.0;
        for (const MeasureInterval& interval : intervals)
        {
            const auto& vector = interval.gasMeasurements;
            if (!vector.empty())
                sum += (double)*std::max_element(std::begin(vector), std::end(vector));
        }
        return sum;
    }
} // namespace GSL
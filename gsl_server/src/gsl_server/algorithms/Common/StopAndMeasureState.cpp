#include <gsl_server/algorithms/Common/StopAndMeasureState.hpp>
#include <gsl_server/algorithms/Common/Algorithm.hpp>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>

namespace GSL
{
    StopAndMeasureState::StopAndMeasureState(Algorithm* _algorithm) : State(_algorithm)
    {
        measure_time = algorithm->getParam<double>("stop_and_measure_time", 2.0);
    }

    void StopAndMeasureState::OnEnterState(State* previous)
    {
        GSL_TRACE("Entering StopAndMeasure");
        time_stopped = algorithm->node->now();
        gas_v.clear();
        windSpeed_v.clear();
        windDirection_v.clear();
    }

    void StopAndMeasureState::OnUpdate()
    {
        if ((algorithm->node->now() - time_stopped).seconds() >= measure_time)
        {
            double concentration = average_concentration();
            double windSpeed = average_windSpeed();
            double windDirection = average_windDirection();

            GSL_INFO("avg_gas={:.2};  avg_windSpeed={:.2};  avg_wind_dir={:.2}", concentration, windSpeed, windDirection);
            algorithm->processGasAndWindMeasurements(concentration, windSpeed, windDirection);
        }
    }

    void StopAndMeasureState::addGasReading(double concentration)
    {
        if (algorithm->stateMachine.getCurrentState() != this)
            return;
        gas_v.push_back(concentration);
    }

    void StopAndMeasureState::addWindReading(double speed, double direction)
    {
        if (algorithm->stateMachine.getCurrentState() != this)
            return;
        windSpeed_v.push_back(speed);
        windDirection_v.push_back(direction);
    }

    void StopAndMeasureState::OnExitState(State* next)
    {
        if (next != this)
            algorithm->currentResult = algorithm->checkSourceFound();
    }

    double StopAndMeasureState::average_concentration()
    {
        float average = Utils::getAverageFloatCollection(gas_v.begin(), gas_v.end());
        if (average == Utils::INVALID_AVERAGE)
        {
            GSL_WARN("No gas measurements were received during StopAndMeasure!");
            return 0;
        }
        return average;
    }
    double StopAndMeasureState::average_windDirection()
    {
        float average = Utils::getAverageDirection(windDirection_v.begin(), windDirection_v.end());
        if (average == Utils::INVALID_AVERAGE)
            return 0;
        return average;
    }
    double StopAndMeasureState::average_windSpeed()
    {
        float average = Utils::getAverageFloatCollection(windSpeed_v.begin(), windSpeed_v.end());
        if (average == Utils::INVALID_AVERAGE)
        {
            GSL_WARN("No wind measurements were received during StopAndMeasure!");
            return 0;
        }
        return average;
    }
} // namespace GSL
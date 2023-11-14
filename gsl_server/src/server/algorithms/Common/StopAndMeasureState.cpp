#include <gsl_server/algorithms/Common/StopAndMeasureState.hpp>
#include <gsl_server/algorithms/Algorithm.hpp>
#include <gsl_server/Utils/Math.hpp>

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
            double wind_speed = average_wind_speed();
            double wind_direction = average_wind_direction();

            GSL_INFO("avg_gas={:.2};  avg_wind_speed={:.2};  avg_wind_dir={:.2}", concentration, wind_speed, wind_direction);
            algorithm->processGasAndWindMeasurements(concentration, wind_speed, wind_direction);
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
        algorithm->currentResult = algorithm->checkSourceFound();
    }

    double StopAndMeasureState::average_concentration()
    {
        return Utils::getAverageVector(gas_v);
    }
    double StopAndMeasureState::average_wind_direction()
    {
        return Utils::getAverageDirection(windDirection_v);
    }
    double StopAndMeasureState::average_wind_speed()
    {
        return Utils::getAverageVector(windSpeed_v);
    }
} // namespace GSL
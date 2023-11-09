#include <gsl_server/algorithms/Common/StopAndMeasureState.h>
#include <gsl_server/algorithms/Algorithm.h>

namespace GSL
{
    void StopAndMeasureState::OnEnterState(State* previous)
    {
        GSL_TRACE("Entering StopAndMeasure");
        time_stopped = algorithm->node->now();
        gas_v.clear();
        windS_v.clear();
        windD_v.clear();
    }

    void StopAndMeasureState::OnExitState(State* next)
    {
    }

    double StopAndMeasureState::average_concentration()
    {
        return get_average_vector(gas_v);
    }
    double StopAndMeasureState::average_wind_direction()
    {
        return get_average_vector(windS_v);
    }
    double StopAndMeasureState::average_wind_speed()
    {
        return get_average_vector(windD_v);
    }
} // namespace GSL
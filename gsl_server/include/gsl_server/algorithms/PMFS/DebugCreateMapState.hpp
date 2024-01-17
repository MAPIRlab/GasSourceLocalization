#pragma once
#include <gsl_server/algorithms/Common/GSLState.hpp>
#include <gsl_server/algorithms/PMFS/internal/PublishersAndSubscribers.hpp>
#include <gsl_server/Utils/Time.hpp>

namespace GSL
{
    class PMFS;

    class DebugCreateMapState : public State
    {
    public:
        DebugCreateMapState(Algorithm* _algorithm);
        void OnEnterState(State* previousState) override;
        void OnExitState(State* nextState) override;
        void OnUpdate() override;

    private:
        PMFS* pmfs;
        rclcpp::Client<gaden_player::srv::GasPosition>::SharedPtr gasClient;
        float anemometerZ;
        int iterationsCount;
        Utils::Time::Countdown timer;
    };
}
#include <gsl_server/algorithms/PlumeTracking/PlumeTracking.h>
#include <gsl_server/algorithms/PlumeTracking/MovingStatePlumeTracking.h>
#include <gsl_server/Utils/Math.h>

namespace GSL
{
    void MovingStatePlumeTracking::OnUpdate()
    {
        PlumeTracking* plumeTracking = dynamic_cast<PlumeTracking*>(algorithm);
        bool gasHit = Utils::getAverageDeque(plumeTracking->lastConcentrationReadings) > plumeTracking->thresholdGas;

        bool foundGas = gasHit && (currentMovement == PTMovement::Exploration || currentMovement == PTMovement::RecoverPlume);
        bool lostGas = !gasHit && (currentMovement == PTMovement::FollowPlume);
        if (foundGas || lostGas)
            plumeTracking->stateMachine.forceResetState(plumeTracking->stopAndMeasureState.get());
    }

} // namespace GSL
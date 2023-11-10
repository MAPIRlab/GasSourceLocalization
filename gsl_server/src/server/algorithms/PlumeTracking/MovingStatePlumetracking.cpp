#include <gsl_server/algorithms/PlumeTracking/PlumeTracking.h>
#include <gsl_server/algorithms/PlumeTracking/MovingStatePlumeTracking.h>
#include <gsl_server/Utils/Math.h>

namespace GSL
{
    void MovingStatePlumeTracking::OnUpdate()
    {
        // TODO unifinished
        bool gasHit = Utils::get_average_deque(dynamic_cast<PlumeTracking*>(algorithm)->lastConcentrationReadings);
        if (currentMovement == PTMovement::Exploration || currentMovement == PTMovement::RecoverPlume)
        {
        }
    }

} // namespace GSL
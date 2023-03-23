#pragma once
#include <gsl_plume_tracking.h>
/*   DESCRIPTION OF SEARCH STATES
 * --------------------------------
 * WAITING_FOR_MAP: No map is available, waiting for map_server to provide one.
 * EXPLORATION: Explore the map, looking for traces of the gas release.
 * INSPECTION: There is gas but no wind information. Search in the surroundings for the gas source.
 * PLUME: Gas and Wind data. Try navigating the plume towards the gas source.
 * STOP_AND_MEASURE: Stop the robot and measure wind and gas for a time-lapse.
 * */


class SurgeCastPT:public PlumeTracking
{

public:
    SurgeCastPT(ros::NodeHandle *nh);
    ~SurgeCastPT();

    void checkState() override;                  //Check gas/wind to see if a state transition is necessary
    void setSurgeGoal() override;                //Set target upwind
    void setCastGoal() override;                 //Set target crosswind
    void save_results_to_file(int result) override;
};

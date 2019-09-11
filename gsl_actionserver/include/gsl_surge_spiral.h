#include <gsl_plume_tracking.h>
/*   DESCRIPTION OF SEARCH STATES
 * --------------------------------
 * WAITING_FOR_MAP: No map is available, waiting for map_server to provide one.
 * EXPLORATION: Explore the map, looking for traces of the gas release.
 * INSPECTION: There is gas but no wind information. Search in the surroundings for the gas source.
 * PLUME: Gas and Wind data. Try navigating the plume towards the gas source.
 * STOP_AND_MEASURE: Stop the robot and measure wind and gas for a time-lapse.
 * */


class SurgeSpiralPT:public PlumeTracking
{

public:
    SurgeSpiralPT(ros::NodeHandle *nh);
    ~SurgeSpiralPT();

    virtual void checkState() override; 

    double spiralStep;
    double spiralStep_increment;
    double initSpiralStep;
    int spiral_iter;


    double deltaT; //measuring interval length
    ros::Time lastUpdateTimestamp;
    
    void setSurgeGoal() override; 

    void setCastGoal() override;
    move_base_msgs::MoveBaseGoal nextGoalSpiral(geometry_msgs::Pose initial);
    void resetSpiral();

};

#include <algorithms/gsl_surge_spiral.h>

/*   DESCRIPTION OF SEARCH STATES
 * --------------------------------
 * WAITING_FOR_MAP: (initialization)No map is available, waiting to map_server to provide one.
 * EXPLORATION: Explore the map, looking for traces of the gas release.
 * INSPECTION: A gas hit has been detected, so search in the surroundings for the gas source.
 * PLUME (SURGE,CAT): A gas plume may exists, try navigating it towards the gas source.
 * STOP_AND_MEASURE: Stop the robot and measure wind and gas for a time-lapse.
 * */



//-----------------------------------------------------------

    //GSL-PlumeTracking

//-----------------------------------------------------------
SurgeSpiralPT::SurgeSpiralPT(ros::NodeHandle *nh) :
    PlumeTracking(nh)
{
    
    nh->param<double>("initSpiralStep", initSpiralStep, 1);
    nh->param<double>("spiralStep_increment", spiralStep_increment, 0.4);
    nh->param<std::string>("results_file",results_file,"/home/pepe/catkin_ws/src/olfaction/gas_source_localization/results/surge-spiral/results.txt");

    nh->param<double>("deltaT", deltaT, 1);


    lastUpdateTimestamp=ros::Time::now();
    resetSpiral();
    step=1;
}


SurgeSpiralPT::~SurgeSpiralPT()
{
    spdlog::info("- Closing...");
}


//------------------------------------

    //OVERRIDEN PLUME-TRACKING LOGIC

//------------------------------------

void SurgeSpiralPT::checkState()
{  
    //If we are here, that means we are moving towards a target location reactively
    switch (current_state)
    {
    case PT_state::EXPLORATION:
        //We are looking for gas clues
        if(get_average_vector(gasConcentration_v) > th_gas_present)
        {
            if (verbose) spdlog::info("GAS HIT!");
            gasHit=true;
            cancel_navigation();                //Stop Robot
            previous_state = PT_state::EXPLORATION;
            current_state = PT_state::STOP_AND_MEASURE;
            if (verbose) spdlog::warn("[GSL-SurgeSpiral] New state --> STOP_AND_MEASURE");
        }
        break;
    case PT_state::INSPECTION:
        //Check surroundings of current position
        // No break conditions, wait till end of inspection
        break;
    case PT_state::UPWIND_SURGE:
        //We are moving within the gas plume
        if(ros::Time::now().toSec()-lastUpdateTimestamp.toSec()>=deltaT)
        {   
            if(gasConcentration_v.back() >= th_gas_present){

                //if we haven't moved in 3 seconds, the path might not be good, resample everything
                if(ros::Time::now().toSec()-recoveryTimestamp.toSec()>=3){ 
                    current_state = PT_state::STOP_AND_MEASURE;
                    cancel_navigation();
                    if (verbose) spdlog::warn("[GSL-SurgeSpiral] New state --> STOP_AND_MEASURE");
                }
                lastUpdateTimestamp=ros::Time::now();

                double dist = sqrt(pow(current_robot_pose.pose.pose.position.x-movingPose.pose.pose.position.x,2)+
                    pow(current_robot_pose.pose.pose.position.y-movingPose.pose.pose.position.y,2));

                //only send new goals it we are already moving
                if(dist>0.1){ 
                    if (verbose) spdlog::warn("[GSL-SurgeSpiral] More gas! Surge distance has been reset"); 
                    setSurgeGoal();
                    movingTimestamp=ros::Time::now();
                    movingPose=current_robot_pose;
                    recoveryTimestamp=ros::Time::now();

                    return;
                }
            }
            else{
                lastUpdateTimestamp=ros::Time::now();
            }      
        }
        break;
    case PT_state::CROSSWIND_CAST:
        //We are trying to return to the plume
        if(get_average_vector(gasConcentration_v) > th_gas_present)
        {
            if (verbose) spdlog::info("Gas plume found! - Returning to UPWIND_SURGE movement!");
            gasHit=true;
            cancel_navigation();                //Stop Robot
            previous_state = PT_state::CROSSWIND_CAST;
            current_state = PT_state::STOP_AND_MEASURE;
            resetSpiral();
            if (verbose) spdlog::warn("[GSL-SurgeSpiral] New state --> STOP_AND_MEASURE");
        }   
        else{
            if(ros::Time::now().toSec()-lastUpdateTimestamp.toSec()>=deltaT){
                lastUpdateTimestamp=ros::Time::now();
            }
        }     
        break;
    default:
        current_state = PT_state::STOP_AND_MEASURE;
        cancel_navigation();
        spdlog::error("ERROR: State undefined!");
    }
}

void SurgeSpiralPT::setSurgeGoal()
{
    // Initially, get Upwind direction with respect reference /map
    double upwind_dir = angles::normalize_angle(average_wind_direction + 3.14159);

    //Set goal in the Upwind direction
    navigation_assistant::nav_assistantGoal goal;
    current_step = step;
    double movement_dir = upwind_dir;
    do
    {
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        //Set a goal in the upwind direction
        goal.target_pose.pose.position.x = current_robot_pose.pose.pose.position.x + current_step * cos(movement_dir);
        goal.target_pose.pose.position.y = current_robot_pose.pose.pose.position.y + current_step * sin(movement_dir);
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angles::normalize_angle(movement_dir));

        // If goal is unreachable
        //add noise in angle (in case goal is an obstacle)
        movement_dir = upwind_dir + fRand(-(M_PI/4),M_PI/4);
        //reduce step
        current_step = current_step-0.2;

        if (current_step<=0.2)
        {
            spdlog::error("GSL-SurgeSpiral - {} - ERROR: Cannot move further Upwind!", __FUNCTION__);
            cancel_navigation();
            current_state=PT_state::STOP_AND_MEASURE;
            return;
        }
        lastUpdateTimestamp=ros::Time::now();
    }
    while(!checkGoal(&goal));

    //Send goal to the Move_Base node for execution
    if (verbose) spdlog::debug("GSL-SurgeSpiral - {} - Sending robot to {} {}", __FUNCTION__, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    mb_ac.sendGoal(goal, std::bind(&SurgeSpiralPT::goalDoneCallback, this,  std::placeholders::_1, std::placeholders::_2), std::bind(&SurgeSpiralPT::goalActiveCallback, this), std::bind(&SurgeSpiralPT::goalFeedbackCallback, this, std::placeholders::_1));
    inMotion = true;
}
//-------------------------

    //Spiral movement to recover the plume

//-------------------------

void SurgeSpiralPT::resetSpiral(){
    if(verbose) spdlog::info("[GSL-SurgeSpiral] Resetting spiral movement");
    spiralStep=initSpiralStep;
    spiral_iter=1;
}

navigation_assistant::nav_assistantGoal SurgeSpiralPT::nextGoalSpiral(geometry_msgs::Pose initial){

    double yaw=tf::getYaw(initial.orientation);
    navigation_assistant::nav_assistantGoal goal;
    goal.target_pose.header.frame_id="map";
    goal.target_pose.header.stamp=ros::Time::now();
    if(spiral_iter%2==0){
        spiralStep+=spiralStep_increment;
    }
    goal.target_pose.pose.position.x=initial.position.x-(-spiralStep)*sin(yaw);
    goal.target_pose.pose.position.y=initial.position.y+(-spiralStep)*cos(yaw);
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw-M_PI/2);
    spiral_iter++;
    
    spdlog::info("[SPIRAL_SEARCH] New Goal [{:.2}, {:.2}]", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    return goal;
}

void SurgeSpiralPT::setCastGoal(){
    navigation_assistant::nav_assistantGoal goal = nextGoalSpiral(current_robot_pose.pose.pose);
    int i=0;
    bool blocked=false;
    while(ros::ok()&&!checkGoal(&goal)){
        if(verbose) spdlog::info("[GSL-SurgeSpiral] SKIPPING NEXT POINT IN SPIRAL (OBSTACLES)");
        goal=nextGoalSpiral(goal.target_pose.pose);
        i++;
        if(i>3){
            i=0;
            if(blocked){
                resetSpiral();
                goal.target_pose=get_random_pose_environment();
            }else{
                spdlog::info("[GSL-SurgeSpiral] UNABLE TO CONTINUE SPIRAL (OBSTACLES)");
                resetSpiral();
                blocked=true;
            }
        }
    }
    inMotion=true;
    mb_ac.sendGoal(goal, std::bind(&SurgeSpiralPT::goalDoneCallback, this,  std::placeholders::_1, std::placeholders::_2), std::bind(&SurgeSpiralPT::goalActiveCallback, this), std::bind(&SurgeSpiralPT::goalFeedbackCallback, this, std::placeholders::_1));
}


// EOF

#include "gsl_server.h"
#include "gsl_spiral.h"
#include "gsl_particle_filter.h"
#include "gsl_surge_cast.h"
#include "gsl_grid.h"

// GAS SOURCE LOCALIZATION BASED ON SEMANTIC KNOWLEDGE
int CGSLServer::doSurgeCast()
{
    ROS_INFO("[GSL-PlumeTracking] New Action request. Initializing Plume Tracking algorithm.");
    // Create object implementing Plume Tracking
    ros::NodeHandle pn_("~");
    SurgeCastPT pt(&pn_);

    ROS_DEBUG("[GSL-PlumeTracking] inMotion = %s", pt.get_inMotion() ? "true" : "false");
    ROS_DEBUG("[GSL-PlumeTracking] GSLState = %d", pt.get_state());

    //Loop
    ROS_INFO("[GSL-PlumeTracking] Staring Plume Tracking Loop");
    ros::Rate loop_rate(10);
    size_t num_iterations = 10;
    size_t iter_i = 0;
    while(iter_i < num_iterations)
    {
        // check if search ended
        if (pt.checkSourceFound() != -1)
        {
            //Search ended and results are saved to file
            return 1;
        }

        //Check if AS is preempted
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("[GSL-PlumeTracking] %s: Preempted", action_name_.c_str());
            //Cancell the current search
            pt.cancel_navigation();
            // set the action state to preempted, and return failure
            as_.setPreempted();
            return 0;
        }

        //attend subscriptions
        ros::spinOnce();

        //If moving towards a target location, check for gas hits
        if (pt.get_inMotion())
            pt.checkState();
        else
        {
            switch (pt.get_state())
            {
            case PT_state::STOP_AND_MEASURE :
                pt.getGasWindObservations();
                break;
            case PT_state::EXPLORATION :
                pt.setExplorationGoal();
                break;
            case PT_state::INSPECTION :
                pt.setInspectionGoal();
                break;
            case PT_state::UPWIND_SURGE :
                pt.setSurgeGoal();
                break;
            case PT_state::CROSSWIND_CAST :
                pt.setCastGoal();
                break;
            case PT_state::WAITING_FOR_MAP:
                ROS_INFO("[GSL-PlumeTracking] Waiting for the map of the environment!....");    //Waiting call back function
                break;
            default:
                ROS_ERROR("[GSL-PlumeTracking] Search state is undefined!");
            }
        }

        loop_rate.sleep();
    }
}

int CGSLServer::doSpiral()
{
    ros::NodeHandle nh("~");
    SpiralSearcher spiral(&nh);
    ros::Rate loop_rate(10);
    bool blocked=false;
    while(ros::ok()&&spiral.checkSourceFound()==-1)
    {
        if (spiral.checkSourceFound() != -1)
        {
            //Search ended and results are saved to file
            return 1;
        }
        if (!spiral.get_inMotion())
        {
            switch(spiral.getCurrentState()){
                case(SPIRAL_state::WAITING_FOR_MAP):
                    ROS_INFO("[SPIRAL_SEARCH] Waiting for the map of the environment!...."); 
                    break;
                case(SPIRAL_state::STOP_AND_MEASURE):
                    spiral.getGasObservations();
                    break;
                case(SPIRAL_state::SPIRAL):
                    if(!blocked){
                        if(!spiral.doSpiral()){
                            blocked=true;
                        }
                    }else{
                        spiral.setRandomGoal();
                        blocked=false;
                    }
                    break;
                default:
                ROS_ERROR("[SPIRAL_SEARCH] Undefined state");
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

int CGSLServer::doSurgeSpiral()
{
    ROS_INFO("[GSL-PlumeTracking] New Action request. Initializing Plume Tracking algorithm.");
    // Create object implementing Plume Tracking
    ros::NodeHandle pn_("~");
    SurgeSpiralPT pt(&pn_);

    ROS_DEBUG("[GSL-PlumeTracking] inMotion = %s", pt.get_inMotion() ? "true" : "false");
    ROS_DEBUG("[GSL-PlumeTracking] GSLState = %d", pt.get_state());

    //Loop
    ROS_INFO("[GSL-PlumeTracking] Staring Plume Tracking Loop");
    ros::Rate loop_rate(10);
    size_t num_iterations = 10;
    size_t iter_i = 0;
    while(iter_i < num_iterations)
    {
        // check if search ended
        if (pt.checkSourceFound() != -1)
        {
            //Search ended and results are saved to file
            return 1;
        }

        //Check if AS is preempted
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("[GSL-PlumeTracking] %s: Preempted", action_name_.c_str());
            //Cancell the current search
            pt.cancel_navigation();
            // set the action state to preempted, and return failure
            as_.setPreempted();
            return 0;
        }

        //attend subscriptions
        ros::spinOnce();

        //If moving towards a target location, check for gas hits
        if (pt.get_inMotion())
            pt.checkState();
        else
        {
            switch (pt.get_state())
            {
            case PT_state::STOP_AND_MEASURE :
                pt.getGasWindObservations();
                break;
            case PT_state::EXPLORATION :
                pt.setExplorationGoal();
                break;
            case PT_state::INSPECTION :
                pt.setInspectionGoal();
                break;
            case PT_state::UPWIND_SURGE :
                pt.setSurgeGoal();
                break;
            case PT_state::CROSSWIND_CAST :
                pt.setCastGoal();
                break;
            case PT_state::WAITING_FOR_MAP:
                ROS_INFO("[GSL-PlumeTracking] Waiting for the map of the environment!....");    //Waiting call back function
                break;
            default:
                ROS_ERROR("[GSL-PlumeTracking] Search state is undefined!");
            }
        }

        loop_rate.sleep();
    }
}

int CGSLServer::doParticleFilter()
{
    ROS_INFO("[GSL-ParticleFilter] New Action request. Initializing Plume Tracking algorithm.");
    // Create object implementing Plume Tracking
    ros::NodeHandle pn_("~");
    ParticleFilter pt(&pn_);

    double source_poseX;
    double source_poseY;
    pn_.param<std::string>("resultsFile", resultsFile, "");
    pn_.param<double>("source_pose_x", source_poseX, 0.0);
    pn_.param<double>("source_pose_y", source_poseY, 0.0);
    ROS_DEBUG("[GSL-ParticleFilter] inMotion = %s", pt.get_inMotion() ? "true" : "false");
    ROS_DEBUG("[GSL-ParticleFilter] GSLState = %d", pt.get_state());

    //Loop
    ROS_INFO("[GSL-ParticleFilter] Staring Plume Tracking Loop");
    ros::Rate loop_rate(10);
    size_t num_iterations = 10;
    size_t iter_i = 0;

    while(iter_i < num_iterations)
    {
        if(pt.checkSourceFound()!=-1){
            return 1;
        }

        //Check if AS is preempted
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("[GSL-ParticleFilter] %s: Preempted", action_name_.c_str());
            //Cancel the current search
            pt.cancel_navigation();
            // set the action state to preempted, and return failure
            as_.setPreempted();
            return 0;
        }
    
        //attend subscriptions
        ros::spinOnce();

        //If moving towards a target location, check for gas hits
        if (pt.get_inMotion())
            pt.checkState();
        else
        {
            switch (pt.get_state())
            {
            case PT_state::STOP_AND_MEASURE :
                pt.getGasWindObservations();
                break;
            case PT_state::EXPLORATION :
                pt.firstObserv=false;
                pt.setExplorationGoal();
                break;
            case PT_state::INSPECTION :
                if(!pt.firstObserv){
                    visualization_msgs::Marker points=pt.emptyMarker();
                    pt.generateParticles(points);
                    pt.firstObserv=true;
                }else{
                    if(pt.particlesConverge()){
                        pt.estimateLocation();
                    }
                    pt.updateWeights(true);
                    if(pt.isDegenerated()){
                        pt.resample();
                    }
                }
                pt.setInspectionGoal();
                break;
            case PT_state::UPWIND_SURGE :
                if(!pt.firstObserv){
                    visualization_msgs::Marker points=pt.emptyMarker();
                    pt.generateParticles(points);
                    pt.firstObserv=true;
                }else{
                    if(pt.particlesConverge()){
                        pt.estimateLocation();
                    }
                    pt.updateWeights(true);
                    if(pt.isDegenerated()){
                        pt.resample();
                    }
                }
                pt.setSurgeGoal();
                break;
            case PT_state::CROSSWIND_CAST :
                if(pt.particlesConverge()){
                    pt.estimateLocation();
                }
                pt.updateWeights(false);
                if(pt.isDegenerated()){
                    pt.resample();
                }
                pt.setCastGoal();
                break;
            case PT_state::WAITING_FOR_MAP:
                ROS_INFO("[GSL-ParticleFilter] Waiting for the map of the environment!....");    //Waiting call back function
                break;
            default:
                ROS_ERROR("[GSL-ParticleFilter] Search state is undefined!");
            }
        }

        loop_rate.sleep();
    }
}

int CGSLServer::doGrid()
{
    ros::NodeHandle nh("~");
    GridGSL grid(&nh);
    ros::Rate loop_rate(10);
    while(ros::ok()&&grid.checkSourceFound()==-1)
    {
        ros::spinOnce();
        if(!grid.get_inMotion()){
            switch(grid.getState()){
                case Grid_state::WAITING_FOR_MAP:
                    ROS_INFO("[GSL-PlumeTracking] Waiting for the map of the environment!...."); 
                    break;
                case Grid_state::STOP_AND_MEASURE:
                    grid.getGasWindObservations();
                    break;
                case Grid_state::MOVING:
                    grid.setGoal();
                    break;
                case Grid_state::EXPLORATION:
                    break;
                default:
                    ROS_ERROR("[GSL-Grid] Search state is undefined!");
            }
        }
        
        loop_rate.sleep();
    }
    return 1;
}

//=======================================================
// Action Server Callback when Goal is received (START!)
//=======================================================
void CGSLServer::executeCB(const gsl_actionserver::gsl_action_msgGoalConstPtr &goal)
{
    int res;

    //1. Start the localization of the gas source

    if ( goal->gsl_method == "surge_cast" )
    {
        res = doSurgeCast();
    }
    else if(goal->gsl_method == "spiral"){
        res = doSpiral();
    }
    else if(goal->gsl_method == "surge_spiral"){
        res = doSurgeSpiral();
    }
    else if(goal->gsl_method == "particle_filter"){
        res=doParticleFilter();
    }
    else if(goal->gsl_method == "grid"){
        res=doGrid();
    }
    else
        ROS_ERROR("[GSL_server] Invalid GSL method: %s, candidates are: 'surge_cast', 'surge_spiral, 'spiral', 'particle_filter", goal->gsl_method.c_str());


    //2. Return result
    // res: -1(fail) 0(cancelled) 1(sucess)
    if (res == 1) // Completed
    {
        result_.success = 1;
        ROS_INFO("[GSL_server]%s: I found the emission source! ", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }
    else if (res==0)  // Canceled/Preempted by user?
    {
        result_.success = 0;
        ROS_INFO("[GSL_server] %s: Action cancelled/preemted", action_name_.c_str());
        // set the action state to succeeded (end)
        as_.setPreempted(result_);
    }
    else //failure (-1)
    {
        result_.success = 0;
        ROS_INFO("[GSL_server] %s: Couldn't find the gas source! SORRY!", action_name_.c_str());
        // set the action state to succeeded (end)
        as_.setSucceeded(result_);
    }
    ros::shutdown();
}

//===================================================================================
//================================== MAIN ===========================================
//===================================================================================
int main(int argc, char** argv)
{
    ros::init(argc,argv,"gsl_ac");

    CGSLServer gsl_node("gsl");
    ROS_INFO("[GSL_server] GSL action server is ready for action!...");

    ros::spin();

    return 0;
}


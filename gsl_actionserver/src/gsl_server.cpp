#include "gsl_server.h"
#include "algorithms/gsl_spiral.h"
#include "algorithms/gsl_particle_filter.h"
#include "algorithms/gsl_surge_cast.h"
#include "algorithms/PMFS/PMFS.h"
#include "algorithms/gsl_GrGSL.h"
#include "gsl_imgui.h"
#include "gsl_implot.h"

#include <thread>


int CGSLServer::doSurgeCast()
{
    spdlog::info(" New Action request. Initializing Plume Tracking algorithm.");
    // Create object implementing Plume Tracking
    ros::NodeHandle pn_("~");
    SurgeCastPT pt(&pn_);

    spdlog::debug(" inMotion = {}", pt.get_inMotion() ? "true" : "false");

    //Loop
    spdlog::info(" Staring Plume Tracking Loop");
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
            spdlog::info(" {}: Preempted", action_name_.c_str());
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
                spdlog::info(" Waiting for the map of the environment!....");    //Waiting call back function
                break;
            default:
                spdlog::error(" Search state is undefined!");
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
                    spdlog::info("[SPIRAL_SEARCH] Waiting for the map of the environment!...."); 
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
                spdlog::error("[SPIRAL_SEARCH] Undefined state");
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

int CGSLServer::doSurgeSpiral()
{
    spdlog::info(" New Action request. Initializing Plume Tracking algorithm.");
    // Create object implementing Plume Tracking
    ros::NodeHandle pn_("~");
    SurgeSpiralPT pt(&pn_);

    spdlog::debug(" inMotion = {}", pt.get_inMotion() ? "true" : "false");

    //Loop
    spdlog::info(" Staring Plume Tracking Loop");
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
            spdlog::info(" {}: Preempted", action_name_.c_str());
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
                spdlog::info(" Waiting for the map of the environment!....");    //Waiting call back function
                break;
            default:
                spdlog::error(" Search state is undefined!");
            }
        }

        loop_rate.sleep();
    }
}

int CGSLServer::doParticleFilter()
{
    spdlog::info("[GSL-ParticleFilter] New Action request. Initializing Plume Tracking algorithm.");
    // Create object implementing Plume Tracking
    ros::NodeHandle pn_("~");
    ParticleFilter pt(&pn_);

    double source_poseX;
    double source_poseY;
    pn_.param<std::string>("resultsFile", resultsFile, "");
    pn_.param<double>("source_pose_x", source_poseX, 0.0);
    pn_.param<double>("source_pose_y", source_poseY, 0.0);
    spdlog::debug("[GSL-ParticleFilter] inMotion = {}", pt.get_inMotion() ? "true" : "false");

    //Loop
    spdlog::info("[GSL-ParticleFilter] Staring Plume Tracking Loop");
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
            spdlog::info("[GSL-ParticleFilter] {}: Preempted", action_name_.c_str());
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
                spdlog::info("[GSL-ParticleFilter] Waiting for the map of the environment!....");    //Waiting call back function
                break;
            default:
                spdlog::error("[GSL-ParticleFilter] Search state is undefined!");
            }
        }

        loop_rate.sleep();
    }
}

int CGSLServer::doGrGSL()
{
    using namespace GrGSL;
    ros::NodeHandle nh("~");
    GrGSL::GrGSL grgsl(&nh);
    ros::Rate loop_rate(2);
    int sourceFound = -1;
    double startTime = ros::Time::now().toSec();

    while(ros::ok()&&sourceFound==-1 && ros::Time::now().toSec()-startTime<grgsl.max_search_time*1.2)
    {
        if(!grgsl.get_inMotion()){
            switch(grgsl.getState()){
                case State::STOP_AND_MEASURE:
                    grgsl.getGasWindObservations();
                    grgsl.showWeights();
                    break;
                case State::MOVING:
                    sourceFound = grgsl.checkSourceFound();
                    grgsl.setGoal();

                    grgsl.showWeights();
                    break;
                case State::EXPLORATION:
                    break;
                default:
                    spdlog::error("[GSL-Grid] Search state is undefined!");
            }
        }
        
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 1;
}

int CGSLServer::doPMFS()
{
    using namespace PMFS;
    ros::NodeHandle nh("~");
    PMFS_GSL grid(&nh);
    ros::Rate loop_rate(2);
    int sourceFound = -1;
    double startTime = ros::Time::now().toSec();
    bool initializationStarted = false;

    while(grid.getState() == State::WAITING_FOR_MAP){
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    grid.initialize();
    
    std::thread renderThread = std::thread(&PMFS_GSL::renderImgui, &grid);

    while(ros::ok()&&sourceFound==-1 && ros::Time::now().toSec()-startTime<grid.max_search_time*1.2)
    {
        grid.runSubmitedQueue();
        if(!grid.get_inMotion() && !grid.debugStuff.paused){
            PMFS::State state = grid.getState();

            if(state == State::STOP_AND_MEASURE)
                    grid.processGasWindObservations();
            else if( state == State::MOVING || state == State::EXPLORATION)
            {
                sourceFound = grid.checkSourceFound();
                grid.setGoal();

                grid.showWeights();
                grid.showDebugInfo();
                grid.plotWindVectors();
            }
            else
                spdlog::error("[GSL-Grid] Search state is undefined!");
        }
        else if(grid.debugStuff.paused){
            grid.showWeights();
            grid.showDebugInfo();   
            grid.plotWindVectors();
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
    
    grid.finished = true;
    renderThread.join();
    return sourceFound;
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
    else if(goal->gsl_method == "GrGSL"){
        res=doGrGSL();
    }
    else if(goal->gsl_method == "PMFS"){
        res=doPMFS();
    }
    else
        spdlog::error("[GSL_server] Invalid GSL method: \"{}\", candidates are:\n 'surge_cast', 'surge_spiral, 'spiral', 'particle_filter', 'grid', 'PMFS'", goal->gsl_method.c_str());


    //2. Return result
    // res: -1(fail) 0(cancelled) 1(sucess)
    if (res == 1) // Completed
    {
        result_.success = 1;
        spdlog::info("[GSL_server]{}: I found the emission source! ", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }
    else if (res==0)  // Canceled/Preempted by user?
    {
        result_.success = 0;
        spdlog::info("[GSL_server] {}: Action cancelled/preemted", action_name_.c_str());
        // set the action state to succeeded (end)
        as_.setPreempted(result_);
    }
    else //failure (-1)
    {
        result_.success = 0;
        spdlog::info("[GSL_server] {}: Couldn't find the gas source! SORRY!", action_name_.c_str());
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
    spdlog::set_pattern("[%^%l%$] (%T) [GSL] %v");
    ros::init(argc,argv,"gsl_ac");

    CGSLServer gsl_node("gsl");
    spdlog::info("[GSL_server] GSL action server is ready for action!...");

    ros::spin();

    return 0;
}


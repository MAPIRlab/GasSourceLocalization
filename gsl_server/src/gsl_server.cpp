#include "gsl_server.h"
#include "algorithms/gsl_spiral.h"
#include "algorithms/gsl_particle_filter.h"
#include "algorithms/gsl_surge_cast.h"
#include "algorithms/PMFS/PMFS.h"
#include "algorithms/gsl_GrGSL.h"
#include "gsl_imgui.h"
#include "gsl_implot.h"

#include <thread>


int main(int argc, char** argv)
{
    spdlog::set_pattern("[%^%l%$] (%T) [GSL] %v");
    rclcpp::init(argc,argv);

    auto gsl_node = std::make_shared<CGSLServer>("gsl");
    spdlog::info("[GSL_server] GSL action server is ready for action!...");

    while(rclcpp::ok())
    {
        rclcpp::spin_some(gsl_node);
        if(gsl_node->m_activeGoal.get() == nullptr)
        {
            gsl_node->execute(gsl_node->m_activeGoal);
            rclcpp::shutdown();
        }
    }

    return 0;
}


void CGSLServer::execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<DoGSL>> goal_handle)
{
    int res;

    //1. Start the localization of the gas source

    if ( goal_handle->get_goal()->gsl_method == "surge_cast" )
    {
        res = doSurgeCast();
    }
    else if(goal_handle->get_goal()->gsl_method == "spiral"){
        res = doSpiral();
    }
    else if(goal_handle->get_goal()->gsl_method == "surge_spiral"){
        res = doSurgeSpiral();
    }
    else if(goal_handle->get_goal()->gsl_method == "particle_filter"){
        res=doParticleFilter();
    }
    else if(goal_handle->get_goal()->gsl_method == "GrGSL"){
        res=doGrGSL();
    }
    else if(goal_handle->get_goal()->gsl_method == "PMFS"){
        res=doPMFS();
    }
    else
        spdlog::error("[GSL_server] Invalid GSL method: \"{}\", candidates are:\n 'surge_cast', 'surge_spiral, 'spiral', 'particle_filter', 'grid', 'PMFS'", goal_handle->get_goal()->gsl_method.c_str());


    //2. Return result
    // res: -1(fail) 0(cancelled) 1(sucess)
    auto result = std::make_shared<DoGSL::Result>();
    if (res == 1) // Completed
    {
        result->success = 1;
        spdlog::info("[GSL_server]{}: I found the emission source! ", action_name_.c_str());
        // set the action state to succeeded
        goal_handle->succeed(result);
    }
    else if (res==0)  // Canceled/Preempted by user?
    {
        result->success = 0;
        spdlog::info("[GSL_server] {}: Action cancelled/preemted", action_name_.c_str());
        // set the action state to succeeded (end)
        goal_handle->succeed(result);
    }
    else //failure (-1)
    {
        result->success = 0;
        spdlog::info("[GSL_server] {}: Couldn't find the gas source! SORRY!", action_name_.c_str());
        // set the action state to succeeded (end)
        goal_handle->succeed(result);
    }
    rclcpp::shutdown();
}


rclcpp_action::GoalResponse  CGSLServer::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DoGSL::Goal> goal)
{
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CGSLServer::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<DoGSL>> goal_handle)
{
    rclcpp::shutdown();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CGSLServer::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<DoGSL>> goal_handle)
{
    m_activeGoal = goal_handle;
}

int CGSLServer::doSurgeCast()
{
    spdlog::info(" New Action request. Initializing Plume Tracking algorithm.");
    SurgeCastPT pt(shared_from_this());

    spdlog::debug(" inMotion = {}", pt.get_inMotion() ? "true" : "false");

    //Loop
    spdlog::info(" Staring Plume Tracking Loop");
    rclcpp::Rate loop_rate(10);
    size_t num_iterations = 10;
    size_t iter_i = 0;
    while(iter_i < num_iterations && rclcpp::ok())
    {
        // check if search ended
        if (pt.checkSourceFound() != -1)
        {
            //Search ended and results are saved to file
            return 1;
        }

        //attend subscriptions
        rclcpp::spin_some(shared_from_this());

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
    return 0;
}

int CGSLServer::doSpiral()
{
    SpiralSearcher spiral(shared_from_this());
    rclcpp::Rate loop_rate(10);
    bool blocked=false;
    while(rclcpp::ok()&&spiral.checkSourceFound()==-1)
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
        rclcpp::spin_some(shared_from_this());
        loop_rate.sleep();
    }
    return 0;
}

int CGSLServer::doSurgeSpiral()
{
    spdlog::info(" New Action request. Initializing Plume Tracking algorithm.");
    SurgeSpiralPT pt(shared_from_this());

    spdlog::debug(" inMotion = {}", pt.get_inMotion() ? "true" : "false");

    //Loop
    spdlog::info(" Staring Plume Tracking Loop");
    rclcpp::Rate loop_rate(10);
    size_t num_iterations = 10;
    size_t iter_i = 0;
    while(iter_i < num_iterations && rclcpp::ok())
    {
        // check if search ended
        if (pt.checkSourceFound() != -1)
        {
            //Search ended and results are saved to file
            return 1;
        }

        //attend subscriptions
        rclcpp::spin_some(shared_from_this());

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
    return 0;
}

int CGSLServer::doParticleFilter()
{
    spdlog::info("[GSL-ParticleFilter] New Action request. Initializing Plume Tracking algorithm.");
    ParticleFilter pt(shared_from_this());

    //Loop
    spdlog::info("[GSL-ParticleFilter] Staring Plume Tracking Loop");
    rclcpp::Rate loop_rate(10);
    size_t num_iterations = 10;
    size_t iter_i = 0;

    while(rclcpp::ok() && iter_i < num_iterations)
    {
        if(pt.checkSourceFound()!=-1){
            return 1;
        }

        rclcpp::spin_some(shared_from_this());

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
                    visualization_msgs::msg::Marker points=pt.emptyMarker();
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
                    visualization_msgs::msg::Marker points=pt.emptyMarker();
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
    return 0;
}

int CGSLServer::doGrGSL()
{
    using namespace GrGSL;
    GrGSL::GrGSL grgsl(shared_from_this());
    rclcpp::Rate loop_rate(2);
    int sourceFound = -1;
    double startTime = now().seconds();

    while(rclcpp::ok()&&sourceFound==-1 && now().seconds()-startTime<grgsl.max_search_time*1.2)
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
        rclcpp::spin_some(shared_from_this());
    }
    return 1;
}

int CGSLServer::doPMFS()
{
    using namespace PMFS;
    PMFS_GSL grid(shared_from_this());
    rclcpp::Rate loop_rate(2);
    int sourceFound = -1;
    double startTime = now().seconds();
    bool initializationStarted = false;

    while(grid.getState() == State::WAITING_FOR_MAP){
        rclcpp::spin_some(shared_from_this());
        loop_rate.sleep();
    }
    
    grid.initialize();
    
    std::thread renderThread = std::thread(&PMFS_GSL::renderImgui, &grid);

    while(rclcpp::ok()&&sourceFound==-1 && now().seconds()-startTime<grid.max_search_time*1.2)
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
        rclcpp::spin_some(shared_from_this());
    }
    
    grid.finished = true;
    renderThread.join();
    return sourceFound;
}

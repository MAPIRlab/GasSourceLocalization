#include "algorithms/gsl_spiral.h"

using namespace std::placeholders;

SpiralSearcher::SpiralSearcher(std::shared_ptr<rclcpp::Node> _node) :
    GSLAlgorithm(_node)
{
    // Subscribers
    //------------
    gas_sub_ = node->create_subscription<olfaction_msgs::msg::GasSensor>(enose_topic,1, std::bind(&SpiralSearcher::gasCallback, this, _1) );
    map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic, 1,  std::bind(&SpiralSearcher::mapCallback, this, _1) );

    // Init State
    previous_state = SPIRAL_state::WAITING_FOR_MAP;
    current_state = SPIRAL_state::WAITING_FOR_MAP;
    spdlog::info("[SPIRAL_SEARCH] INITIALIZATON COMPLETED--> WAITING_FOR_MAP");

    average_concentration=0;

    previousPI=0;
    currentInterval=0;

    //Init variables
    step=initStep;
    consecutive_misses=0;
    spiral_iter=1;
}

void SpiralSearcher::declareParameters()
{
    GSLAlgorithm::declareParameters();
    minPI = node->declare_parameter<double>("minPI", 0.1);
    stop_and_measure_time = node->declare_parameter<double>("stop_and_measure_time", 1.0);
   
    initStep = node->declare_parameter<double>("initial_step", 0.6);
    step_increment = node->declare_parameter<double>("step_increment", 0.3);
    Kmu = node->declare_parameter<double>("Kmu", 0.5);
    Kp = node->declare_parameter<double>("Kp", 1);
    intervalLength = node->declare_parameter<double>("intervalLength", 0.5);

    gas_present_thr = node->declare_parameter<double>("gas_present_thr", 1);
}

SpiralSearcher::~SpiralSearcher(){}

//-------------------------

    // CallBack functions

//-------------------------

void SpiralSearcher::gasCallback(const olfaction_msgs::msg::GasSensor::SharedPtr msg)
{
    //Only if we are in the Stop_and_Measure
    if (this->current_state == SPIRAL_state::STOP_AND_MEASURE)
    {
        stop_and_measure_gas_v[currentInterval].push_back(msg->raw);
    }
}

void SpiralSearcher::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    if (verbose) spdlog::info("[SPIRAL_SEARCH] - {} - Got the map of the environment!", __FUNCTION__);
    //ROS convention is to consider cell [0,0] as the lower-left corner (see http://docs.ros.org/api/nav_msgs/html/msg/MapMetaData.html)
    map_ = *msg;

    if (verbose) spdlog::info("--------------GSL---------------");
    if (verbose) spdlog::info("Occupancy Map dimensions:");
    if (verbose) spdlog::info("x_min:{:.2} x_max:{:.2}   -   y_min:{:.2} y_max:{:.2}",map_.info.origin.position.x, map_.info.origin.position.x+map_.info.width*map_.info.resolution, map_.info.origin.position.y,map_.info.origin.position.y+map_.info.height*map_.info.resolution);
   if (verbose)  spdlog::info("--------------------------------");

    //Start the fun!!
    cancelNavigation();
    resetSpiral();
    previous_state = current_state;
    current_state = SPIRAL_state::SPIRAL;
    start_time = node->now();     //start measuring time
    robot_poses_vector.clear();         //start measuring distance
    inExecution = true;
    inMotion = false;
    spdlog::warn("[SPIRAL_SEARCH] STARTING THE SEARCH --> NEW SPIRAL");
}

void SpiralSearcher::windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg)
{
    
}
//----------------------

    //Gas measurement functions

//----------------------


//Stop the robot and measure gas for X seconds (standing)
//Then switch to corresponding search state
void SpiralSearcher::getGasObservations()
{
    if( (node->now() - time_stopped).seconds() > stop_and_measure_time )
    {
        //Get averaged values of the observations taken while standing
        average_concentration = get_average_vector(stop_and_measure_gas_v);

        double PI=getPI();

        //Check thresholds and set new search-state
        if (verbose){
            spdlog::info("[SPIRAL_SEARCH] Previous PI={:.2}", previousPI);
        } 

        if (PI > previousPI)
        {
            consecutive_misses=0;
            previous_state = current_state;
            previousPI=PI;
            current_state=SPIRAL_state::SPIRAL;
            resetSpiral();
            if (verbose) spdlog::warn("[SPIRAL_SEARCH] NEW SPIRAL --> Proximity Index:{:.4}",PI);
        }
        else
        {
            //Nothing
            previous_state = current_state;
            current_state=SPIRAL_state::SPIRAL;
            if (verbose) spdlog::warn("[SPIRAL_SEARCH] CONTINUE --> Proximity Index:{:.4}", PI);
            if(consecutive_misses>3){
                previousPI=previousPI/2.0;
                if (verbose) spdlog::warn("[SPIRAL_SEARCH] PI reduced to:{:.4}",previousPI);

                consecutive_misses=0;
            }else{
                consecutive_misses++;
            }
        }
    }
    else if( (node->now() - lastInterval).seconds() > intervalLength )
    {
        lastInterval=node->now();
        stop_and_measure_gas_v.push_back(std::vector<double>());
        currentInterval++;
    }
}

double SpiralSearcher::getSumOfLocalMaxima(std::vector<std::vector<double> > const &v){
    double sum=0.0;
    for(const std::vector<double> &w:v){
        if(!w.empty())
            sum+=(double) *std::max_element(std::begin(w),std::end(w));
    }
    return sum;
}

double SpiralSearcher::getPI(){
    double P = getSumOfLocalMaxima(stop_and_measure_gas_v);

    //Proximity Index (used to determine if we are getting closer to the source)
    return Kmu*average_concentration+Kp*P*intervalLength;
}
//----------------------

    //Movement functions

//----------------------
  
void SpiralSearcher::cancelNavigation()
{
    nav_client->async_cancel_all_goals();               //Cancel current navigations
    inMotion = false;

    //Start a new measurement-phase while standing
    stop_and_measure_gas_v.clear();
    stop_and_measure_gas_v.push_back(std::vector<double>());
    time_stopped = node->now();    //Start timer for initial measurement
    lastInterval = time_stopped;
    currentInterval=0;
}

void SpiralSearcher::resetSpiral(){
    step=initStep;
    spiral_iter=-1;
}

NavAssistant::Goal SpiralSearcher::nextGoalSpiral(Pose initial){

    double yaw=Utils::getYaw(initial.orientation);
    NavAssistant::Goal goal;
    goal.target_pose.header.frame_id="map";
    goal.target_pose.header.stamp=node->now();
    if(spiral_iter==-1){
        goal.target_pose.pose.position.x=initial.position.x+step*cos(yaw)-step*sin(yaw);
        goal.target_pose.pose.position.y=initial.position.y+step*sin(yaw)+step*cos(yaw);
        goal.target_pose.pose.orientation = Utils::createQuaternionMsgFromYaw(yaw+M_PI/4);
        spiral_iter=1;
    }else{
        if(spiral_iter%2==0){
            step+=step_increment;
        }
        goal.target_pose.pose.position.x=initial.position.x-(-step)*sin(yaw);
        goal.target_pose.pose.position.y=initial.position.y+(-step)*cos(yaw);
        goal.target_pose.pose.orientation = Utils::createQuaternionMsgFromYaw(yaw-M_PI/2);
        spiral_iter++;
    }
    
    spdlog::info("[SPIRAL_SEARCH] New Goal [{:.2}, {:.2}]", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    return goal;
}

bool SpiralSearcher::doSpiral(){
    NavAssistant::Goal goal = nextGoalSpiral(current_robot_pose.pose.pose);
    int i=0;
    bool blocked=false;
    while(rclcpp::ok()&&!checkGoal(goal)){
        if(verbose) spdlog::info("[SPIRAL_SEARCH] SKIPPING NEXT POINT IN SPIRAL (OBSTACLES)");
        goal=nextGoalSpiral(goal.target_pose.pose);
        i++;
        if(i>3){
            //if we fail 3 consecutive times, reset the spiral
            //if it happens again, abort the spiral and go to a random nearby position
            i=0;
            if(blocked){return false;}
            spdlog::info("[SPIRAL_SEARCH] UNABLE TO CONTINUE SPIRAL (OBSTACLES)");
            resetSpiral();
            blocked=true;
        }
    }
    inMotion=true;
    sendGoal(goal);

    return true;
}

NavAssistant::Goal SpiralSearcher::get_random_pose_environment()
{
    bool valid_pose = false;

    NavAssistant::Goal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = node->now();
    double randomPoseDistance=1;
    while (!valid_pose)
    {
        //random pose in the vecinity of the robot
        goal.target_pose.pose.position.x = fRand(current_robot_pose.pose.pose.position.x-randomPoseDistance, current_robot_pose.pose.pose.position.x+randomPoseDistance);
        goal.target_pose.pose.position.y = fRand(current_robot_pose.pose.pose.position.x-randomPoseDistance, current_robot_pose.pose.pose.position.x+randomPoseDistance);
        goal.target_pose.pose.orientation = Utils::createQuaternionMsgFromYaw(0.0);
        randomPoseDistance+=0.5;
        //check pose validity
        if( checkGoal(goal))
            valid_pose = true;
        else if (verbose) 
            spdlog::warn("[SPIRAL_SEARCH] invalid random pose=[{:.2}, {:.2}, {:.2}]", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.z);
    }
   
    //show content
    if (verbose) 
        spdlog::info("[SPIRAL_SEARCH] Random Goal pose =[{:.2}, {:.2}, {:.2}]", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.z);
    return goal;
}

//Set a random goal within the map (EXPLORATION)
void SpiralSearcher::setRandomGoal()
{
    NavAssistant::Goal goal = get_random_pose_environment();

    //Send goal to the Move_Base node for execution
    if (verbose) 
        spdlog::info("[SPIRAL_SEARCH] - {} - Sending robot to {} {}", __FUNCTION__, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    sendGoal(goal);
    inMotion = true;
}

//------------------------

    //getters

//------------------------

SPIRAL_state SpiralSearcher::getCurrentState(){
    return current_state;
}

//------------------------

    //Auxiliary functions

//------------------------

double SpiralSearcher::get_average_vector(std::vector<std::vector<double> > const &v)
{
    size_t length = 0;
    for(const std::vector<double>& w:v){
        length+=w.size();
    }
    double sum = 0.0;
    for(const std::vector<double>& w:v){
        for(const double &f:w){
            sum +=f;
        }
    }
    return sum/length;
}

double SpiralSearcher::fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}
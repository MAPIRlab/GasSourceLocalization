#include "gsl_spiral.h"


SpiralSearcher::SpiralSearcher(ros::NodeHandle *nh) :
    nh_(nh),
    mb_ac("move_base", true)
{
    srand(time(NULL));      //initialize random seed

    //tell the action client that we want to spin a thread by default
    
    ROS_INFO("[SPIRAL_SEARCH] Waiting for the move_base action server to come online...");

    bool mb_aconline = false;
    for(int i=0 ; i<10 ; i++)
    {
        if(mb_ac.waitForServer(ros::Duration(1.0)))
        {
            mb_aconline = true;
            break;
        }
        ROS_INFO("[SPIRAL_SEARCH] Unable to find the move_base action server, retrying...");
    }

    if(!mb_aconline)
    {
        ROS_FATAL("[SPIRAL_SEARCH] No move_base node found. Please ensure the move_base node is active.");
        ROS_BREAK();
        return;
    }
    ROS_INFO("[SPIRAL_SEARCH] Found MoveBase! Initializing module...");


    // Load Parameters
    //-----------------
    nh->param<double>("minPI", minPI, 0.1);
    nh->param<double>("stop_and_measure_time", stop_and_measure_time, 1.0);
    
    nh->param<std::string>("enose_topic", enose_topic, "/PID/Sensor_reading");
    nh->param<std::string>("robot_location_topic", robot_location_topic, "/amcl_pose");
    nh->param<std::string>("map_topic", map_topic, "/map");

    nh->param<double>("max_search_time", max_search_time, 600.0);
    nh->param<double>("distance_found", distance_found, 0.5);
    nh->param<double>("source_pose_x", source_pose_x, 5.85);
    nh->param<double>("source_pose_y", source_pose_y, 7.7);
    nh->param<double>("robot_pose_x", robot_pose_x, 0.0);
    nh->param<double>("robot_pose_y", robot_pose_y, 0.0);
    nh->param<double>("initial_step", initStep, 0.6);
    nh->param<double>("step_increment", step_increment, 0.3);
    nh->param<double>("Kmu", Kmu, 0.5);
    nh->param<double>("Kp", Kp, 1);
    nh->param<double>("intervalLength", intervalLength, 0.5);
    
    nh->param<std::string>("results_file",results_file,"/home/pepe/catkin_ws/src/olfaction/gas_source_localization/results/spiral/results.txt");
    nh->param<bool>("verbose",verbose,"false");
    
    nh->param<double>("gas_present_thr", gas_present_thr, 1);
    // Subscribers
    //------------
    gas_sub_ = nh_->subscribe(enose_topic,1, &SpiralSearcher::gasCallback, this);
    map_sub_ = nh_->subscribe(map_topic, 1,  &SpiralSearcher::mapCallback, this);
    localization_sub_ = nh_->subscribe(robot_location_topic,100, &SpiralSearcher::localizationCallback, this);

    // Services
    //-----------
    mb_client = nh->serviceClient<nav_msgs::GetPlan>("/move_base/GlobalPlanner/make_plan");

    // Init State
    previous_state = SPIRAL_state::WAITING_FOR_MAP;
    current_state = SPIRAL_state::WAITING_FOR_MAP;
    ROS_INFO("[SPIRAL_SEARCH] INITIALIZATON COMPLETED--> WAITING_FOR_MAP");

    average_concentration=0;

    previousPI=0;
    currentInterval=0;

    //Init variables
    step=initStep;
    consecutive_misses=0;
    inMotion = false;
    inExecution = false;
    spiral_iter=1;
}

SpiralSearcher::~SpiralSearcher(){}

//-------------------------

    // CallBack functions

//-------------------------

void SpiralSearcher::gasCallback(const olfaction_msgs::gas_sensorPtr& msg)
{
    //Only if we are in the Stop_and_Measure
    if (this->current_state == SPIRAL_state::STOP_AND_MEASURE)
    {
        stop_and_measure_gas_v[currentInterval].push_back(msg->raw);
    }
}

void SpiralSearcher::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    if (verbose) ROS_INFO("[SPIRAL_SEARCH] - %s - Got the map of the environment!", __FUNCTION__);
    //ROS convention is to consider cell [0,0] as the lower-left corner (see http://docs.ros.org/api/nav_msgs/html/msg/MapMetaData.html)
    map_ = *msg;

    if (verbose) ROS_INFO("--------------GSL---------------");
    if (verbose) ROS_INFO("Occupancy Map dimensions:");
    if (verbose) ROS_INFO("x_min:%.2f x_max:%.2f   -   y_min:%.2f y_max:%.2f",map_.info.origin.position.x, map_.info.origin.position.x+map_.info.width*map_.info.resolution, map_.info.origin.position.y,map_.info.origin.position.y+map_.info.height*map_.info.resolution);
   if (verbose)  ROS_INFO("--------------------------------");

    //Start the fun!!
    cancelNavigation();
    resetSpiral();
    previous_state = current_state;
    current_state = SPIRAL_state::SPIRAL;
    start_time = ros::Time::now();     //start measuring time
    robot_poses_vector.clear();         //start measuring distance
    inExecution = true;
    inMotion = false;
    ROS_WARN("[SPIRAL_SEARCH] STARTING THE SEARCH --> NEW SPIRAL");
}


void SpiralSearcher::localizationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    //keep the most recent robot pose
    current_robot_pose = *msg;

    //Keep all poses for later distance estimation
    robot_poses_vector.push_back(current_robot_pose);
    robot_pose_x=current_robot_pose.pose.pose.position.x;
    robot_pose_y=current_robot_pose.pose.pose.position.y;
}

// Move Base CallBacks
void SpiralSearcher::goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_DEBUG("SPIRAL_SEARCH - %s - Target achieved!", __FUNCTION__);
        // This makes the search slower, but more robust!
        cancelNavigation();
        previous_state = current_state;
        current_state = SPIRAL_state::STOP_AND_MEASURE;
        ROS_WARN("[SPIRAL_SEARCH] New state --> STOP_AND_MEASURE");
    }
    else if(state.state_ == actionlib::SimpleClientGoalState::ABORTED)
        ROS_DEBUG("SPIRAL_SEARCH - %s - UPS! Couldn't reach the target.", __FUNCTION__);

    //Notify that the objective has been reached
    inMotion = false;
}

void SpiralSearcher::goalActiveCallback(){}
void SpiralSearcher::goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){}


//----------------------

    //Gas measurement functions

//----------------------


//Stop the robot and measure gas for X seconds (standing)
//Then switch to corresponding search state
void SpiralSearcher::getGasObservations()
{
    if( (ros::Time::now() - time_stopped).toSec() > stop_and_measure_time )
    {
        //Get averaged values of the observations taken while standing
        average_concentration = get_average_vector(stop_and_measure_gas_v);

        double PI=getPI();

        //Check thresholds and set new search-state
        if (verbose){
            ROS_INFO("[SPIRAL_SEARCH] Previous PI=%2f", previousPI);
        } 

        if (PI > previousPI)
        {
            consecutive_misses=0;
            previous_state = current_state;
            previousPI=PI;
            current_state=SPIRAL_state::SPIRAL;
            resetSpiral();
            if (verbose) ROS_WARN("[SPIRAL_SEARCH] NEW SPIRAL --> Proximity Index:%.4f",PI);
        }
        else
        {
            //Nothing
            previous_state = current_state;
            current_state=SPIRAL_state::SPIRAL;
            if (verbose) ROS_WARN("[SPIRAL_SEARCH] CONTINUE --> Proximity Index:%.4f", PI);
            if(consecutive_misses>3){
                previousPI=previousPI/2.0;
                if (verbose) ROS_WARN("[SPIRAL_SEARCH] PI reduced to:%.4f",previousPI);

                consecutive_misses=0;
            }else{
                consecutive_misses++;
            }
        }
    }
    else if( (ros::Time::now() - lastInterval).toSec() > intervalLength )
    {
        lastInterval=ros::Time::now();
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
    mb_ac.cancelAllGoals();               //Cancel current navigations
    inMotion = false;

    //Start a new measurement-phase while standing
    stop_and_measure_gas_v.clear();
    stop_and_measure_gas_v.push_back(std::vector<double>());
    time_stopped = ros::Time::now();    //Start timer for initial measurement
    lastInterval = time_stopped;
    currentInterval=0;
}

void SpiralSearcher::resetSpiral(){
    step=initStep;
    spiral_iter=-1;
}

move_base_msgs::MoveBaseGoal SpiralSearcher::nextGoalSpiral(geometry_msgs::Pose initial){

    double yaw=tf::getYaw(initial.orientation);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id="map";
    goal.target_pose.header.stamp=ros::Time::now();
    if(spiral_iter==-1){
        goal.target_pose.pose.position.x=initial.position.x+step*cos(yaw)-step*sin(yaw);
        goal.target_pose.pose.position.y=initial.position.y+step*sin(yaw)+step*cos(yaw);
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw+M_PI/4);
        spiral_iter=1;
    }else{
        if(spiral_iter%2==0){
            step+=step_increment;
        }
        goal.target_pose.pose.position.x=initial.position.x-(-step)*sin(yaw);
        goal.target_pose.pose.position.y=initial.position.y+(-step)*cos(yaw);
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw-M_PI/2);
        spiral_iter++;
    }
    
    ROS_INFO("[SPIRAL_SEARCH] New Goal [%.2f, %.2f]", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    return goal;
}

bool SpiralSearcher::doSpiral(){
    move_base_msgs::MoveBaseGoal goal = nextGoalSpiral(current_robot_pose.pose.pose);
    int i=0;
    bool blocked=false;
    while(ros::ok()&&!checkGoal(&goal)){
        if(verbose) ROS_INFO("[SPIRAL_SEARCH] SKIPPING NEXT POINT IN SPIRAL (OBSTACLES)");
        goal=nextGoalSpiral(goal.target_pose.pose);
        i++;
        if(i>3){
            //if we fail 3 consecutive times, reset the spiral
            //if it happens again, abort the spiral and go to a random nearby position
            i=0;
            if(blocked){return false;}
            ROS_INFO("[SPIRAL_SEARCH] UNABLE TO CONTINUE SPIRAL (OBSTACLES)");
            resetSpiral();
            blocked=true;
        }
    }
    inMotion=true;
    mb_ac.sendGoal(goal, boost::bind(&SpiralSearcher::goalDoneCallback, this,  _1, _2), boost::bind(&SpiralSearcher::goalActiveCallback, this), boost::bind(&SpiralSearcher::goalFeedbackCallback, this, _1));
    return true;
}

geometry_msgs::PoseStamped SpiralSearcher::get_random_pose_environment()
{
    bool valid_pose = false;
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = ros::Time::now();
    double randomPoseDistance=1;
    while (!valid_pose)
    {
        //random pose in the vecinity of the robot
        p.pose.position.x = fRand(current_robot_pose.pose.pose.position.x-randomPoseDistance, current_robot_pose.pose.pose.position.x+randomPoseDistance);
        p.pose.position.y = fRand(current_robot_pose.pose.pose.position.y-randomPoseDistance, current_robot_pose.pose.pose.position.y+randomPoseDistance);
        p.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        randomPoseDistance+=0.5;
        //check pose validity
        nav_msgs::GetPlan mb_srv;
        geometry_msgs::PoseStamped start_point;
        start_point.header.frame_id = "map";
        start_point.header.stamp = ros::Time::now();
        start_point.pose = current_robot_pose.pose.pose;
        mb_srv.request.start = start_point;
        mb_srv.request.goal = p;

        //get path from robot to candidate.
        if( mb_client.call(mb_srv) && mb_srv.response.plan.poses.size())
            valid_pose = true;
        else
            if (verbose) ROS_WARN("[SPIRAL_SEARCH] invalid random pose=[%.2f, %.2f, %.2f]", p.pose.position.x, p.pose.position.y, p.pose.orientation.z);
    }
   
    //show content
    if (verbose) ROS_INFO("[SPIRAL_SEARCH] Random Goal pose =[%.2f, %.2f, %.2f]", p.pose.position.x, p.pose.position.y, p.pose.orientation.z);
    return p;
}

//Set a random goal within the map (EXPLORATION)
void SpiralSearcher::setRandomGoal()
{
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = get_random_pose_environment();

    //Send goal to the Move_Base node for execution
    if (verbose) ROS_INFO("[SPIRAL_SEARCH] - %s - Sending robot to %lf %lf", __FUNCTION__, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    mb_ac.sendGoal(goal, boost::bind(&SpiralSearcher::goalDoneCallback, this,  _1, _2), boost::bind(&SpiralSearcher::goalActiveCallback, this), boost::bind(&SpiralSearcher::goalFeedbackCallback, this, _1));
    inMotion = true;
}

bool SpiralSearcher::checkGoal(move_base_msgs::MoveBaseGoal * goal)
{
    //ROS_INFO("[DEBUG] Checking Goal [%.2f, %.2f] in map frame", goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);

    //1. Get dimensions of OccupancyMap
    double map_min_x = map_.info.origin.position.x;
    double map_max_x = map_.info.origin.position.x + map_.info.width*map_.info.resolution;
    double map_min_y = map_.info.origin.position.y;
    double map_max_y = map_.info.origin.position.y + map_.info.height*map_.info.resolution;

    //2. Check that goal falls inside the map
    if (goal->target_pose.pose.position.x < map_min_x ||
        goal->target_pose.pose.position.x > map_max_x ||
        goal->target_pose.pose.position.y < map_min_y ||
        goal->target_pose.pose.position.y > map_max_y)
    {
        if (verbose) ROS_INFO("[DEBUG] Goal is out of map dimensions");
        return false;
    }

    //3. Use Move Base Service to declare a valid navigation goal
    nav_msgs::GetPlan mb_srv;
    geometry_msgs::PoseStamped start_point;
    start_point.header.frame_id = "map";
    start_point.header.stamp = ros::Time::now();
    start_point.pose = current_robot_pose.pose.pose;
    mb_srv.request.start = start_point;
    mb_srv.request.start.header.frame_id = "map";
    mb_srv.request.goal = goal->target_pose;
    mb_srv.request.tolerance=0;
    //get path from robot to candidate.
    bool ok =  mb_client.call(mb_srv);
    if( ok && mb_srv.response.plan.poses.size()>1)
    {
        return true;
    }
    else
    {
        if (verbose) ROS_INFO("[DEBUG] Unable to reach  [%.2f, %.2f] with MoveBase", goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);
        return false;
    }
}

//Check if the robot found the gas source.
// -1 = running,  0=failure, 1=sucess
int SpiralSearcher::checkSourceFound()
{ 
    if (inExecution)
    {
        //1. Check that working time < max allowed time for search
        ros::Duration time_spent = ros::Time::now() - start_time;
        if (time_spent.toSec() > max_search_time)
        {
            //Report failure, we were too slow
            ROS_INFO("[SPIRAL_SEARCH] - FAILURE-> Time spent (%.3f s) > max_search_time = %.3f", time_spent.toSec(), max_search_time);
            save_results_to_file(0);
            return 0;
        }

        //2. Distance from robot to source
        double Ax = current_robot_pose.pose.pose.position.x - source_pose_x;
        double Ay = current_robot_pose.pose.pose.position.y - source_pose_y;
        double dist = sqrt( pow(Ax,2) + pow(Ay,2) );
        if (dist < distance_found)
        {
            //GSL has finished with success!
            ROS_INFO("[SPIRAL_SEARCH] - SUCCESS -> Time spent (%.3f s)", time_spent.toSec());
            save_results_to_file(1);
            return 1;
        }
    }

    //In other case, we are still searching (keep going)
    return -1;
}

void SpiralSearcher::save_results_to_file(int result)
{
    mb_ac.cancelAllGoals();

    //1. Search time.
    ros::Duration time_spent = ros::Time::now() - start_time;
    double search_t = time_spent.toSec();


    // 2. Search distance
    // Get distance from array of path followed (vector of PoseWithCovarianceStamped
    double search_d;
    double Ax, Ay, d = 0;

    for (size_t h=1; h<robot_poses_vector.size(); h++)
    {
        Ax = robot_poses_vector[h-1].pose.pose.position.x - robot_poses_vector[h].pose.pose.position.x;
        Ay = robot_poses_vector[h-1].pose.pose.position.y - robot_poses_vector[h].pose.pose.position.y;
        d += sqrt( pow(Ax,2) + pow(Ay,2) );
    }
    search_d = d;



    // 3. Navigation distance (from robot to source)
    // Estimate the distances by getting a navigation path from Robot initial pose to Source points in the map
    double nav_d;
    geometry_msgs::PoseStamped source_pose;
    source_pose.header.frame_id = "map";
    source_pose.header.stamp = ros::Time::now();
    source_pose.pose.position.x = source_pose_x;
    source_pose.pose.position.y = source_pose_y;

    // Set MoveBase srv to estimate the distances
    mb_ac.cancelAllGoals();
    nav_msgs::GetPlan mb_srv;
    mb_srv.request.start.header.frame_id = "map";
    mb_srv.request.start.header.stamp  = ros::Time::now();
    mb_srv.request.start.pose = robot_poses_vector[0].pose.pose;
    mb_srv.request.goal = source_pose;

    // Get path
    if( !mb_client.call(mb_srv) )
    {
        ROS_ERROR("[GSL-PlumeTracking] Unable to GetPath from MoveBase");
        nav_d = -1;
    }
    else
    {
        // get distance [m] from vector<pose>
        double Ax, Ay, d = 0;
        for (size_t h=0; h<mb_srv.response.plan.poses.size(); h++)
        {
            if (h==0)
            {
                Ax = mb_srv.request.start.pose.position.x - mb_srv.response.plan.poses[h].pose.position.x;
                Ay = mb_srv.request.start.pose.position.y - mb_srv.response.plan.poses[h].pose.position.y;
            }
            else
            {
                Ax = mb_srv.response.plan.poses[h-1].pose.position.x - mb_srv.response.plan.poses[h].pose.position.x;
                Ay = mb_srv.response.plan.poses[h-1].pose.position.y - mb_srv.response.plan.poses[h].pose.position.y;
            }
            d += sqrt( pow(Ax,2) + pow(Ay,2) );
        }
        nav_d = d;
    }

    // 4. Nav time
    double nav_t = nav_d/0.4;   //assumming a cte speed of 0.4m/s
    std::string str = boost::str(boost::format("[PlumeTracking] RESULT IS: Success=%u, Search_d=%.3f, Nav_d=%.3f, Search_t=%.3f, Nav_t=%.3f\n") % result % search_d % nav_d % search_t % nav_t).c_str();
    ROS_INFO(str.c_str());


    //Save to file
    
    if (FILE* output_file=fopen(results_file.c_str(), "w"))
    {
        fprintf(output_file, "%s", str.c_str());
        for(geometry_msgs::PoseWithCovarianceStamped p : robot_poses_vector){
            fprintf(output_file, "%f, %f\n", p.pose.pose.position.x, p.pose.pose.position.y);
        }
        fclose(output_file);
    }
    else
        ROS_ERROR("Unable to open Results file at: %s", results_file.c_str());
}

//------------------------

    //getters

//------------------------

SPIRAL_state SpiralSearcher::getCurrentState(){
    return current_state;
}

bool SpiralSearcher::isInMotion(){
    return inMotion;
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
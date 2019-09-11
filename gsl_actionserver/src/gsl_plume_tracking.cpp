#include <gsl_plume_tracking.h>

PlumeTracking::PlumeTracking(ros::NodeHandle *nh) :
    nh_(nh),
    mb_ac("move_base", true)
{
    srand(time(NULL));      //initialize random seed

    ROS_INFO("[PlumeTracking] Waiting for the move_base action server to come online...");
    bool mb_aconline = false;
    for(int i=0 ; i<10 ; i++)
    {
        if(mb_ac.waitForServer(ros::Duration(1.0)))
        {
            mb_aconline = true;
            break;
        }
        ROS_INFO("[PlumeTracking] Unable to find the move_base action server, retrying...");
    }

    if(!mb_aconline)
    {
        ROS_FATAL("[PlumeTracking] No move_base node found. Please ensure the move_base node is active.");
        ROS_BREAK();
        return;
    }
    ROS_INFO("[PlumeTracking] Found MoveBase! Initializing module...");


    // Load Parameters
    //-----------------
    nh->param<int>("moving_average_size", moving_average_size, 10);
    nh->param<double>("th_gas_present", th_gas_present, 0.3);
    nh->param<double>("th_wind_present", th_wind_present, 0.0);
    nh->param<double>("stop_and_measure_time", stop_and_measure_time, 2);
    nh->param<double>("inspection_radius", inspection_radius, 1.0);
    nh->param<double>("step", step, 1.0);
    nh->param<double>("timeout_cast", timeout_cast, 60.0);

    nh->param<std::string>("enose_topic", enose_topic, "/PID/Sensor_reading");
    nh->param<std::string>("anemometer_topic", anemometer_topic, "/Anemometer01/WindSensor_reading");
    nh->param<std::string>("robot_location_topic", robot_location_topic, "/amcl_pose");
    nh->param<std::string>("map_topic", map_topic, "/map");

    nh->param<double>("max_search_time", max_search_time, 600.0);
    nh->param<double>("distance_found", distance_found, 0.5);
    nh->param<double>("source_pose_x", source_pose_x, 0.0);
    nh->param<double>("source_pose_y", source_pose_y, 0.0);
    nh->param<double>("robot_pose_x", robot_pose_x, 0.0);
    nh->param<double>("robot_pose_y", robot_pose_y, 0.0);
    nh->param<bool>("verbose",verbose,"false");


    // Subscribers
    //------------
    gas_sub_ = nh->subscribe(enose_topic,1,&PlumeTracking::gasCallback, this);
    wind_sub_ = nh->subscribe(anemometer_topic,1,&PlumeTracking::windCallback, this);
    map_sub_ = nh->subscribe(map_topic, 1, &PlumeTracking::mapCallback, this);
    localization_sub_ = nh_->subscribe(robot_location_topic,100,&PlumeTracking::localizationCallback,this);

    //Publishers
    //------------
    wind_measurements_pub = nh_->advertise<visualization_msgs::MarkerArray>("GSL_windMeasurement", 10);

    // Services
    //-----------
    mb_client = nh_->serviceClient<nav_msgs::GetPlan>("/move_base/GlobalPlanner/make_plan");
    


    //Init variables
    gasConcentration_v.resize(moving_average_size,0.0);
    windSpeed_v.resize(moving_average_size,0.0);
    windDirection_v.resize(moving_average_size,0.0);
    gas_it = gasConcentration_v.begin();
    windS_it = windSpeed_v.begin();
    windD_it = windDirection_v.begin();

    // Init Marker: arrow to display the wind direction measured on state transitions.
    wind_point.header.frame_id = "map";
    wind_point.action = visualization_msgs::Marker::ADD;
    wind_point.ns = "measured_wind";
    wind_point.type = visualization_msgs::Marker::ARROW;
    arrow_count = 0;

    // Init State
    previous_state = PT_state::WAITING_FOR_MAP;
    current_state = PT_state::WAITING_FOR_MAP;
    ROS_INFO("[PlumeTracking] INITIALIZATON COMPLETED--> WAITING_FOR_MAP");
    inMotion = false;
    inExecution = false;
    gasHit= false;
    
    recoveryTimestamp=ros::Time::now();
    movingTimestamp=ros::Time::now();
}


PlumeTracking::~PlumeTracking()
{
    ROS_INFO("[PlumeTracking] - Closing...");
}



//------------------------

    // CallBack functions

//------------------------
void PlumeTracking::gasCallback(const olfaction_msgs::gas_sensorPtr& msg)
{
    //ROS_INFO("PlumeTracking - %s - Got a new gas observation!", __FUNCTION__);

    //[always] Add obs to the vector of the last N gas concentrations
    *gas_it = msg->raw;
    gas_it++;
    if (gas_it == gasConcentration_v.end())
        gas_it = gasConcentration_v.begin();

    //Only if we are in the Stop_and_Measure
    if (this->current_state == PT_state::STOP_AND_MEASURE)
    {
        stop_and_measure_gas_v.push_back(msg->raw);
    }
}

void PlumeTracking::windCallback(const olfaction_msgs::anemometerPtr& msg)
{
    //1. Add obs to the vector of the last N wind speeds
    *windS_it = msg->wind_speed;

    //2. Add obs to the vector of the last N wind directions
    /*
      Wind direction is reported by the direction from which it originates
      That is, upwind in the anemometer reference system
      Being positive to the right, negative to the left, range [-pi,pi] (This is contrary to Pose conventions!!)
      Instead, we store the downWind direction following the ROS convention positive to the left, negative to the right (right hand rule), range [-pi,pi]
    */
    float downWind_direction = angles::normalize_angle(msg->wind_direction+ M_PI);
    //Transform from anemometer ref_system to map ref_system using TF
    geometry_msgs::PoseStamped anemometer_downWind_pose, map_downWind_pose;
    try
    {
        anemometer_downWind_pose.header.frame_id = msg->header.frame_id;
        anemometer_downWind_pose.pose.position.x = 0.0;
        anemometer_downWind_pose.pose.position.y = 0.0;
        anemometer_downWind_pose.pose.position.z = 0.0;
        anemometer_downWind_pose.pose.orientation = tf::createQuaternionMsgFromYaw(downWind_direction);

        tf_.transformPose("map", anemometer_downWind_pose, map_downWind_pose);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("SurgeCastPT - %s - Error: %s", __FUNCTION__, ex.what());
        return;
    }

    *windD_it = tf::getYaw(map_downWind_pose.pose.orientation);
    //*windD_it = msg->wind_direction;

    //Update iterators
    windS_it++;
    if (windS_it == windSpeed_v.end())
        windS_it = windSpeed_v.begin();
    windD_it++;
    if (windD_it == windDirection_v.end())
        windD_it = windDirection_v.begin();

    //Only if we are in the Stop_and_Measure
    if (this->current_state == PT_state::STOP_AND_MEASURE)
    {
        stop_and_measure_windS_v.push_back(msg->wind_speed);
        stop_and_measure_windD_v.push_back(tf::getYaw(map_downWind_pose.pose.orientation));
    }

}



void PlumeTracking::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    if (verbose) ROS_INFO("[PlumeTracking] - %s - Got the map of the environment!", __FUNCTION__);
    //ROS convention is to consider cell [0,0] as the lower-left corner (see http://docs.ros.org/api/nav_msgs/html/msg/MapMetaData.html)
    map_ = *msg;

    if (verbose) ROS_INFO("--------------GSL---------------");
    if (verbose) ROS_INFO("Occupancy Map dimensions:");
    if (verbose) ROS_INFO("x_min:%.2f x_max:%.2f   -   y_min:%.2f y_max:%.2f",map_.info.origin.position.x, map_.info.origin.position.x+map_.info.width*map_.info.resolution, map_.info.origin.position.y,map_.info.origin.position.y+map_.info.height*map_.info.resolution);
   if (verbose)  ROS_INFO("--------------------------------");

    //Start the fun!!
    cancel_navigation();
    previous_state = current_state;
    current_state = PT_state::STOP_AND_MEASURE;
    start_time = ros::Time::now();     //start measuring time
    robot_poses_vector.clear();         //start measuring distance
    inExecution = true;
    inMotion = false;
    ROS_WARN("[PlumeTracking] STARTING THE SEARCH --> STOP_AND_MEASURE");
}


void PlumeTracking::localizationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    //keep the most recent robot pose
    current_robot_pose = *msg;

    //Keep all poses for later distance estimation
    robot_poses_vector.push_back(current_robot_pose);

}


// Move Base CallBacks
void PlumeTracking::goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_DEBUG("PlumeTracking - %s - Target achieved!", __FUNCTION__);
        // This makes the search slower, but more robust!
        if (current_state != PT_state::CROSSWIND_CAST)
        {
            cancel_navigation();
            previous_state = current_state;
            current_state = PT_state::STOP_AND_MEASURE;
            ROS_WARN("[PlumeTracking] New state --> STOP_AND_MEASURE");
        }
    }
    else if(state.state_ == actionlib::SimpleClientGoalState::ABORTED)
        ROS_DEBUG("PlumeTracking - %s - UPS! Couldn't reach the target.", __FUNCTION__);

    //Notify that the objective has been reached
    inMotion = false;
}

void PlumeTracking::goalActiveCallback(){}
void PlumeTracking::goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){}




//----------------------------------------------

// Public Functions

//----------------------------------------------
PT_state PlumeTracking::get_state()
{
    return current_state;
}

bool PlumeTracking::get_inMotion()
{
    return inMotion;
}

float PlumeTracking::get_gas_concentration()
{
    return get_average_vector(gasConcentration_v);
}

//Stop the robot
void PlumeTracking::cancel_navigation()
{
    mb_ac.cancelAllGoals();               //Cancel current navigations
    inMotion = false;
    current_step = step;                //set max step allowed for surge/cast targets
    cast_movement = 0;                  //for crosswind movement

    //Start a new measurement-phase while standing
    stop_and_measure_gas_v.clear();
    stop_and_measure_windS_v.clear();
    stop_and_measure_windD_v.clear();
    time_stopped = ros::Time::now();    //Start timer for initial wind measurement
}


//Stop the robot and measure gas and wind for X seconds (standing)
//Then switch to corresponding search state
void PlumeTracking::getGasWindObservations()
{

    if( (ros::Time::now() - time_stopped).toSec() > stop_and_measure_time )
    {
        movingTimestamp=ros::Time::now();
        movingPose=current_robot_pose;
        recoveryTimestamp=ros::Time::now();

        //Get averaged values of the observations taken while standing
        //Wind direction is reported as DownWind in the map frame_id
        //Being positive to the right, negative to the left, range [-pi,pi]
        average_concentration = get_average_vector(stop_and_measure_gas_v);
        average_wind_direction = get_average_wind_direction(stop_and_measure_windD_v);
        average_wind_spped = get_average_vector(stop_and_measure_windS_v);

        //Add wind marker for Rviz
        if (average_wind_spped >= th_wind_present)
        {
            wind_point.header.stamp = ros::Time::now();
            wind_point.header.frame_id = "map";
            wind_point.points.clear();
            wind_point.id = arrow_count;  //unique identifier for each arrow
            arrow_count++;
            wind_point.pose.position.x = current_robot_pose.pose.pose.position.x;
            wind_point.pose.position.y = current_robot_pose.pose.pose.position.y;
            wind_point.pose.position.z = current_robot_pose.pose.pose.position.z;
            wind_point.pose.orientation = tf::createQuaternionMsgFromYaw(average_wind_direction);
            wind_point.scale.x = 0.5;      //arrow lenght
            wind_point.scale.y = 0.1;      //arrow width
            wind_point.scale.z = 0.1;      //arrow height
            // color
            wind_point.color.r = 0.0;
            wind_point.color.g = 1.0;
            wind_point.color.b = 1.0;
            wind_point.color.a = 1.0;
            //Add new arrow to existing ones
            wind_pointcloud.markers.clear();
            wind_pointcloud.markers.push_back(wind_point);
            wind_measurements_pub.publish(wind_pointcloud);
        }


        //Check thresholds and set new search-state
        if (verbose) ROS_INFO("[GSL-PlumeTracking]   avg_gas=%.3f    avg_wind_speed=%.3f     avg_wind_dir=%.3f",
                 average_concentration,
                 average_wind_spped,
                 average_wind_direction);

        if (average_concentration > th_gas_present && average_wind_spped > th_wind_present)
        {
            //Gas & wind
            previous_state = current_state;
            current_state = PT_state::UPWIND_SURGE;
            if (verbose) ROS_WARN("[PlumeTracking] New state --> UPWIND_SURGE");
        }
        else if (average_concentration > th_gas_present)
        {
            //Only gas
            previous_state = current_state;
            current_state = PT_state::INSPECTION;
            inspection_iter = 0;
            if (verbose) ROS_WARN("[PlumeTracking] New state --> INSPECTION");
        }
        else if (average_wind_spped > th_wind_present)
        {
            //Only Wind
            if (previous_state == PT_state::UPWIND_SURGE)
            {
                previous_state = current_state;
                current_state = PT_state::CROSSWIND_CAST;
                if (verbose) ROS_WARN("[PlumeTracking] New state --> CROSSWIND_CAST");
            }
            else
            {
                if(gasHit){
                    previous_state = current_state;
                    current_state = PT_state::CROSSWIND_CAST;
                    if (verbose) ROS_WARN("[PlumeTracking] New state --> CROSSWIND_CAST");
                    gasHit=false;
                }
                else{
                    previous_state = current_state;
                    current_state = PT_state::EXPLORATION;
                    if (verbose) ROS_WARN("[PlumeTracking] New state --> EXPLORATION");
                }          
            }
        }
        else
        {
            //Nothing
            previous_state = current_state;
            current_state = PT_state::EXPLORATION;
            if (verbose) ROS_WARN("[PlumeTracking] New state --> EXPLORATION - no gas or wind found");
        }
    }
}



//Set a target within the map to search gas clues
void PlumeTracking::setExplorationGoal()
{
    setRandomGoal();
}


//Set up to 4 different goals to inspect area close to current location
void PlumeTracking::setInspectionGoal()
{
    if (inspection_iter < 4)
    {
        move_base_msgs::MoveBaseGoal goal;
        current_step = inspection_radius;   //meters
        do
        {
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            switch (inspection_iter)
            {
            case 0: //(x+1,y)
                goal.target_pose.pose.position.x = current_robot_pose.pose.pose.position.x + current_step;
                goal.target_pose.pose.position.y = current_robot_pose.pose.pose.position.y;
                goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(3*3.14159/4);
                break;
            case 1: //(x,y+1)
                goal.target_pose.pose.position.x = current_robot_pose.pose.pose.position.x - current_step;
                goal.target_pose.pose.position.y = current_robot_pose.pose.pose.position.y + current_step;
                goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-3*3.14159/4);
                break;
            case 2: //(x-1,y)
                goal.target_pose.pose.position.x = current_robot_pose.pose.pose.position.x - current_step;
                goal.target_pose.pose.position.y = current_robot_pose.pose.pose.position.y - current_step;
                goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-3.14159/4);
                break;
            case 3: //(x,y-1)
                goal.target_pose.pose.position.x = current_robot_pose.pose.pose.position.x + current_step;
                goal.target_pose.pose.position.y = current_robot_pose.pose.pose.position.y - current_step;
                goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(3.14159/4);
                break;
            default:
                ROS_WARN("ERROR IN SEARCH-STATE INSPECTION");
            }

            if(!nh_->ok())
            {
                ROS_ERROR("PlumeTracking - %s - Exiting...", __FUNCTION__);
                return;
            }

            //reduce step (in case goal is an obstacle)
            current_step = current_step-0.1;
        }
        while(!checkGoal(&goal));

        //Send goal to the Move_Base node for execution
        if (verbose) ROS_DEBUG("INSPECTION - Sending robot to %lf %lf", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
        mb_ac.sendGoal(goal, boost::bind(&PlumeTracking::goalDoneCallback, this,  _1, _2), boost::bind(&PlumeTracking::goalActiveCallback, this), boost::bind(&PlumeTracking::goalFeedbackCallback, this, _1));
        inMotion = true;
        inspection_iter++;
    }
    else
    {
        //End of inspection, return to Stop&Measure
        cancel_navigation();
        previous_state = current_state;
        current_state = PT_state::STOP_AND_MEASURE;
        if (verbose) ROS_WARN("[PlumeTracking] New state --> STOP_AND_MEASURE");
    }
}


//Set a random goal within the map (EXPLORATION)
void PlumeTracking::setRandomGoal()
{
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = get_random_pose_environment();

    //Send goal to the Move_Base node for execution
    if (verbose) ROS_INFO("[PlumeTracking] - %s - Sending robot to %lf %lf", __FUNCTION__, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    mb_ac.sendGoal(goal, boost::bind(&PlumeTracking::goalDoneCallback, this,  _1, _2), boost::bind(&PlumeTracking::goalActiveCallback, this), boost::bind(&PlumeTracking::goalFeedbackCallback, this, _1));
    inMotion = true;
}

geometry_msgs::PoseStamped PlumeTracking::get_random_pose_environment()
{
    int idx=0;
    move_base_msgs::MoveBaseGoal goal;
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = ros::Time::now();
    double randomPoseDistance=1;
    do
    {
        p.pose.position.x = fRand(current_robot_pose.pose.pose.position.x-randomPoseDistance, current_robot_pose.pose.pose.position.x+randomPoseDistance);
        p.pose.position.y = fRand(current_robot_pose.pose.pose.position.y-randomPoseDistance, current_robot_pose.pose.pose.position.y+randomPoseDistance);
        p.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        if(idx%5==0){
            randomPoseDistance+=0.5;
        }
        idx++;
        goal.target_pose=p;
    }while (!checkGoal(&goal));
   
    //show content
    if (verbose) ROS_INFO("[SPIRAL_SEARCH] Random Goal pose =[%.2f, %.2f, %.2f]", p.pose.position.x, p.pose.position.y, p.pose.orientation.z);
    return p;
}

float PlumeTracking::get_average_vector(std::vector<float> const &v)
{
    size_t length = v.size();
    float sum = 0.0;
    for(std::vector<float>::const_iterator i = v.begin(); i != v.end(); ++i)
        sum += *i;

    return sum/length;
}


float PlumeTracking::get_average_wind_direction(std::vector<float> const &v)
{
    //Average of wind direction, avoiding the problems of +/- pi angles.
    float x =0.0, y = 0.0;
    for(std::vector<float>::const_iterator i = v.begin(); i != v.end(); ++i)
    {
        x += cos(*i);
        y += sin(*i);
    }
    float average_angle = atan2(y, x);   


    return average_angle;
}


bool PlumeTracking::checkGoal(move_base_msgs::MoveBaseGoal * goal)
{
    //ROS_INFO("[DEBUG] Checking Goal [%.2f, %.2f] in map frame", goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);

    //1. Get dimensions of OccupancyMap
    float map_min_x = map_.info.origin.position.x;
    float map_max_x = map_.info.origin.position.x + map_.info.width*map_.info.resolution;
    float map_min_y = map_.info.origin.position.y;
    float map_max_y = map_.info.origin.position.y + map_.info.height*map_.info.resolution;

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
    mb_srv.request.tolerance=0.0;
    mb_srv.request.start.header.frame_id = "map";
    mb_srv.request.goal = goal->target_pose;
    //get path from robot to candidate.
    if( mb_client.call(mb_srv) && mb_srv.response.plan.poses.size()>1)
    {
        return true;
    }
    else
    {
        if (verbose) ROS_INFO("[DEBUG] Unable to reach  [%.2f, %.2f] with MoveBase", goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);
        return false;
    }
}

//Get random number (double)
double PlumeTracking::fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}



//Check if the robot found the gas source.
// -1 = running,  0=failure, 1=sucess
int PlumeTracking::checkSourceFound()
{
    if (inExecution)
    {
        //1. Check that working time < max allowed time for search
        ros::Duration time_spent = ros::Time::now() - start_time;
        if (time_spent.toSec() > max_search_time)
        {
            //Report failure, we were too slow
            ROS_INFO("[PlumeTracking] - FAILURE-> Time spent (%.3f s) > max_search_time = %.3f", time_spent.toSec(), max_search_time);
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
            ROS_INFO("[PlumeTracking] - SUCCESS -> Time spent (%.3f s)", time_spent.toSec());
            save_results_to_file(1);
            return 1;
        }
    }

    //In other case, we are still searching (keep going)
    return -1;
}


void PlumeTracking::save_results_to_file(int result)
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


void PlumeTracking::restart_search()
{
    // Once a search is done... command the robot back to its initial location to start a new run
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped initial_pose;
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = ros::Time::now();
    initial_pose.pose.position.x = robot_pose_x;
    initial_pose.pose.position.y = robot_pose_y;
    initial_pose.pose.orientation = tf::createQuaternionMsgFromYaw(3.14159);
    goal.target_pose = initial_pose;

    //Send goal to the Move_Base node for execution
    ROS_DEBUG("[PlumeTracking] - %s - Sending robot to initial pose [%lf %lf]", __FUNCTION__, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    mb_ac.sendGoal(goal);
    mb_ac.waitForResult();

    // Reset all metrics
    cancel_navigation();
    previous_state = PT_state::STOP_AND_MEASURE;
    current_state = PT_state::STOP_AND_MEASURE;
    start_time = ros::Time::now();      //start measuring time
    robot_poses_vector.clear();         //start measuring distance
    inExecution = true;
    inMotion = false;
    ROS_WARN("[PlumeTracking] STARTING NEW RUN at state --> STOP_AND_MEASURE");
}
#include <algorithms/gsl_plume_tracking.h>

PlumeTracking::PlumeTracking(ros::NodeHandle *nh) :
    GSLAlgorithm(nh)
{

    // Load Parameters
    //-----------------
    nh->param<double>("th_gas_present", th_gas_present, 0.3);
    nh->param<double>("th_wind_present", th_wind_present, 0.0);
    nh->param<double>("stop_and_measure_time", stop_and_measure_time, 2);
    nh->param<double>("inspection_radius", inspection_radius, 1.0);
    nh->param<double>("step", step, 1.0);
    nh->param<double>("timeout_cast", timeout_cast, 60.0);

    // Subscribers
    //------------
    gas_sub_ = nh->subscribe(enose_topic,1,&PlumeTracking::gasCallback, this);
    wind_sub_ = nh->subscribe(anemometer_topic,1,&PlumeTracking::windCallback, this);
    map_sub_ = nh->subscribe(map_topic, 1, &PlumeTracking::mapCallback, this);



    //Init variables
    gasConcentration_v.resize(moving_average_size,0.0);
    windSpeed_v.resize(moving_average_size,0.0);
    windDirection_v.resize(moving_average_size,0.0);
    gas_it = gasConcentration_v.begin();
    windS_it = windSpeed_v.begin();
    windD_it = windDirection_v.begin();

    // Init State
    previous_state = PT_state::WAITING_FOR_MAP;
    current_state = PT_state::WAITING_FOR_MAP;
    spdlog::info("INITIALIZATON COMPLETED--> WAITING_FOR_MAP");
    gasHit= false;
    gasFound=false;

    recoveryTimestamp=ros::Time::now();
    movingTimestamp=ros::Time::now();
}


PlumeTracking::~PlumeTracking()
{
    spdlog::info("- Closing...");
}



//------------------------

    // CallBack functions

//------------------------
void PlumeTracking::gasCallback(const olfaction_msgs::gas_sensorPtr& msg)
{
    //spdlog::info("PlumeTracking - {} - Got a new gas observation!", __FUNCTION__);

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
    if(!gasFound&&msg->raw>th_gas_present){
        gasFound=true;
        if (verbose) spdlog::info("GAS HIT!");
        gasHit=true;
        cancel_navigation();                //Stop Robot
        previous_state = current_state;
        current_state = PT_state::STOP_AND_MEASURE;
        if (verbose) spdlog::warn("[SurgeCastPT] New state --> STOP_AND_MEASURE");
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

        tf_listener.transformPose("map", anemometer_downWind_pose, map_downWind_pose);
    }
    catch(tf::TransformException &ex)
    {
        spdlog::error("SurgeCastPT - {} - Error: {}", __FUNCTION__, ex.what());
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
    if (verbose) spdlog::info("- {} - Got the map of the environment!", __FUNCTION__);
    //ROS convention is to consider cell [0,0] as the lower-left corner (see http://docs.ros.org/api/nav_msgs/html/msg/MapMetaData.html)
    map_ = *msg;

    if (verbose) spdlog::info("--------------GSL---------------");
    if (verbose) spdlog::info("Occupancy Map dimensions:");
    if (verbose) spdlog::info("x_min:{:.2} x_max:{:.2}   -   y_min:{:.2} y_max:{:.2}",map_.info.origin.position.x, map_.info.origin.position.x+map_.info.width*map_.info.resolution, map_.info.origin.position.y,map_.info.origin.position.y+map_.info.height*map_.info.resolution);
    if (verbose)  spdlog::info("--------------------------------");

    //Start the fun!!
    cancel_navigation();
    previous_state = current_state;
    current_state = PT_state::STOP_AND_MEASURE;
    start_time = ros::Time::now();     //start measuring time
    robot_poses_vector.clear();         //start measuring distance
    inExecution = true;
    inMotion = false;
    spdlog::warn("STARTING THE SEARCH --> STOP_AND_MEASURE");
}


// Move Base CallBacks
void PlumeTracking::goalDoneCallback(const actionlib::SimpleClientGoalState &state, const navigation_assistant::nav_assistantResultConstPtr &result)
{
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        spdlog::debug("PlumeTracking - {} - Target achieved!", __FUNCTION__);
        // This makes the search slower, but more robust!
        if (current_state != PT_state::CROSSWIND_CAST)
        {
            cancel_navigation();
            previous_state = current_state;
            current_state = PT_state::STOP_AND_MEASURE;
            spdlog::warn("New state --> STOP_AND_MEASURE");
        }
    }
    else if(state.state_ == actionlib::SimpleClientGoalState::ABORTED)
        spdlog::debug("PlumeTracking - {} - UPS! Couldn't reach the target.", __FUNCTION__);

    //Notify that the objective has been reached
    inMotion = false;
}


//----------------------------------------------

// Public Functions

//----------------------------------------------
PT_state PlumeTracking::get_state()
{
    return current_state;
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

    if( (ros::Time::now() - time_stopped).toSec() >= stop_and_measure_time )
    {
        movingTimestamp=ros::Time::now();
        movingPose=current_robot_pose;
        recoveryTimestamp=ros::Time::now();

        //Get averaged values of the observations taken while standing
        //Wind direction is reported as DownWind in the map frame_id
        //Being positive to the right, negative to the left, range [-pi,pi]
        average_concentration = *max_element(stop_and_measure_gas_v.begin(), stop_and_measure_gas_v.end()); //get_average_vector(stop_and_measure_gas_v);
        average_wind_direction = get_average_wind_direction(stop_and_measure_windD_v);
        average_wind_spped = get_average_vector(stop_and_measure_windS_v);


        //Check thresholds and set new search-state
        if (verbose) spdlog::info("avg_gas={:.2}    avg_wind_speed={:.2}     avg_wind_dir={:.2}",
                 average_concentration,
                 average_wind_spped,
                 average_wind_direction);

        if (average_concentration > th_gas_present && average_wind_spped > th_wind_present)
        {
            //Gas & wind
            previous_state = current_state;
            current_state = PT_state::UPWIND_SURGE;
            if (verbose) spdlog::warn("New state --> UPWIND_SURGE");
        }
        else if (average_concentration > th_gas_present)
        {
            //Only gas
            previous_state = current_state;
            current_state = PT_state::INSPECTION;
            inspection_iter = 0;
            if (verbose) spdlog::warn("New state --> INSPECTION");
        }
        else if (average_wind_spped > th_wind_present)
        {
            //Only Wind
            if (previous_state == PT_state::UPWIND_SURGE)
            {
                previous_state = current_state;
                current_state = PT_state::CROSSWIND_CAST;
                if (verbose) spdlog::warn("New state --> CROSSWIND_CAST");
            }
            else
            {
                if(gasHit){
                    previous_state = current_state;
                    current_state = PT_state::CROSSWIND_CAST;
                    if (verbose) spdlog::warn("New state --> CROSSWIND_CAST");
                    gasHit=false;
                }
                else{
                    previous_state = current_state;
                    current_state = PT_state::EXPLORATION;
                    if (verbose) spdlog::warn("New state --> EXPLORATION");
                }          
            }
        }
        else
        {
            //Nothing
            previous_state = current_state;
            current_state = PT_state::EXPLORATION;
            if (verbose) spdlog::warn("New state --> EXPLORATION - no gas or wind found");
        }
    }
}



//Set a target within the map to search gas clues
void PlumeTracking::setExplorationGoal()
{   
    if(gasFound){
        setRandomGoal();
    }
}


//Set up to 4 different goals to inspect area close to current location
void PlumeTracking::setInspectionGoal()
{
    if (inspection_iter < 4)
    {
        navigation_assistant::nav_assistantGoal goal;
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
                spdlog::warn("ERROR IN SEARCH-STATE INSPECTION");
            }

            //reduce step (in case goal is an obstacle)
            current_step = current_step-0.1;
        }
        while(!checkGoal(&goal));

        //Send goal to the Move_Base node for execution
        if (verbose) spdlog::debug("INSPECTION - Sending robot to {} {}", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
        mb_ac.sendGoal(goal, std::bind(&PlumeTracking::goalDoneCallback, this,  std::placeholders::_1, std::placeholders::_2), std::bind(&PlumeTracking::goalActiveCallback, this), std::bind(&PlumeTracking::goalFeedbackCallback, this, std::placeholders::_1));
        inMotion = true;
        inspection_iter++;
    }
    else
    {
        //End of inspection, return to Stop&Measure
        cancel_navigation();
        previous_state = current_state;
        current_state = PT_state::STOP_AND_MEASURE;
        if (verbose) spdlog::warn("New state --> STOP_AND_MEASURE");
    }
}


//Set a random goal within the map (EXPLORATION)
void PlumeTracking::setRandomGoal()
{
    navigation_assistant::nav_assistantGoal goal;
    goal.target_pose = get_random_pose_environment();

    //Send goal to the Move_Base node for execution
    if (verbose) spdlog::info("- {} - Sending robot to {} {}", __FUNCTION__, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    mb_ac.sendGoal(goal, std::bind(&PlumeTracking::goalDoneCallback, this,  std::placeholders::_1, std::placeholders::_2), std::bind(&PlumeTracking::goalActiveCallback, this), std::bind(&PlumeTracking::goalFeedbackCallback, this, std::placeholders::_1));
    inMotion = true;
}

geometry_msgs::PoseStamped PlumeTracking::get_random_pose_environment()
{
    int idx=0;
    navigation_assistant::nav_assistantGoal goal;
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
    if (verbose) spdlog::info("[Plume-Tracking] Random Goal pose =[{:.2}, {:.2}, {:.2}]", p.pose.position.x, p.pose.position.y, p.pose.orientation.z);
    return p;
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

//Get random number (double)
double PlumeTracking::fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

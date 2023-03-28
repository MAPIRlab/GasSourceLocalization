#include <algorithms/gsl_algorithm.h>

GSLAlgorithm::GSLAlgorithm(ros::NodeHandle *nh) :
    nh_(nh),
    mb_ac("nav_assistant")
{
    srand(time(NULL));      //initialize random seed

    spdlog::info("[GSL_NODE] Waiting for the move_base action server to come online...");
    bool mb_aconline = false;
    for(int i=0 ; i<100 ; i++)
    {
        if(mb_ac.waitForServer(ros::Duration(1.0)))
        {
            mb_aconline = true;
            break;
        }
        spdlog::info("[GSL_NODE] Unable to find the move_base action server, retrying...");
    }

    if(!mb_aconline)
    {
        ROS_FATAL("[GSL_NODE] No move_base node found. Please ensure the move_base node is active.");
        ROS_BREAK();
        return;
    }
    spdlog::info("[GSL_NODE] Found MoveBase! Initializing module...");


    // Load Parameters
    //-----------------

    nh->param<int>("moving_average_size", moving_average_size, 10);

    nh->param<std::string>("enose_topic", enose_topic, "/PID/Sensor_reading");
    nh->param<std::string>("anemometer_topic", anemometer_topic, "/Anemometer/WindSensor_reading");
    nh->param<std::string>("robot_location_topic", robot_location_topic, "/amcl_pose");
    nh->param<std::string>("map_topic", map_topic, "/map");
    nh->param<std::string>("costmap_topic", costmap_topic, "/move_base/global_costmap/costmap");

    nh->param<double>("max_search_time", max_search_time, 300.0);
    nh->param<double>("distance_found", distance_found, 0.5);
    nh->param<double>("ground_truth_x", source_pose_x, 0.0);
    nh->param<double>("ground_truth_y", source_pose_y, 0.0);
    nh->param<double>("robot_pose_x", robot_pose_x, 0.0);
    nh->param<double>("robot_pose_y", robot_pose_y, 0.0);
    nh->param<bool>("verbose",verbose,"false");


    // Subscribers
    //------------
    gas_sub_ = nh->subscribe(enose_topic,1,&GSLAlgorithm::gasCallback, this);
    wind_sub_ = nh->subscribe(anemometer_topic,1,&GSLAlgorithm::windCallback, this);
    map_sub_ = nh->subscribe(map_topic, 1, &GSLAlgorithm::mapCallback, this);
    costmap_sub_ = nh->subscribe(costmap_topic, 1, &GSLAlgorithm::costmapCallback, this);
    localization_sub_ = nh_->subscribe(robot_location_topic,100,&GSLAlgorithm::localizationCallback,this);

    // Services
    //-----------
    make_plan_client = nh_->serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
    // Init State
    spdlog::info("[GSL NODE] INITIALIZATON COMPLETED--> WAITING_FOR_MAP");
    inMotion = false;
    inExecution = false;

}

GSLAlgorithm::~GSLAlgorithm(){}

//-----------------------------

//CALLBACKS

//-----------------------------

void GSLAlgorithm::localizationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    //keep the most recent robot pose
    current_robot_pose = *msg;

    //Keep all poses for later distance estimation
    robot_poses_vector.push_back(current_robot_pose);

}

void GSLAlgorithm::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    costmap_ = *msg;
}

// Move Base CallBacks
void GSLAlgorithm::goalDoneCallback(const actionlib::SimpleClientGoalState &state, const navigation_assistant::nav_assistantResultConstPtr &result)
{
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        //spdlog::debug("PlumeTracking - {} - Target achieved!", __FUNCTION__);
    }
    else if(state.state_ == actionlib::SimpleClientGoalState::ABORTED)
        spdlog::debug("PlumeTracking - {} - UPS! Couldn't reach the target.", __FUNCTION__);

    //Notify that the objective has been reached
    inMotion = false;
}

void GSLAlgorithm::goalActiveCallback(){}
void GSLAlgorithm::goalFeedbackCallback(const navigation_assistant::nav_assistantFeedbackConstPtr &feedback){}


//-----------------

//AUX

//-----------------

bool GSLAlgorithm::get_inMotion()
{
    return inMotion;
}


float GSLAlgorithm::get_average_vector(std::vector<float> const &v)
{
    size_t length = v.size();
    float sum = 0.0;
    for(std::vector<float>::const_iterator i = v.begin(); i != v.end(); ++i)
        sum += *i;

    return sum/length;
}


bool GSLAlgorithm::checkGoal(navigation_assistant::nav_assistantGoal* goal)
{
    //spdlog::info("[DEBUG] Checking Goal [{:.2}, {:.2}] in map frame", goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);
    float pos_x = goal->target_pose.pose.position.x;
    float pos_y = goal->target_pose.pose.position.y;
    if(!isPointInsideMapBounds({pos_x, pos_y}))
    {
        if (verbose) spdlog::info("[DEBUG] Goal is out of map dimensions");
        return false;
    }
    
    nav_msgs::GetPlan serviceArgs;
    serviceArgs.request.start.header.frame_id = "map";
    serviceArgs.request.start.header.stamp  = ros::Time::now();
    serviceArgs.request.start.pose = current_robot_pose.pose.pose;

    serviceArgs.request.goal = goal->target_pose;
    
    if(!make_plan_client.call(serviceArgs))
    {
        spdlog::error("Unable to call make_plan!");
        return false;
    }

    if(serviceArgs.response.plan.poses.empty())
        return false;
    else
        return true;
}


int GSLAlgorithm::checkSourceFound()
{
    if (inExecution)
    {
        //1. Check that working time < max allowed time for search
        ros::Duration time_spent = ros::Time::now() - start_time;
        if (time_spent.toSec() > max_search_time)
        {
            //Report failure, we were too slow
            spdlog::info("- FAILURE-> Time spent ({} s) > max_search_time = {}", time_spent.toSec(), max_search_time);
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
            spdlog::info("- SUCCESS -> Time spent ({} s)", time_spent.toSec());
            save_results_to_file(1);
            return 1;
        }
    }

    //In other case, we are still searching (keep going)
    return -1;
}

void GSLAlgorithm::save_results_to_file(int result)
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
    if( !make_plan_client.call(mb_srv) )
    {
        spdlog::error(" Unable to GetPath from MoveBase");
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
    std::string str = fmt::format("RESULT IS: Success={}, Search_d={}, Nav_d={}, Search_t={}, Nav_t={}\n", result, search_d, nav_d, search_t, nav_t);
    spdlog::info(str.c_str());


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
        spdlog::error("Unable to open Results file at: {}", results_file.c_str());
}

bool GSLAlgorithm::isPointInsideMapBounds(const Utils::Vector2& point) const
{
    const static Utils::Vector2 mapStart = (Utils::Vector2) map_.info.origin.position;
    const static Utils::Vector2 mapEnd = mapStart + Utils::Vector2(map_.info.width, map_.info.height) * map_.info.resolution;
    
    return point.x >= mapStart.x &&  point.x < mapEnd.x
    && point.y >= mapStart.y &&  point.y < mapEnd.y;
}


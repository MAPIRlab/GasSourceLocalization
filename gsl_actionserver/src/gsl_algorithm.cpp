#include <gsl_algorithm.h>

GSLAlgorithm::GSLAlgorithm(ros::NodeHandle *nh) :
    nh_(nh),
    mb_ac("move_base", true)
{
    srand(time(NULL));      //initialize random seed

    ROS_INFO("[GSL_NODE] Waiting for the move_base action server to come online...");
    bool mb_aconline = false;
    for(int i=0 ; i<10 ; i++)
    {
        if(mb_ac.waitForServer(ros::Duration(1.0)))
        {
            mb_aconline = true;
            break;
        }
        ROS_INFO("[GSL_NODE] Unable to find the move_base action server, retrying...");
    }

    if(!mb_aconline)
    {
        ROS_FATAL("[GSL_NODE] No move_base node found. Please ensure the move_base node is active.");
        ROS_BREAK();
        return;
    }
    ROS_INFO("[GSL_NODE] Found MoveBase! Initializing module...");


    // Load Parameters
    //-----------------

    nh->param<int>("moving_average_size", moving_average_size, 10);

    nh->param<std::string>("enose_topic", enose_topic, "/PID/Sensor_reading");
    nh->param<std::string>("anemometer_topic", anemometer_topic, "/Anemometer/WindSensor_reading");
    nh->param<std::string>("robot_location_topic", robot_location_topic, "/amcl_pose");
    nh->param<std::string>("map_topic", map_topic, "/map");

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
    localization_sub_ = nh_->subscribe(robot_location_topic,100,&GSLAlgorithm::localizationCallback,this);

    // Services
    //-----------
    mb_client = nh_->serviceClient<nav_msgs::GetPlan>("/move_base/GlobalPlanner/make_plan");

    // Init State
    ROS_INFO("[GSL NODE] INITIALIZATON COMPLETED--> WAITING_FOR_MAP");
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


// Move Base CallBacks
void GSLAlgorithm::goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_DEBUG("PlumeTracking - %s - Target achieved!", __FUNCTION__);
    }
    else if(state.state_ == actionlib::SimpleClientGoalState::ABORTED)
        ROS_DEBUG("PlumeTracking - %s - UPS! Couldn't reach the target.", __FUNCTION__);

    //Notify that the objective has been reached
    inMotion = false;
}

void GSLAlgorithm::goalActiveCallback(){}
void GSLAlgorithm::goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback){}


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


bool GSLAlgorithm::checkGoal(move_base_msgs::MoveBaseGoal * goal)
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


int GSLAlgorithm::checkSourceFound()
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

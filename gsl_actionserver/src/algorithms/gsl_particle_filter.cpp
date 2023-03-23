#include "gsl_particle_filter.h"

Particle::Particle(double a, double b, double c){
    x=a; y=b, weight=c;
}

Particle::~Particle(){
}

ParticleFilter::ParticleFilter(ros::NodeHandle *nh) :
    SurgeSpiralPT(nh)
{
    nh->param<int>("numberOfParticles", numberOfParticles, 500);
    nh->param<int>("maxEstimations", maxEstimations, 20);
    nh->param<int>("numberOfWindObs", numberOfWindObs, 30);
    nh->param<std::string>("results_file",results_file,"home/pepe/catkin_ws/src/olfaction/gas_source_localization/results/PF/results.txt");
    nh->param<double>("convergenceThr", convergenceThr, 0.5);
    nh->param<double>("deltaT", deltaT, 1);
    nh->param<double>("mu", mu, 0.9);
    nh->param<double>("Sp", Sp, 0.01);
    nh->param<double>("Rconv", Rconv, 0.5);

    particle_markers = nh->advertise<visualization_msgs::Marker>("particle_markers", 10);
    estimation_markers = nh->advertise<visualization_msgs::Marker>("estimation_markers", 10);
    average_estimation_marker = nh->advertise<visualization_msgs::Marker>("average_estimation_marker", 10);

    lastWindObservation=ros::Time(0);
    windDirection_v.clear();
    windSpeed_v.clear();
    firstObserv=false;
    iterationsToConverge=0;
}

ParticleFilter::~ParticleFilter(){
    
}

//------------------------------------

    //OVERRIDEN PLUME-TRACKING LOGIC

//------------------------------------

void ParticleFilter::windCallback(const olfaction_msgs::anemometerPtr& msg){
    
   windSpeed_v.push_back(msg->wind_speed);

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
        spdlog::error("[Particle_Filter] - {} - Error: {}", __FUNCTION__, ex.what());
        return;
    }

    windDirection_v.push_back(tf::getYaw(map_downWind_pose.pose.orientation));

    //Only if we are in the Stop_and_Measure
    if (this->current_state == PT_state::STOP_AND_MEASURE)
    {
        stop_and_measure_windS_v.push_back(msg->wind_speed);
        stop_and_measure_windD_v.push_back(tf::getYaw(map_downWind_pose.pose.orientation));
    }

    if(ros::Time::now().toSec()-lastWindObservation.toSec()>=deltaT){
        //store the measurement in the list of wind history as a (x,y) vector
        double length = 1;
        if(lastWindObservation!=ros::Time(0)){
            length=(ros::Time::now().toSec()-lastWindObservation.toSec())/deltaT;
        }
        lastWindObservation=ros::Time::now();
        double speed = get_average_vector(windSpeed_v)*length;
        double angle = get_average_wind_direction(windDirection_v);
        historicWind.push_back(Utils::Vector2(speed*cos(angle),speed*sin(angle)));

        if(historicWind.size()>numberOfWindObs){
            historicWind.erase(historicWind.begin());
        }
        windSpeed_v.clear();
        windDirection_v.clear();
        
    }
}

void ParticleFilter::checkState()
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
            if (verbose) spdlog::warn("[GSL-ParticleFilter] New state --> STOP_AND_MEASURE");
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
                    return;
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
                }

                updateWeights(true);
                estimateLocation();
                if(isDegenerated()){
                    resample();
                }
            }
            else{
                updateWeights(false);
                if(isDegenerated()){
                    resample();
                }
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
            if (verbose) spdlog::warn("[GSL-ParticleFilter] New state --> STOP_AND_MEASURE");
        }   
        else{
            if(ros::Time::now().toSec()-lastUpdateTimestamp.toSec()>=deltaT){
                updateWeights(false);
                if(isDegenerated()){
                    resample();
                }
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

//-----------------------------

    //AUXILIARY FUNCTIONS

//-----------------------------

Utils::Vector2 ParticleFilter::average_vector(std::vector<Utils::Vector2> &data){
    double sumX=0;
    double sumY=0;
    for(Utils::Vector2 &vec : data){
        sumX+=vec.x;
        sumY+=vec.y;
    }
    return Utils::Vector2(sumX/data.size(),sumY/data.size());
}


bool ParticleFilter::cellIsFree(double x, double y){
    if(x<map_.info.origin.position.x||
        x>=map_.info.origin.position.x+map_.info.width*map_.info.resolution||
        y<map_.info.origin.position.y||
        y>=map_.info.origin.position.y+map_.info.height*map_.info.resolution){
            return false;
        }

    int h = (x- map_.info.origin.position.x)/map_.info.resolution;
    int v = (y- map_.info.origin.position.y)/map_.info.resolution;

    return map_.data[v*map_.info.width+h]==0;
}

Utils::Vector2 ParticleFilter::standardDeviationWind(int first){
    int n = historicWind.size();
    Utils::Vector2 average=average_vector(historicWind);
    double sumX=0, sumY=0;
    for(int i=first; i<historicWind.size();i++){
        sumX+=pow(historicWind[i].x-average.x,2);
        sumY+=pow(historicWind[i].y-average.y,2);
    }
    return Utils::Vector2(sqrt(sumX/n),sqrt(sumY/n));
    
}

visualization_msgs::Marker ParticleFilter::emptyMarker(){
    visualization_msgs::Marker points;
                        points.header.frame_id="map";
                        points.header.stamp=ros::Time::now();
                        points.ns = "particles";
                        points.id = 0;
                        points.type=visualization_msgs::Marker::POINTS;
                        points.action=visualization_msgs::Marker::ADD;

                        Eigen::Vector3d colour = valueToColor(1.0/numberOfParticles);
                        points.color.r = colour[0];
                        points.color.g = colour[1];
                        points.color.b = colour[2];
                        points.color.a = 1.0;
                        points.scale.x=0.1;
                        points.scale.y=0.1;
    return points;
}

Eigen::Vector3d ParticleFilter::valueToColor(double val){
    double r, g, b;
    double logaritmo=log10(val);
    if(logaritmo<-6.6){
        r=0;
        g=1;
        b=1-std::max<double>((logaritmo+10)/3.3,0);
    }
    else if(logaritmo<-3.3){
        r=(logaritmo+6.6)/3.3;
        g=1;
        b=0;
    }else{
        r=1;
        g=1-(logaritmo+3.3)/3.3;
        b=0;
    }
    return Eigen::Vector3d(r,g,b);
}

//-----------------------------

    //PARTICLE FILTER

//-----------------------------

void ParticleFilter::generateParticles(visualization_msgs::Marker &points){

    std::random_device rd; 
    std::mt19937 gen(rd()); 
    std::normal_distribution<double> dist(0, 1);

    std::vector<Utils::Vector2> stDev;
    std::vector<Utils::Vector2> estSourcePos;
    estSourcePos.push_back(Utils::Vector2(current_robot_pose.pose.pose.position.x,current_robot_pose.pose.pose.position.y));
    stDev.push_back(Utils::Vector2(0.1,0.1));
    for(int i = historicWind.size() ; i>=0; i--){       
            stDev.push_back(standardDeviationWind(i));
            estSourcePos.push_back(estSourcePos.back()-historicWind[i]*deltaT);
    }

    while(particles.size()<numberOfParticles){ //Keep generating until we get the desired amount of particles
        int i = rand() % (estSourcePos.size()); //choose which OS to use with uniform probability
        geometry_msgs::Point p;
            p.x=estSourcePos[i].x+dist(gen)*stDev[i].x; //generate a point in that OS according to a gaussian
            p.y=estSourcePos[i].y+dist(gen)*stDev[i].y;
            p.z=0;

        if(cellIsFree(p.x, p.y)){ //no particles outside of the map nor inside obstacles
            points.points.push_back(p);
            particles.push_back(Particle (p.x, p.y, 0));
        }
    }
    particle_markers.publish(points);
}

double ParticleFilter::probability(bool hit, Particle &particle){
    double total=1;
    Utils::Vector2 stDev=standardDeviationWind(0);

    for(int windIndex=0; windIndex< historicWind.size(); windIndex++){

        double sx=0, sy=0;
        for(int j=windIndex;j<historicWind.size();j++){
            sx+=historicWind[j].x*deltaT;
            sy+=historicWind[j].y*deltaT;
        }

        double deltaIX = current_robot_pose.pose.pose.position.x-particle.x-sx;
        double deltaIY = current_robot_pose.pose.pose.position.y-particle.y-sy;
        double t1=exp(-pow(deltaIX,2) / (2*pow(stDev.x,2)*(historicWind.size()-windIndex)*deltaT));
        double t2=exp(-pow(deltaIY,2) / (2*pow(stDev.y,2)*(historicWind.size()-windIndex)*deltaT));

        double density = 1.0/(2*M_PI*stDev.x*stDev.y*(historicWind.size()-windIndex)*deltaT)   *t1*t2;
        total=total*(1-mu*Sp*density);
    }

    if(hit){
        return (1-total);
    }else{
        return total;         
    }
}

void ParticleFilter::updateWeights(bool hit){
    double sum=0;
    for(int i = 0; i<particles.size();i++){
        double prob = probability(hit, particles[i]);
        particles[i].weight=particles[i].weight*prob;
        sum+=particles[i].weight;
    }
    for(int i = 0; i<particles.size();i++){
        particles[i].weight=particles[i].weight/sum;
    }

    std::sort(particles.begin(), particles.end(), [](Particle p1, Particle p2){return p1.weight>p2.weight;});
    
    visualization_msgs::Marker parts;
                parts.header.frame_id="map";
                parts.header.stamp=ros::Time::now();
                parts.ns = "particles";
                parts.id = 0;
                parts.type=visualization_msgs::Marker::POINTS;
                parts.action=visualization_msgs::Marker::ADD;
                parts.scale.x=0.1;
                parts.scale.y=0.1;
    for(const Particle &p : particles){
        geometry_msgs::Point point;
            point.x=p.x;
            point.y=p.y;
        Eigen::Vector3d col = valueToColor(p.weight);
        std_msgs::ColorRGBA color;
            color.a=1;
            color.r=col[0];
            color.g=col[1];
            color.b=col[2];

        parts.points.push_back(point);
        parts.colors.push_back(color);
    }
    particle_markers.publish(parts);
}

bool ParticleFilter::isDegenerated(){
    iterationsToConverge++;
    double neff=0; double sum=0;
    for(const Particle &p : particles){ 
        neff+=pow(p.weight,2);
        sum+=p.weight;
    }
    double effectiveP=1.0/neff;
    spdlog::info("Effective particles {}Total particles: {}", (int)effectiveP, particles.size());

    return effectiveP<particles.size()*0.25||std::isnan(effectiveP);
}

void ParticleFilter::resample(){
    if (verbose) spdlog::warn("[Particle Filter] Resampling"); 
    std::vector<Particle> newParticles;

    std::random_device rd; 
    std::mt19937 gen(rd()); 
    std::normal_distribution<double> dist(0.0, 0.05);
    visualization_msgs::Marker points=emptyMarker();

    for(int i = 0; i<particles.size();i++){
        if(newParticles.size()>=2*numberOfParticles){
            break;
        }
        if(particles[i].weight>(1.0/particles.size())){ //if the particle is good, use it to generate new ones
            int n = std::min(5, (int)(particles[i].weight*particles.size())); 
            int count=0;
            while(count<n){
                geometry_msgs::Point p;
                    p.x=dist(gen)+particles[i].x;
                    p.y=dist(gen)+particles[i].y;
                if(cellIsFree(p.x, p.y)){
                    points.points.push_back(p);
                    newParticles.push_back(Particle(p.x, p.y, 0));
                    count++;
                }
            }         
        }else if(rand()%3==0){ //if the particle is bad, *maybe* kill it
            geometry_msgs::Point p;
                    p.x=particles[i].x;
                    p.y=particles[i].y;
            newParticles.push_back(particles[i]);
            points.points.push_back(p);
        }
    }

    particles.clear();
    particles.swap(newParticles);
    generateParticles(points);

    double w = 1.0/particles.size();
    for(Particle &p : particles){
        p.weight=w;
    }
}

bool ParticleFilter::particlesConverge(){
    Utils::Vector2 average(0,0);

    for(const Particle &p : particles){
        average.x+=p.weight*p.x;
        average.y+=p.weight*p.y;
    }
    double var=0;
    for(const Particle &p : particles){
        var+=pow((Utils::Vector2(p.x,p.y)-average).norm(),2)*p.weight;
    }
    return true;//sqrt(var)<=Rconv;
}

void ParticleFilter::estimateLocation(){
    visualization_msgs::Marker estimation;
                estimation.header.frame_id="map";
                estimation.header.stamp=ros::Time::now();
                estimation.ns = "estimations";
                estimation.id = 1;
                estimation.type=visualization_msgs::Marker::POINTS;
                estimation.action=visualization_msgs::Marker::ADD;
                estimation.color.b = 1.0;
                estimation.color.a = 1.0;
                estimation.scale.x=0.1;
                estimation.scale.y=0.1;

    double averageX=0;
    double averageY=0;

    for(const Particle &p : particles){
        averageX+=p.weight*p.x;
        averageY+=p.weight*p.y;
    }

    if(!std::isnan(averageX)&&!std::isnan(averageY)){
        estimatedLocations.push_back(Utils::Vector2(averageX, averageY));
        allEstimations.push_back(Utils::Vector2(averageX, averageY));
        if(estimatedLocations.size()>maxEstimations){
            estimatedLocations.erase(estimatedLocations.begin());
        }
    }
    
    for(const Utils::Vector2 &est:estimatedLocations){
        geometry_msgs::Point p;
        p.x=est.x; p.y=est.y;
        estimation.points.push_back(p);
    }
    estimation_markers.publish(estimation); 
}

Utils::Vector2 ParticleFilter::sourceLocalizationEstimation(){
    return average_vector(estimatedLocations);
}

int ParticleFilter::checkSourceFound()
{
    if (inExecution)
    {
        if(estimatedLocations.size()<maxEstimations){
            return -1;
        }
        //1. Check that working time < max allowed time for search
        ros::Duration time_spent = ros::Time::now() - start_time;
        if (time_spent.toSec() > max_search_time)
        {
            //Report failure, we were too slow
            spdlog::info("- FAILURE-> Time spent ({} s) > max_search_time = {}", time_spent.toSec(), max_search_time);
            save_results_to_file(0);
            return 0;
        }

        Utils::Vector2 average=average_vector(estimatedLocations);
        visualization_msgs::Marker estimation;
                    estimation.header.frame_id="map";
                    estimation.header.stamp=ros::Time::now();
                    estimation.ns = "average";
                    estimation.id = 2;
                    estimation.type=visualization_msgs::Marker::POINTS;
                    estimation.action=visualization_msgs::Marker::ADD;
                    estimation.color.g = 1.0;
                    estimation.color.a = 1.0;
                    estimation.scale.x=0.1;
                    estimation.scale.y=0.1;
        geometry_msgs::Point p;
            p.x=average.x, p.y=average.y;

        estimation.points.push_back(p);
        average_estimation_marker.publish(estimation);

        for(const Utils::Vector2& vec : estimatedLocations){
            if((vec-average).norm()>convergenceThr){
                return -1;
            }
        }

        double dist  = sqrt(pow(p.x-source_pose_x,2)+pow(p.y-source_pose_y,2));
        if (dist<distance_found && cellIsFree(p.x, p.y))
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

void ParticleFilter::save_results_to_file(int result)
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
    std::string str = fmt::format("RESULT IS: Success=%u, Search_d={}, Nav_d={}, Search_t={}, Nav_t={}", result, search_d, nav_d, search_t, nav_t);
    spdlog::info(str);


    //Save to file
    
    if (FILE* output_file=fopen(results_file.c_str(), "w"))
    {
        fprintf(output_file, "%s", str.c_str());
        for(geometry_msgs::PoseWithCovarianceStamped p : robot_poses_vector){
            fprintf(output_file, "%f, %f\n", p.pose.pose.position.x, p.pose.pose.position.y);
        }
        fprintf(output_file,"%s", "pred\n");
        for(Utils::Vector2 p : allEstimations){
            fprintf(output_file, "%f, %f\n", p.x, p.y);
        }
        fclose(output_file);
    }
    else
        spdlog::error("Unable to open Results file at: {}", results_file.c_str());
}

#include <gsl_grid.h>


Cell::Cell(bool f, double a, double b, double c){
    free=f;
    x=a;
    y=b;
    weight=c;
    auxWeight=0;
    distance=0;
}

Cell::~Cell(){
}

GridGSL::GridGSL(ros::NodeHandle *nh) :
    GSLAlgorithm(nh)
{

    // Load Parameters
    //-----------------
    nh->param<double>("th_gas_present", th_gas_present, 0.3);
    nh->param<double>("th_wind_present", th_wind_present, 0.05);
    nh->param<double>("stop_and_measure_time", stop_and_measure_time, 3);

    // Subscribers
    //------------
    gas_sub_ = nh->subscribe(enose_topic,1,&GridGSL::gasCallback, this);
    wind_sub_ = nh->subscribe(anemometer_topic,1,&GridGSL::windCallback, this);
    map_sub_ = nh->subscribe(map_topic, 1, &GridGSL::mapCallback, this);

    probability_markers = nh->advertise<visualization_msgs::Marker>("probability_markers", 10);

    // Init State
    previous_state = Grid_state::WAITING_FOR_MAP;
    current_state = Grid_state::WAITING_FOR_MAP;
    ROS_INFO("[PlumeTracking] INITIALIZATON COMPLETED--> WAITING_FOR_MAP");
    gasHit= false;

    nh->param<double>("scale", scale, 65); //scale for dynamic map reduction

    nh->param<double>("convergence_thr", convergence_thr, 0.5); //threshold for source declaration
    nh->param<double>("stdev_hit", stdev_hit, 1.0);
    nh->param<double>("stdev_miss", stdev_miss, 2.0);

    nh->param<bool>("infoTaxis", infoTaxis, false);

    nh->param<double>("ground_truth_x", ground_truth_x, 0);
    nh->param<double>("ground_truth_y", ground_truth_y, 0);
    nh->param<std::string>("results_file", results_file, "/home/pepe/Documents/Results/grid1");
    
	ros::NodeHandle n;
    clientW = n.serviceClient<gmrf_wind_mapping::WindEstimation>("/WindEstimation");
}


GridGSL::~GridGSL()
{
    ROS_INFO("[PlumeTracking] - Closing...");
}

//------------------
    //CALLBACKS
//------------------

void GridGSL::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    if (verbose) ROS_INFO("[PlumeTracking] - %s - Got the map of the environment!", __FUNCTION__);
    //ROS convention is to consider cell [0,0] as the lower-left corner (see http://docs.ros.org/api/nav_msgs/html/msg/MapMetaData.html)
    map_ = *msg;

    if (verbose) ROS_INFO("--------------GSL---------------");
    if (verbose) ROS_INFO("Occupancy Map dimensions:");
    if (verbose) ROS_INFO("x_min:%.2f x_max:%.2f   -   y_min:%.2f y_max:%.2f",map_.info.origin.position.x, map_.info.origin.position.x+map_.info.width*map_.info.resolution, map_.info.origin.position.y,map_.info.origin.position.y+map_.info.height*map_.info.resolution);
    if (verbose) ROS_INFO("--------------------------------");

    //i is y, j is x

    std::vector<std::vector<int> > mapa(map_.info.height,std::vector<int>(map_.info.width));
    int index=0;
    for(int i=0;i<mapa.size();i++){
        for(int j=0;j<mapa[0].size();j++){
            mapa[i][j]=map_.data[index]==0?1:0;
            index++;
        }
    }
        
    cells.resize(ceil((float)map_.info.height/scale));
    for(auto &cell :cells){
        cell.resize(ceil((float)map_.info.width/scale),Cell(false,0,0,0));
    }

    numCells=0;

    int cellsI=0,cellsJ=0;
    for(int i=0;i<mapa.size();i+=scale){
        cellsJ=0;
        for(int j=0;j<mapa[0].size();j+=scale){
            bool libre=true;
            for(int row=i;row<mapa.size()&&row<i+scale;row++){
                for(int col=j;col<mapa[0].size()&&col<j+scale;col++){
                    if(mapa[row][col]!=1){
                        libre = false;
                    }
                }
            }
            Eigen::Vector2d coord =indexToCoordinates(cellsI,cellsJ);
            cells[cellsI][cellsJ].free=libre;
            cells[cellsI][cellsJ].x=coord.x();
            cells[cellsI][cellsJ].y=coord.y();
            cells[cellsI][cellsJ].weight=libre?1:0;
            cellsJ++; 

            if(libre) numCells++;
        }
        cellsI++;
    }
    normalizeWeights();
    showWeights();
    //Start the fun!!
    current_state=Grid_state::EXPLORATION;
    start_time = ros::Time::now();     //start measuring time
    robot_poses_vector.clear();         //start measuring distance
    inExecution = true;
    ROS_WARN("[GridGSL] STARTING THE SEARCH --> WAITING FOR GAS");
}

void GridGSL::gasCallback(const olfaction_msgs::gas_sensorPtr& msg)
{
    //Only if we are in the Stop_and_Measure
    if (current_state == Grid_state::STOP_AND_MEASURE)
    {
        stop_and_measure_gas_v.push_back(msg->raw);
    }
    else if(current_state == Grid_state::EXPLORATION){
        if(msg->raw>th_gas_present){
            cancel_navigation();
            ROS_WARN("FOUND GAS! ---> STOP_AND_MEASURE");
        }
    }
}

void GridGSL::windCallback(const olfaction_msgs::anemometerPtr& msg)
{
    //1. Add obs to the vector of the last N wind speeds
    float downWind_direction = angles::normalize_angle(msg->wind_direction);
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

    //*windD_it = msg->wind_direction;

    //Only if we are in the Stop_and_Measure
    if (this->current_state == Grid_state::STOP_AND_MEASURE)
    {
        stop_and_measure_windS_v.push_back(msg->wind_speed);
        stop_and_measure_windD_v.push_back(tf::getYaw(map_downWind_pose.pose.orientation));
    }

}

void GridGSL::goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_DEBUG("PlumeTracking - %s - Target achieved!", __FUNCTION__);
    }
    else
        ROS_DEBUG("PlumeTracking - %s - UPS! Couldn't reach the target.", __FUNCTION__);

    //Notify that the objective has been reached
    cancel_navigation();
}

//---------------
    //ESTIMATE
//---------------

void GridGSL::getGasWindObservations()
{

    if( (ros::Time::now() - time_stopped).toSec() >= stop_and_measure_time )
    {
        //Get averaged values of the observations taken while standing
        //Wind direction is reported as DownWind in the map frame_id
        //Being positive to the right, negative to the left, range [-pi,pi]
        average_concentration = *max_element(stop_and_measure_gas_v.begin(), stop_and_measure_gas_v.end()); // get_average_vector(stop_and_measure_gas_v);
        average_wind_direction = get_average_wind_direction(stop_and_measure_windD_v);
        average_wind_speed = get_average_vector(stop_and_measure_windS_v);

        stop_and_measure_gas_v.clear();
        stop_and_measure_windS_v.clear();
        //Check thresholds and set new search-state
        if (verbose) ROS_INFO("[GSL-PlumeTracking]   avg_gas=%.3f    avg_wind_speed=%.3f     avg_wind_dir=%.3f",
                 average_concentration,
                 average_wind_speed,
                 average_wind_direction);

        if (average_concentration > th_gas_present && average_wind_speed > th_wind_present)
        {
            //Gas & wind
            previous_robot_pose=Eigen::Vector2d(current_robot_pose.pose.pose.position.x,
                                        current_robot_pose.pose.pose.position.y); //register where you were before moving
            gasHit=true;
            estimateProbabilities(cells, true, average_wind_direction, currentPosIndex);
            if (verbose) ROS_WARN("[GRID-GSL] GAS HIT\nNew state --> MOVING");
            previous_state=current_state;
            current_state=Grid_state::MOVING;
        }
        else if (average_concentration > th_gas_present)
        {
            //Only gas
            gasHit=true;
            previous_robot_pose=Eigen::Vector2d(current_robot_pose.pose.pose.position.x,
                                        current_robot_pose.pose.pose.position.y); //register where you were before moving

            if (verbose) ROS_WARN("[GRID-GSL] GAS, BUT NO WIND\n[GRID-GSL] New state --> MOVING");
            previous_state=current_state;
            current_state=Grid_state::MOVING;
        }
        else if (average_wind_speed > th_wind_present)
        {
            if(previous_state!=Grid_state::EXPLORATION){
                //Only Wind
                gasHit=false;
                estimateProbabilities(cells, false, average_wind_direction, currentPosIndex);
                if (verbose) ROS_WARN("[GRID-GSL] NO GAS HIT\n[GRID-GSL] New state --> MOVING");
                previous_state=current_state;
                current_state=Grid_state::MOVING;
            }else{
                if (verbose) ROS_WARN("[GRID-GSL] NOTHING --> WAITING FOR GAS");
            }
            
        }
        else
        {
            //Nothing
            gasHit=false;
            estimateProbabilities(cells, false, average_wind_direction, currentPosIndex);
            if (verbose) ROS_WARN("[GRID-GSL] NOTHING\n[GRID-GSL] New state --> MOVING");
            previous_state=current_state;
            current_state=Grid_state::MOVING;
        }
        
    }
}

float GridGSL::get_average_wind_direction(std::vector<float> const &v)
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

void GridGSL::estimateProbabilities(std::vector<std::vector<Cell> >& map,
                                    bool hit,
                                    double wind_direction,
                                    Eigen::Vector2i robot_pos){
    std::unordered_set< 
        std::pair<int, int>, 
        boost::hash< std::pair<int, int> > 
    > openPropagationSet;
    std::unordered_set< 
        std::pair<int, int>, 
        boost::hash< std::pair<int, int> > 
    > activePropagationSet;
    std::unordered_set< 
        std::pair<int, int>, 
        boost::hash< std::pair<int, int> > 
    > closedPropagationSet;

    int i=robot_pos.x(), j=robot_pos.y();

    int oI=std::max(0,i-1);
    int fI=std::min((int) map.size()-1,i+1);
    int oJ=std::max(0,j-1);
    int fJ=std::min((int) map[0].size()-1,j+1);
    
    double sum=0;
    //estimate the probabilities for the immediate 8 neighbours
    Eigen::Vector2d coordR = indexToCoordinates(i,j);
    double upwind_dir=angles::normalize_angle(wind_direction+M_PI);
    double move_dir =atan2((previous_robot_pose.y()-coordR.y()),(previous_robot_pose.x()-coordR.x()))+M_PI;
    
    double maxHit=gaussian(0,stdev_hit);
    double maxMiss=gaussian(0,stdev_miss);
    for(int r=oI;r<=fI;r++){
        for(int c=oJ;c<=fJ;c++){
            if(map[r][c].free){
                if(c!=j||r!=i){
                    Eigen::Vector2d coordP = indexToCoordinates(r,c);
                    double dist;
                    double cell_vector=atan2((coordR.y()-coordP.y()),(coordR.x()-coordP.x()));

                    if(hit){
                        dist=gaussian(atan2(sin(upwind_dir-cell_vector), cos(upwind_dir-cell_vector)),stdev_hit);
                    }else{
                        dist=gaussian(atan2(sin(move_dir-cell_vector), cos(move_dir-cell_vector)), stdev_miss);
                    }
                    map[r][c].weight=dist*map[r][c].weight; 
                    activePropagationSet.insert(std::pair<int,int>(r,c));
                    map[r][c].auxWeight=dist;
                    map[r][c].distance=(r==i||c==j)?1:sqrt(2);
                    sum+=map[r][c].weight;
                }
            }
        }
    }
    
    map[i][j].weight=map[i][j].weight*gaussian((hit?0:M_PI), (hit?stdev_hit:stdev_miss));
    closedPropagationSet.insert(std::pair<int,int>(i,j));
    map[i][j].auxWeight=0;
    map[i][j].distance=0;

    //propagate these short-range estimations to the entire environment using the navigation map
    propagateProbabilities(map, openPropagationSet, closedPropagationSet, activePropagationSet);   
    
}


void GridGSL::propagateProbabilities(std::vector<std::vector<Cell> >& map,
                                    std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& openPropagationSet,
                                    std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& closedPropagationSet,
                                    std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& activePropagationSet){
    
    while(!activePropagationSet.empty()){
        while(!activePropagationSet.empty()){
            auto p = *activePropagationSet.begin();
            activePropagationSet.erase(activePropagationSet.begin());
            closedPropagationSet.insert(p);

            // this very ugly and confusing bit of code just does 4-neighbour propagation 
            int oR=std::max(0,p.first-1);
            int fR=std::min((int) map.size()-1,p.first+1);
            int oC=std::max(0,p.second-1);
            int fC=std::min((int) map[0].size()-1,p.second+1);
            
            //8-neighbour propagation
            for(int i=oR;i<=fR;i++){
                for(int j=oC;j<=fC;j++){
                    calculateWeight(map, i,j, p, openPropagationSet,closedPropagationSet,activePropagationSet);
                }
            }
            
            
        }
        
        for(auto& par : openPropagationSet){
            map[par.first][par.second].weight=map[par.first][par.second].weight*map[par.first][par.second].auxWeight;
        }
        activePropagationSet=openPropagationSet;
        openPropagationSet.clear();
    }
    
    normalizeWeights();
}


void GridGSL::calculateWeight(std::vector<std::vector<Cell> >& map, int i, int j, std::pair<int,int> p, 
                                    std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& openPropagationSet,
                                    std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& closedPropagationSet,
                                    std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& activePropagationSet){
    if(map[i][j].free&&
    closedPropagationSet.find(std::pair<int,int>(i,j))==closedPropagationSet.end()&&
    activePropagationSet.find(std::pair<int,int>(i,j))==activePropagationSet.end()){

        if(openPropagationSet.find(std::pair<int,int>(i,j))!=openPropagationSet.end()){
            //if there already was a path to this cell
            double d = map[p.first][p.second].distance +
                ((i==p.first||j==p.second)?1:sqrt(2)); //distance of this new path to the same cell
            
            if(abs(d-map[i][j].distance)<0.1){ //if the distance is the same, keep the best probability!
                map[i][j].auxWeight=std::max(map[p.first][p.second].auxWeight , map[i][j].auxWeight);

            }else if(d<map[i][j].distance){ //keep the shortest path
                map[i][j].auxWeight=map[p.first][p.second].auxWeight;
                map[i][j].distance=d;
            }                        
        }else{
            map[i][j].auxWeight=map[p.first][p.second].auxWeight;
            map[i][j].distance = map[p.first][p.second].distance +
                ((i==p.first||j==p.second)?1:sqrt(2));
            openPropagationSet.insert(std::pair<int,int>(i,j));
        }
    }
}

void GridGSL::normalizeWeights(){
    double s=0.0;
    for(int i=0;i<cells.size();i++){
        for(int j=0;j<cells[0].size();j++){
            if(cells[i][j].free)
                s+=cells[i][j].weight;
        }
    }
    for(int i=0;i<cells.size();i++){
        for(int j=0;j<cells[0].size();j++){
            if(cells[i][j].free)
                cells[i][j].weight=cells[i][j].weight/s;
        }
    }

}

//----------------
    //NAVIGATION
//----------------

void GridGSL::cancel_navigation(){
    mb_ac.cancelAllGoals();               //Cancel current navigations
    inMotion = false;

    //Start a new measurement-phase while standing
    stop_and_measure_gas_v.clear();
    stop_and_measure_windS_v.clear();
    stop_and_measure_windD_v.clear();
    time_stopped = ros::Time::now();    //Start timer for initial wind measurement
    currentPosIndex=coordinatesToIndex(current_robot_pose.pose.pose.position.x,
                                        current_robot_pose.pose.pose.position.y);
    
    previous_state = current_state;
    current_state=Grid_state::STOP_AND_MEASURE;
}

void GridGSL::updateSets(){
    int i=currentPosIndex.x(), j=currentPosIndex.y();

    closedMoveSet.insert(std::pair<int,int>(i,j));

    int oI=std::max(0,i-2);
    int fI=std::min((int) cells.size()-1,i+2);
    int oJ=std::max(0,j-2);
    int fJ=std::min((int) cells[0].size()-1,j+2);

    for(int r=oI; r<=fI; r++){
        for(int c=oJ;c<=fJ;c++){
            std::pair<int,int> p(r,c);
            if(closedMoveSet.find(p)==closedMoveSet.end()
            &&cells[r][c].free
            &&cells[r][c].distance<2){
                openMoveSet.insert(p);
            }
        }
    }
}

void GridGSL::setGoal(){
    showWeights();

    int i,j;
    if(infoTaxis){
        //Infotactic navigation
        updateSets();

        std::vector<WindVector> wind = estimateWind();

        double ent=100;
        double entAux=0;
        if(!openMoveSet.empty()){
            double max = 0;
            double maxD = 0;
            for (auto &p:wind){
                int r=p.i; int c=p.j;
                entAux=entropy(r,c, Eigen::Vector2d(p.speed,p.angle));
                if(entAux<ent){
                    ent=entAux;
                    i=r; j=c;
                }
            }
            openMoveSet.erase(std::pair<int,int>(i,j));
        }else{
            ROS_ERROR("Set of open nodes is empty! Are you certain the source is reachable?");
        }
    }
    else{
        //Graph exploration
        updateSets();

        if(!openMoveSet.empty()){
            double max = 0;
            double maxD = 0;
            for (auto &p:openMoveSet){
                if(cells[p.first][p.second].weight>max){
                    i=p.first; j=p.second;
                    max=cells[p.first][p.second].weight;
                    maxD=cells[p.first][p.second].distance;
                }
            }
            openMoveSet.erase(std::pair<int,int>(i,j));
        }else{
            ROS_ERROR("Set of open nodes is empty! Are you certain the source is reachable?");
        }
    }
    moveTo(i,j);
}

void GridGSL::moveTo(int i, int j){
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        Eigen::Vector2d pos = indexToCoordinates(i,j);
        Eigen::Vector2d coordR = indexToCoordinates(currentPosIndex.x(),currentPosIndex.y());

        double move_angle= (atan2(pos.y()-coordR.y(),pos.x()-coordR.x()));
        goal.target_pose.pose.position.x = pos.x();
        goal.target_pose.pose.position.y = pos.y();
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(angles::normalize_angle(move_angle));

    ROS_INFO("[GRID-GSL] MOVING TO %f,%f",pos.x(),pos.y());
    
    mb_ac.sendGoal(goal, boost::bind(&GridGSL::goalDoneCallback, this,  _1, _2), boost::bind(&GridGSL::goalActiveCallback, this), boost::bind(&GridGSL::goalFeedbackCallback, this, _1));
    inMotion=true;
}

//----------------
    //INFOTAXIS
//----------------

double GridGSL::entropy(int i, int j, Eigen::Vector2d wind){
    auto cells2=cells; //temp copy of the matrix of cells that we can modify to simulate the effect of a measurement
    
    double entH = 0;
    if(wind.x()>=th_wind_present){
        estimateProbabilities(cells2, true, wind.y(), Eigen::Vector2i(i,j));
    
        for(int r =0 ; r<cells2.size();r++){
            for(int c=0; c<cells2[0].size();c++){
                double aux = cells2[r][c].weight*log(cells2[r][c].weight/cells[r][c].weight);
                entH-=isnan(aux)?0:
                                aux;
            }
        }
    }

    cells2=cells;
    estimateProbabilities(cells2, false, wind.y(), Eigen::Vector2i(i,j));
    double entM = 0;
    for(int r =0 ; r<cells2.size();r++){
        for(int c=0; c<cells2[0].size();c++){
            double aux = cells2[r][c].weight*log(cells2[r][c].weight/cells[r][c].weight);
            entM-=isnan(aux)?0:
                            aux;
        }
    }

    return cells[i][j].weight*entH+
        (1-cells[i][j].weight)*entM;
}

std::vector<WindVector> GridGSL::estimateWind(){
    //ask the gmrf_wind service for the estimated wind vector in cell i,j
    gmrf_wind_mapping::WindEstimation srv;

    std::vector<std::pair<int,int> > indices;
    for(auto& p: openMoveSet){
        Eigen::Vector2d coords = indexToCoordinates(p.first,p.second);
        srv.request.x.push_back(coords.x());
        srv.request.y.push_back(coords.y());
        indices.push_back(p);
    }

    std::vector<WindVector> result(openMoveSet.size());
    if(clientW.call(srv)){
        for(int ind=0;ind<openMoveSet.size();ind++){
            result[ind].i=indices[ind].first;
            result[ind].j=indices[ind].second;
            result[ind].speed=srv.response.u[ind];
            result[ind].angle=angles::normalize_angle(srv.response.v[ind]+M_PI);
        }
    }else{
        ROS_WARN("CANNOT READ ESTIMATED WIND VECTORS");
    }
    return result;
}

//----------------
    //AUX
//----------------

void GridGSL::showWeights(){
    visualization_msgs::Marker points=emptyMarker();
    
    double min=1,max=0;
    for(int a=0;a<cells.size();a++){
        for(int b=0;b<cells[0].size();b++){
            if(cells[a][b].weight>max){
                max=cells[a][b].weight;
            }
            if(cells[a][b].weight<min){
                min=cells[a][b].weight;
            }
        }
    }
    std::cout<<"MAX WEIGHT: "<<max<<"\n";
    for(int a=0;a<cells.size();a++){
        for(int b=0;b<cells[0].size();b++){
            if(cells[a][b].free){
                geometry_msgs::Point p;
                    p.x=cells[a][b].x;
                    p.y=cells[a][b].y;
                points.points.push_back(p);
                Eigen::Vector3d col = valueToColor(cells[a][b].weight, 0.0001, convergence_thr);

                std_msgs::ColorRGBA color;
                    color.a=1;
                    color.r=col[0];
                    color.g=col[1];
                    color.b=col[2];
                                   

                points.colors.push_back(color);
                
            }
        }
    }
    probability_markers.publish(points);
}

Eigen::Vector2i GridGSL::coordinatesToIndex(double x, double y){
    return Eigen::Vector2i((y-map_.info.origin.position.y)/(scale*map_.info.resolution),
                            (x-map_.info.origin.position.x)/(scale*map_.info.resolution));
}

Eigen::Vector2d GridGSL::indexToCoordinates(double i, double j){
    return Eigen::Vector2d(map_.info.origin.position.x+(j+0.5)*scale*map_.info.resolution,
                            map_.info.origin.position.y+(i+0.5)*scale*map_.info.resolution);
}

double GridGSL::gaussian(double distance, double sigma){
    return exp(-0.5*(
                    pow(distance,2)
                    /pow(sigma,2)
                )
            )
        /(sigma*sqrt(2*M_PI));
}

visualization_msgs::Marker GridGSL::emptyMarker(){
    visualization_msgs::Marker points;
                        points.header.frame_id="map";
                        points.header.stamp=ros::Time::now();
                        points.ns = "cells";
                        points.id = 0;
                        points.type=visualization_msgs::Marker::POINTS;
                        points.action=visualization_msgs::Marker::ADD;

                        Eigen::Vector3d colour = valueToColor(1.0/numCells, 0, 1);
                        points.color.r = colour[0];
                        points.color.g = colour[1];
                        points.color.b = colour[2];
                        points.color.a = 1.0;
                        points.scale.x=0.15;
                        points.scale.y=0.15;
    return points;
}

Eigen::Vector3d GridGSL::valueToColor(double val, double low, double high){
    double r, g, b;
    val=log10(val);
    double range=(log10(high)-log10(low))/4;
    low=log10(low);
    if(val<low+range){
        r=0;
        g=std::max<double>((val-low)/(range),0);
        b=1;
    }
    else if(val<low+2*range){
        r=0;
        g=1;
        b=1-std::max<double>((val-(low+range))/(range),0);
    }
    else if(val<low+3*range){
        r=(val-(low+2*range))/(range);
        g=1;
        b=0;
    }else{
        r=1;
        g=1-std::max<double>(0,(val-(low+3*range))/(range));
        b=0;
    }
    return Eigen::Vector3d(r,g,b);
}

Grid_state GridGSL::getState(){
    return current_state;
}

int GridGSL::checkSourceFound(){

    if(current_state!=Grid_state::WAITING_FOR_MAP){
        if(!reached){
            if(sqrt(pow(current_robot_pose.pose.pose.position.x-ground_truth_x,2)+
                    pow(current_robot_pose.pose.pose.position.y-ground_truth_y,2))
                <0.5){
                ros::Duration time_spent = ros::Time::now() - start_time;
                t1 = time_spent.toSec();
                reached=true;
            }
        }
        for(int i=0;i<cells.size();i++){
            for(int j=0;j<cells[0].size();j++){
                if(cells[i][j].weight>=convergence_thr){
                    save_results_to_file(1, i, j);
                    return 1;
                }
            }
        }
    }
    return -1;
}

void GridGSL::save_results_to_file(int result, int i, int j)
{
    mb_ac.cancelAllGoals();

    //1. Search time.
    ros::Duration time_spent = ros::Time::now() - start_time;
    double search_t = time_spent.toSec();

    std::string str = boost::str(boost::format("[GridGSL] RESULT IS: Success=%u, Search_t=%.3f \n") % result % search_t).c_str();
    ROS_INFO(str.c_str());


    Eigen::Vector2d pos = indexToCoordinates(i,j);
    double error = sqrt(pow(ground_truth_x-pos.x(),2) + pow(ground_truth_y-pos.y(),2));
    //Save to file
    std::ofstream file;
    file.open(results_file, std::ios_base::app);

    for(geometry_msgs::PoseWithCovarianceStamped p : robot_poses_vector){
       file<<p.pose.pose.position.x<<", "<<p.pose.pose.position.y<<"\n";
    }

    /* 
    if (FILE* output_file=fopen(results_file.c_str(), "w"))
    {
        fprintf(output_file, "%s", str.c_str());

        fprintf(output_file, "Result: %f %f\n", pos.x(), pos.y());
        fprintf(output_file, "Certainty: %f \n", cells[i][j].weight);
        fprintf(output_file, "Estimation error: %f m\n", sqrt( pow(ground_truth_x-pos.x(),2) + pow(ground_truth_y-pos.y(),2)));
    
        fprintf(output_file, "Nodes explored: %d/%d\n", (int) closedMoveSet.size(), numCells);

        fclose(output_file);
    }
    else
        ROS_ERROR("Unable to open Results file at: %s", results_file.c_str());
    */
}
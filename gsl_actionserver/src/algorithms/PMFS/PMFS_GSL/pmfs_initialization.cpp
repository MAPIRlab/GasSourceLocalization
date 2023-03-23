#include <PMFS.h>
namespace PMFS{

PMFS_GSL::PMFS_GSL(ros::NodeHandle *nh) :
    GSLAlgorithm(nh)
{
    //A lot of the initialization is done inside of the map callback, rather than here. That is because we need to know the map beforehand
    //see the file grid_callbacks.cpp

    // Load Parameters
    //-----------------
    nh->param<double>("th_gas_present", settings.th_gas_present, 0.3);
    nh->param<double>("th_wind_present", settings.th_wind_present, 0.05);
    nh->param<double>("stop_and_measure_time", settings.stop_and_measure_time, 3);


    // Subscribers
    //------------
    gas_sub_ = nh->subscribe(enose_topic,1,&PMFS_GSL::gasCallback, this);
    wind_sub_ = nh->subscribe(anemometer_topic,1,&PMFS_GSL::windCallback, this);
    map_sub_ = nh->subscribe(map_topic, 1, &PMFS_GSL::mapCallback, this);
    

    pubs.markers.source_probability_markers = nh->advertise<visualization_msgs::Marker>("probability_markers", 1);
    pubs.markers.hitProbabilityMarkers = nh->advertise<visualization_msgs::Marker>("hitProbabilityMarkers", 1);
    pubs.markers.confidenceMarkers = nh->advertise<visualization_msgs::Marker>("confidenceMarkers", 1);
    pubs.markers.windArrowMarkers = nh->advertise<visualization_msgs::MarkerArray>("arrowMarkers", 1);
    pubs.markers.gradientMarkers = nh->advertise<visualization_msgs::MarkerArray>("gradientMarkers", 1);

    pubs.markers.quadtreePublisher = nh->advertise<visualization_msgs::MarkerArray>("quadtree", 1);
    
    pubs.gas_type_pub = nh->advertise<std_msgs::String>("gas_type", 10);
    
    pubs.wind_repub = nh->advertise<olfaction_msgs::anemometer>("wind_repub", 10);

    pubs.markers.debug.explorationValue = nh->advertise<visualization_msgs::Marker>("explorationValue", 1);
    pubs.markers.debug.varianceHit = nh->advertise<visualization_msgs::Marker>("varianceHit", 1);
    pubs.markers.debug.movementSets = nh->advertise<visualization_msgs::Marker>("movementSets", 1);

    // Init State
    previous_state = Grid_state::WAITING_FOR_MAP;
    current_state = Grid_state::WAITING_FOR_MAP;
    spdlog::info("WAITING_FOR_MAP");

    nh->param<int>("scale", settings.scale, 65); //scale for dynamic map reduction

    nh->param<double>("hitPriorProbability", settings.hitProbability.prior, 0.1); 

    nh->param<int>("openMoveSetExpasion", settings.movement.openMoveSetExpasion, 5); //number of cells in each direction that we add to the open move set in each step
    nh->param<bool>("allowMovementRepetition", settings.movement.allowMovementRepetition, false);
    nh->param<double>("explorationProbability", settings.movement.explorationProbability, 0.1);
    nh->param<int>("initialExplorationMoves", settings.movement.initialExplorationMoves, 5);

    //hit probability
    nh->param<int>("max_updates_per_stop", settings.hitProbability.max_updates_per_stop, 3);

    nh->param<double>("kernel_sigma", settings.hitProbability.kernel_sigma, 0.5);
    nh->param<double>("kernel_stretch_constant", settings.hitProbability.kernel_stretch_constant, 1);

    nh->param<double>("confidence_measurement_weight", settings.hitProbability.confidence_measurement_weight, 0.5);
    nh->param<double>("confidence_sigma_spatial", settings.hitProbability.confidence_sigma_spatial, 0.5);


    //
    nh->param<bool>("useWindGroundTruth", settings.simulation.useWindGroundTruth, false);

    nh->param<double>("sourceDiscriminationPower", settings.simulation.sourceDiscriminationPower, 1);
    nh->param<double>("refineFraction", settings.simulation.refineFraction, 10);
    nh->param<int>("stepsSourceUpdate", settings.simulation.steps_between_source_updates, 10);


    nh->param<int>("maxRegionSize", settings.simulation.maxRegionSize, 5);
    nh->param<double>("deltaTime", settings.simulation.deltaTime, 0.2);
    nh->param<double>("noiseSTDev", settings.simulation.noiseSTDev, 0.5);
    nh->param<int>("iterationsToRecord", settings.simulation.iterationsToRecord, 200);
    nh->param<int>("maxWarmupIterations", settings.simulation.maxWarmupIterations, 500);



    nh->param<double>("ground_truth_x", ground_truth_x, 0);
    nh->param<double>("ground_truth_y", ground_truth_y, 0);

    nh->param<double>("markers_height", settings.markers_height, 0);

    nh->param<int>("localEstimationWindowSize", settings.localEstimationWindowSize, 2);

    nh->param<std::string>("results_file", results_file, "");
    nh->param<std::string>("errors_file", errors_file, "");
    nh->param<std::string>("path_file", path_file, "");

    nh->param<double>("convergence_thr", settings.declaration.threshold, 0.5); //threshold for source declaration
    nh->param<int>("convergence_steps", settings.declaration.steps, 5);


	ros::NodeHandle n;
    pubs.clientWindGMRF = n.serviceClient<gmrf_wind_mapping::WindEstimation>("/WindEstimation");
#ifdef USE_GADEN
    pubs.clientWindGroundTruth = n.serviceClient<gaden_player::WindPosition>("/wind_value");
#endif
    exploredCells=0;
    iterationsCounter=1;
    simulations.grid = this;
}


PMFS_GSL::~PMFS_GSL()
{
    spdlog::info("- Closing...");
}


void PMFS_GSL::initialize(){
    std::vector<std::vector<uint8_t> > mapa(map_.info.height,std::vector<uint8_t>(map_.info.width));
    int index=0;
    for(int i=0;i<mapa.size();i++){
        for(int j=0;j<mapa[0].size();j++){
            mapa[i][j]=map_.data[index];
            index++;
        }
    }


    cells.resize(floor((float)map_.info.height/settings.scale),
        std::vector<Cell>(floor((float)map_.info.width/settings.scale)) 
    );

    //create the cells
    {
        for(int i=0;i<cells.size();i++){
            for(int j=0;j<cells[0].size();j++){
                bool squareIsFree=true;

                for(int row=i * settings.scale; row<(i+1)*settings.scale; row++){
                    for(int col=j * settings.scale; col<(j+1)*settings.scale; col++){
                        if(mapa[row][col]!=0){
                            squareIsFree = false;
                        }
                    }
                }
                cells[i][j].free=squareIsFree;
                cells[i][j].hitProbability.logOdds= std::log( settings.hitProbability.prior/(1-settings.hitProbability.prior) );
                cells[i][j].hitProbability.auxWeight=-1; //this is used to prune the cells that are free but unreachable
            }
        }
    }

    //pruning unreachable free cells
    {
        std::vector<std::vector<uint8_t> > occupancyMap(cells.size(), std::vector<uint8_t>(cells[0].size()));
        tf::StampedTransform tfm;
        tf_listener.lookupTransform("map", "anemometer_frame", ros::Time(0), tfm);
        float anemometer_Z = tfm.getOrigin().z();
        spdlog::info("anemometer z is {}", anemometer_Z);
        numFreeCells = 0;
        hashSet openPropagationSet;
        hashSet activePropagationSet;
        hashSet closedPropagationSet;
        currentPosIndex = coordinatesToIndex(current_robot_pose.pose.pose.position.x, current_robot_pose.pose.pose.position.y);;
        Vector2Int& curr = currentPosIndex; 
        cells[curr.x][curr.y].hitProbability.auxWeight=0;
        activePropagationSet.insert(curr);
        propagateProbabilities(cells, openPropagationSet, closedPropagationSet, activePropagationSet, {1, {1,0}, 0});
        for(int i=0; i<cells.size();i++){
            for(int j=0; j<cells[0].size();j++){
                if(!cells[i][j].free || cells[i][j].hitProbability.auxWeight==-1){
                    cells[i][j].hitProbability.logOdds=DBL_MIN;
                    cells[i][j].free=false;
                }
                else{
                    numFreeCells++;
                    Utils::Vector2 coords = indexToCoordinates(i,j);
                    pubs.GMRFservice.request.x.push_back(coords.x);
                    pubs.GMRFservice.request.y.push_back(coords.y);
#ifdef USE_GADEN
                    pubs.groundTruthWindService.request.x.push_back(coords.x);
                    pubs.groundTruthWindService.request.y.push_back(coords.y);
                    pubs.groundTruthWindService.request.z.push_back(anemometer_Z);
#endif
                }

                occupancyMap[i][j] = cells[i][j].free? 1 : 0;
            }
        }

        simulations.quadtree = std::unique_ptr<NQA::NQAQuadtree>(new NQA::NQAQuadtree(occupancyMap));
        simulations.QTleaves = simulations.quadtree->fusedLeaves(settings.simulation.maxRegionSize);

        simulations.mapSegmentation.resize(cells.size(), std::vector<NQA::Node*>(cells[0].size(), nullptr));

        spdlog::info("Number of cells after fusing quadtree: {0}", simulations.QTleaves.size());
        //generate the image of indices so you can map a cell in the map to the corresponding leaf of the quatree
        for (int i =0 ; i<simulations.QTleaves.size(); i++)
        {
            NQA::Node& node = simulations.QTleaves[i];
            Utils::Vector2Int start = node.origin;
            Utils::Vector2Int end = node.origin + node.size;

            for (int r = start.x; r <end.x; r++)
            {
                for (int c = start.y; c <end.y; c++)
                {
                    GSL_ASSERT_MSG(simulations.mapSegmentation[r][c] == nullptr, "fused cells are overlapping"); 
                    simulations.mapSegmentation[r][c] = &node;
                }   
            }
        }
    }



    //precomputed visibility map
    {
        int visibility_map_range = std::max(settings.movement.openMoveSetExpasion, settings.localEstimationWindowSize);
        for(int i=0; i<cells.size();i++){
            for(int j=0; j<cells[0].size();j++){
                if(!cells[i][j].free)
                    continue;
                int oI=std::max(0,i-visibility_map_range) - i;
                int fI=std::min((int) cells.size()-1, i+visibility_map_range) -i;
                int oJ=std::max(0,j-visibility_map_range) -j;
                int fJ=std::min((int) cells[0].size()-1, j+visibility_map_range) -j;

                Vector2Int ij (i,j);
                hashSet visibleCells;
                double totalX=0;
                double totalY=0;
                for(int r=oI; r<=fI; r++){
                    for(int c=oJ; c<=fJ; c++){
                        Vector2Int thisCell(i+r, j+c);
                        if(pathFree(ij, thisCell)){
                            visibleCells.insert(thisCell);
                        }
                    }
                }
                visibilityMap.emplace(ij, visibleCells);           
            }
        }
    }
    
    

    //source probability before we get any observations
    for(int i=0; i<cells.size();i++){
        for(int j=0; j<cells[0].size();j++){
            cells[i][j].sourceProbability = 1.0/numFreeCells;
        }
    }

    
    estimatedWindVectors = std::vector<std::vector<Utils::Vector2> >(cells.size(), std::vector<Utils::Vector2>(cells[0].size()));
    estimateWind(settings.simulation.useWindGroundTruth);

    simulations.varianceOfHitProb.resize(cells.size(), std::vector<double>(cells[0].size(), 0));


    //Start the fun!!
    cancel_navigation(true);
    start_time = ros::Time::now();     //start measuring time
    robot_poses_vector.clear();         //start measuring distance
    inExecution = true;

    spdlog::warn("EXPLORATION");

}


Cell::Cell(){
    free=false;
    distanceFromRobot=0;
    sourceProbability = 0;
    hitProbability.logOdds = 0;
    hitProbability.auxWeight = 0;
    hitProbability.omega = 0;
    hitProbability.confidence = 0;
}

Cell::~Cell(){
}

}
#include <algorithms/gsl_GrGSL.h>

using namespace std::placeholders;

namespace GrGSL
{

    Cell::Cell(bool _free, double _weight)
    {
        free = _free;
        weight = _weight;
        auxWeight = 0;
        distance = 0;
    }

    Cell::~Cell()
    {
    }

    GrGSL::GrGSL(std::shared_ptr<rclcpp::Node> _node) : GSLAlgorithm(_node)
    {
    }

    void GrGSL::initialize()
    {
        GSLAlgorithm::initialize();

        probability_markers = node->create_publisher<Marker>("probability_markers", 10);
        estimation_markers = node->create_publisher<Marker>("estimation_markers", 10);

        // Init State
        previous_state = State::WAITING_FOR_MAP;
        current_state = State::WAITING_FOR_MAP;
        spdlog::info("INITIALIZATON COMPLETED--> WAITING_FOR_MAP");
        gasHit = false;

        clientWind = node->create_client<WindEstimation>("/WindEstimation");
        exploredCells = 0;
    }

    void GrGSL::declareParameters()
    {
        GSLAlgorithm::declareParameters();
        th_gas_present = getParam<double>("th_gas_present", 0.3);
        th_wind_present = getParam<double>("th_wind_present", 0.05);
        stop_and_measure_time = getParam<double>("stop_and_measure_time", 3);
        scale = getParam<int>("scale", 65); // scale for dynamic map reduction
        stdev_hit = getParam<double>("stdev_hit", 1.0);
        stdev_miss = getParam<double>("stdev_miss", 2.0);
        infoTaxis = getParam<bool>("infoTaxis", false);
        allowMovementRepetition = getParam<bool>("allowMovementRepetition", true);
        ground_truth_x = getParam<double>("ground_truth_x", 0);
        ground_truth_y = getParam<double>("ground_truth_y", 0);
        convergence_thr = getParam<double>("convergence_thr", 0.5); // threshold for source declaration
        convergence_steps = getParam<int>("convergence_steps", 5);
        markers_height = getParam<double>("markers_height", 0);
    }

    GrGSL::~GrGSL()
    {
        spdlog::info("- Closing...");
    }

    //------------------
    // CALLBACKS
    //------------------

    void GrGSL::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (current_state != State::WAITING_FOR_MAP)
        {
            return;
        }

        map_ = *msg;

        if (verbose)
        {

            spdlog::info("--------------GSL---------------");
            spdlog::info("Occupancy Map dimensions:");
            spdlog::info("x_min:{:.2f} x_max:{:.2f}   -   y_min:{:.2f} y_max:{:.2f}", map_.info.origin.position.x,
                         map_.info.origin.position.x + map_.info.width * map_.info.resolution, map_.info.origin.position.y,
                         map_.info.origin.position.y + map_.info.height * map_.info.resolution);
            spdlog::info("--------------------------------");
        }
        current_state = State::EXPLORATION;
    }

    void GrGSL::initializeMap()
    {
        // wait until we know where the robot is
        rclcpp::Rate rate(1);
        while (robot_poses_vector.size() == 0)
        {
            rate.sleep();
            rclcpp::spin_some(node);
            if (verbose)
                spdlog::info("Waiting to hear from localization topic: {}", localization_sub_->get_topic_name());
        }

        // i is y, j is x

        std::vector<std::vector<int>> mapa(map_.info.height, std::vector<int>(map_.info.width));
        int index = 0;
        for (int i = 0; i < mapa.size(); i++)
        {
            for (int j = 0; j < mapa[0].size(); j++)
            {
                mapa[i][j] = map_.data[index] == 0 ? 1 : 0;
                index++;
            }
        }

        cells.resize(ceil((float)map_.info.height / scale));
        for (auto& cell : cells)
        {
            cell.resize(ceil((float)map_.info.width / scale), Cell(false, 0));
        }

        numCells = 0;

        int cellsI = 0, cellsJ = 0;
        for (int i = 0; i < mapa.size(); i += scale)
        {
            cellsJ = 0;
            for (int j = 0; j < mapa[0].size(); j += scale)
            {
                bool libre = true;
                for (int row = i; row < mapa.size() && row < i + scale; row++)
                {
                    for (int col = j; col < mapa[0].size() && col < j + scale; col++)
                    {
                        if (mapa[row][col] != 1)
                        {
                            libre = false;
                        }
                    }
                }
                Eigen::Vector2d coord = indexToCoordinates(cellsI, cellsJ);
                cells[cellsI][cellsJ].free = libre;
                cells[cellsI][cellsJ].weight = 1;
                cells[cellsI][cellsJ].auxWeight = -1; // this is used to prune the cells that are free but unreachable
                cellsJ++;

                if (libre)
                    numCells++;
            }
            cellsI++;
        }

        // remove "free" cells that are actually unreachable
        hashSet openPropagationSet;
        hashSet activePropagationSet;
        hashSet closedPropagationSet;
        currentPosIndex = coordinatesToIndex(current_robot_pose.pose.pose.position.x, current_robot_pose.pose.pose.position.y);

        Vector2Int& curr = currentPosIndex;
        cells[curr.x][curr.y].auxWeight = 1;
        activePropagationSet.insert(curr);
        propagateProbabilities(cells, openPropagationSet, closedPropagationSet, activePropagationSet);
        for (int i = 0; i < cells.size(); i++)
        {
            for (int j = 0; j < cells[0].size(); j++)
            {
                if (cells[i][j].auxWeight == -1)
                {
                    cells[i][j].weight = 0;
                    cells[i][j].free = false;
                    numCells--;
                }
            }
        }

        normalizeWeights(cells);
        // Start the fun!!
        start_time = node->now();   // start measuring time
        robot_poses_vector.clear(); // start measuring distance
        inExecution = true;
        spdlog::warn("STARTING THE SEARCH --> WAITING FOR GAS");
    }

    void GrGSL::gasCallback(const olfaction_msgs::msg::GasSensor::SharedPtr msg)
    {
        // Only if we are in the Stop_and_Measure
        if (current_state == State::STOP_AND_MEASURE)
        {
            float raw = 0;
            // extract ppm reading from MOX sensors
            if (msg->raw_units == msg->UNITS_OHM)
            {
                double rs_r0 = msg->raw / 50000.0;
                raw = std::pow(rs_r0 / msg->calib_a, 1.0 / msg->calib_b);
            }
            else if (msg->raw_units == msg->UNITS_PPM)
            {
                raw = msg->raw;
            }
            stop_and_measure_gas_v.push_back(raw);
        }
        else if (current_state == State::EXPLORATION)
        {
            if (msg->raw > th_gas_present)
            {
                cancel_navigation();

                currentPosIndex = coordinatesToIndex(current_robot_pose.pose.pose.position.x, current_robot_pose.pose.pose.position.y);
                spdlog::warn("FOUND GAS! ---> STOP_AND_MEASURE");
            }
        }
    }

    void GrGSL::windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg)
    {
        // 1. Add obs to the vector of the last N wind speeds
        float downWind_direction = angles::normalize_angle(msg->wind_direction);
        // Transform from anemometer ref_system to map ref_system using TF
        geometry_msgs::msg::PoseStamped anemometer_downWind_pose, map_downWind_pose;
        try
        {
            anemometer_downWind_pose.header.frame_id = msg->header.frame_id;
            anemometer_downWind_pose.pose.position.x = 0.0;
            anemometer_downWind_pose.pose.position.y = 0.0;
            anemometer_downWind_pose.pose.position.z = 0.0;
            anemometer_downWind_pose.pose.orientation = Utils::createQuaternionMsgFromYaw(downWind_direction);

            map_downWind_pose = tf_buffer->transform(anemometer_downWind_pose, "map");
        }
        catch (tf2::TransformException& ex)
        {
            spdlog::error("SurgeCastPT - {} - Error: {}", __FUNCTION__, ex.what());
            return;
        }

        // Only if we are in the Stop_and_Measure
        if (this->current_state == State::STOP_AND_MEASURE)
        {
            stop_and_measure_windS_v.push_back(msg->wind_speed);
            stop_and_measure_windD_v.push_back(Utils::getYaw(map_downWind_pose.pose.orientation));
        }
    }

    void GrGSL::goalDoneCallback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result)
    {
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
            spdlog::error("PlumeTracking - OOPS! Couldn't reach the target. Navigation goal ReturnCode: {}", (int)result.code);

        inMotion = false;
        cancel_navigation();
    }

    //---------------
    // ESTIMATE
    //---------------

    void GrGSL::getGasWindObservations()
    {

        if ((node->now() - time_stopped).seconds() >= stop_and_measure_time)
        {
            // Get averaged values of the observations taken while standing
            // Wind direction is reported as DownWind in the map frame_id
            // Being positive to the right, negative to the left, range [-pi,pi]
            average_concentration = get_average_vector(stop_and_measure_gas_v); // get_average_vector(stop_and_measure_gas_v);
            average_wind_direction = get_average_wind_direction(stop_and_measure_windD_v);
            average_wind_speed = get_average_vector(stop_and_measure_windS_v);

            stop_and_measure_gas_v.clear();
            stop_and_measure_windS_v.clear();
            // Check thresholds and set new search-state
            if (verbose)
                spdlog::info("avg_gas={:.2f}    avg_wind_speed={:.2f}     avg_wind_dir={:.2f}", average_concentration, average_wind_speed,
                             average_wind_direction);

            if (average_concentration > th_gas_present && average_wind_speed > th_wind_present)
            {
                // Gas & wind
                previous_robot_pose = Eigen::Vector2d(current_robot_pose.pose.pose.position.x,
                                                      current_robot_pose.pose.pose.position.y); // register where you were before moving
                gasHit = true;
                estimateProbabilitiesfromGasAndWind(cells, true, true, average_wind_direction, currentPosIndex);
                if (verbose)
                    spdlog::warn(" GAS HIT\nNew state --> MOVING");
                previous_state = current_state;
                current_state = State::MOVING;
            }
            else if (average_concentration > th_gas_present)
            {
                // Only gas
                gasHit = true;
                estimateProbabilitiesfromGasAndWind(cells, true, false, average_wind_direction, currentPosIndex);
                previous_robot_pose = Eigen::Vector2d(current_robot_pose.pose.pose.position.x,
                                                      current_robot_pose.pose.pose.position.y); // register where you were before moving

                if (verbose)
                    spdlog::warn(" GAS, BUT NO WIND\n New state --> MOVING");
                previous_state = current_state;
                current_state = State::MOVING;
            }
            else if (average_wind_speed > th_wind_present)
            {
                if (previous_state != State::EXPLORATION)
                {
                    // Only Wind
                    gasHit = false;
                    estimateProbabilitiesfromGasAndWind(cells, false, true, average_wind_direction, currentPosIndex);
                    if (verbose)
                        spdlog::warn(" NO GAS HIT\n New state --> MOVING");
                    previous_state = current_state;
                    current_state = State::MOVING;
                }
                else
                {
                    if (verbose)
                        spdlog::warn(" NOTHING --> WAITING FOR GAS");
                }
            }
            else
            {
                // Nothing
                gasHit = false;
                estimateProbabilitiesfromGasAndWind(cells, false, true, average_wind_direction, currentPosIndex);
                if (verbose)
                    spdlog::warn(" NOTHING\n New state --> MOVING");
                previous_state = current_state;
                current_state = State::MOVING;
            }
        }
    }

    float GrGSL::get_average_wind_direction(std::vector<float> const& v)
    {
        // Average of wind direction, avoiding the problems of +/- pi angles.
        float x = 0.0, y = 0.0;
        for (std::vector<float>::const_iterator i = v.begin(); i != v.end(); ++i)
        {
            x += cos(*i);
            y += sin(*i);
        }
        float average_angle = atan2(y, x);

        return average_angle;
    }

    void GrGSL::estimateProbabilitiesfromGasAndWind(std::vector<std::vector<Cell>>& map, bool hit, bool advection, double wind_direction,
                                                    Vector2Int robot_pos)
    {
        // ADVECTION
        // this part is always done, to calculate the distance field to the current robot position
        // the actual advection-based source probabilities are discarded if the wind speed is too low for the direction to be reliable
        hashSet openPropagationSet;
        hashSet activePropagationSet;
        hashSet closedPropagationSet;

        int i = robot_pos.x, j = robot_pos.y;

        int oI = std::max(0, i - 2);
        int fI = std::min((int)map.size() - 1, i + 2);
        int oJ = std::max(0, j - 2);
        int fJ = std::min((int)map[0].size() - 1, j + 2);

        // estimate the probabilities for the immediate neighbours
        Eigen::Vector2d coordR = indexToCoordinates(i, j);
        double upwind_dir = angles::normalize_angle(wind_direction + M_PI);
        double move_dir = atan2((previous_robot_pose.y() - coordR.y()), (previous_robot_pose.x() - coordR.x())) + M_PI;

        for (int r = oI; r <= fI; r++)
        {
            for (int c = oJ; c <= fJ; c++)
            {
                if (!map[r][c].free)
                    continue;
                if (std::abs(c - j) > 1 || std::abs(r - i) > 1)
                {
                    Eigen::Vector2d coordP = indexToCoordinates(r, c);
                    double dist;
                    double cell_vector = atan2((coordR.y() - coordP.y()), (coordR.x() - coordP.x()));

                    if (hit)
                    {
                        dist = gaussian(atan2(sin(upwind_dir - cell_vector), cos(upwind_dir - cell_vector)), stdev_hit);
                    }
                    else
                    {
                        dist = gaussian(atan2(sin(move_dir - cell_vector), cos(move_dir - cell_vector)), stdev_miss);
                    }
                    map[r][c].auxWeight = dist;
                    map[r][c].distance = (r == i || c == j) ? 1 : sqrt(2);

                    if (map[r][c].free)
                        activePropagationSet.insert(Vector2Int(r, c));
                    else
                        closedPropagationSet.insert(Vector2Int(r, c));
                }
                else
                {
                    map[r][c].auxWeight = gaussian((hit ? 0 : M_PI), (hit ? stdev_hit : stdev_miss));
                    closedPropagationSet.insert(Vector2Int(i, j));
                    map[r][c].distance = 0;
                }
            }
        }

        // propagate these short-range estimations to the entire environment using the navigation map
        // also calculate the distance field
        propagateProbabilities(map, openPropagationSet, closedPropagationSet, activePropagationSet);

        // if we are going to use these probabilities, normalize them. Otherwise, to the trash with them
        if (advection)
        {
            double sum = 0;
            for (int r = 0; r < map.size(); r++)
            {
                for (int c = 0; c < map[0].size(); c++)
                {
                    if (map[r][c].free)
                    {
                        sum += map[r][c].auxWeight;
                    }
                }
            }
            for (int r = 0; r < map.size(); r++)
            {
                for (int c = 0; c < map[0].size(); c++)
                {
                    if (map[r][c].free)
                    {
                        map[r][c].auxWeight /= sum;
                    }
                }
            }
        }
        else
        {
            for (int r = 0; r < map.size(); r++)
            {
                for (int c = 0; c < map[0].size(); c++)
                {
                    if (map[r][c].free)
                    {
                        map[r][c].auxWeight = 0;
                    }
                }
            }
        }

        // DIFFUSION
        // calculate and normalize these probabilities before combining them with the advection ones

        std::vector<std::vector<double>> diffusionProb = std::vector<std::vector<double>>(map.size(), std::vector<double>(map[0].size()));
        double sum = 0;
        for (int r = 0; r < map.size(); r++)
        {
            for (int c = 0; c < map[0].size(); c++)
            {
                if (map[r][c].free)
                {
                    if (hit)
                        diffusionProb[r][c] = std::max(0.1, gaussian(map[r][c].distance, 3));
                    else
                        diffusionProb[r][c] = std::max(0.1, 1 - gaussian(map[r][c].distance, 3)); // this is no good
                    sum += diffusionProb[r][c];
                }
            }
        }

        for (int r = 0; r < map.size(); r++)
        {
            for (int c = 0; c < map[0].size(); c++)
            {
                if (map[r][c].free)
                {
                    map[r][c].auxWeight += diffusionProb[r][c] / sum;

                    map[r][c].weight *= map[r][c].auxWeight;
                    map[r][c].auxWeight = 0;
                }
            }
        }

        normalizeWeights(map);
    }

    void GrGSL::propagateProbabilities(std::vector<std::vector<Cell>>& map, hashSet& openPropagationSet, hashSet& closedPropagationSet,
                                       hashSet& activePropagationSet)
    {

        while (!activePropagationSet.empty())
        {
            while (!activePropagationSet.empty())
            {
                Vector2Int p = *activePropagationSet.begin();
                activePropagationSet.erase(activePropagationSet.begin());
                closedPropagationSet.insert(p);

                int oR = std::max(0, p.x - 1);
                int fR = std::min((int)map.size() - 1, p.x + 1);
                int oC = std::max(0, p.y - 1);
                int fC = std::min((int)map[0].size() - 1, p.y + 1);

                // 8-neighbour propagation
                for (int i = oR; i <= fR; i++)
                {
                    for (int j = oC; j <= fC; j++)
                    {
                        calculateWeight(map, i, j, p, openPropagationSet, closedPropagationSet, activePropagationSet);
                    }
                }
            }

            activePropagationSet.clear();
            for (auto& par : openPropagationSet)
            {
                if (map[par.x][par.y].free)
                    activePropagationSet.insert(par);
                else
                    closedPropagationSet.insert(par);
            }

            openPropagationSet.clear();
        }
    }

    void GrGSL::calculateWeight(std::vector<std::vector<Cell>>& map, int i, int j, Vector2Int p, hashSet& openPropagationSet,
                                hashSet& closedPropagationSet, hashSet& activePropagationSet)
    {
        if (closedPropagationSet.find(Vector2Int(i, j)) == closedPropagationSet.end() &&
            activePropagationSet.find(Vector2Int(i, j)) == activePropagationSet.end())
        {

            if (openPropagationSet.find(Vector2Int(i, j)) != openPropagationSet.end())
            {
                // if there already was a path to this cell
                double d = map[p.x][p.y].distance + ((i == p.x || j == p.y) ? 1 : sqrt(2)); // distance of this new path to the same cell

                if (std::abs(d - map[i][j].distance) < 0.1)
                { // if the distance is the same, keep the best probability!
                    map[i][j].auxWeight = std::max(map[p.x][p.y].auxWeight, map[i][j].auxWeight);
                }
                else if (d < map[i][j].distance)
                { // keep the shortest path
                    map[i][j].auxWeight = map[p.x][p.y].auxWeight;
                    map[i][j].distance = d;
                }
            }
            else
            {
                map[i][j].auxWeight = map[p.x][p.y].auxWeight;
                map[i][j].distance = map[p.x][p.y].distance + ((i == p.x || j == p.y) ? 1 : sqrt(2));
                openPropagationSet.insert(Vector2Int(i, j));
            }
        }
    }

    void GrGSL::normalizeWeights(std::vector<std::vector<Cell>>& map)
    {
        double s = 0.0;
        for (int i = 0; i < map.size(); i++)
        {
            for (int j = 0; j < map[0].size(); j++)
            {
                if (map[i][j].free)
                    s += map[i][j].weight;
            }
        }

        for (int i = 0; i < map.size(); i++)
        {
            for (int j = 0; j < map[0].size(); j++)
            {
                if (map[i][j].free)
                    map[i][j].weight = map[i][j].weight / s;
            }
        }
    }

    //----------------
    // NAVIGATION
    //----------------

    void GrGSL::cancel_navigation()
    {
        nav_client->async_cancel_all_goals(); // Cancel current navigations
        inMotion = false;

        // Start a new measurement-phase while standing
        stop_and_measure_gas_v.clear();
        stop_and_measure_windS_v.clear();
        stop_and_measure_windD_v.clear();
        time_stopped = node->now(); // Start timer for initial wind measurement

        previous_state = current_state;
        current_state = State::STOP_AND_MEASURE;
    }

    void GrGSL::updateSets()
    {
        int i = currentPosIndex.x, j = currentPosIndex.y;
        int expansionSize = 5;

        int oI = std::max(0, i - expansionSize);
        int fI = std::min((int)cells.size() - 1, i + expansionSize);
        int oJ = std::max(0, j - expansionSize);
        int fJ = std::min((int)cells[0].size() - 1, j + expansionSize);

        for (int r = oI; r <= fI; r++)
        {
            for (int c = oJ; c <= fJ; c++)
            {
                Vector2Int p(r, c);
                if (closedMoveSet.find(p) == closedMoveSet.end() && cells[r][c].free && cells[r][c].distance < 5)
                {
                    openMoveSet.insert(p);
                }
            }
        }
    }

    void GrGSL::setGoal()
    {
        updateSets();

        NavigateToPose::Goal goal;
        int i = -1, j = -1;
        if (infoTaxis)
        {
            // Infotactic navigation
            computingInfoGain = true;
            std::vector<WindVector> wind = estimateWind();
            std::mutex mtx;
            double ent = -DBL_MAX;
            double maxDist = 0;
            rclcpp::Time tstart = node->now();
            if (!wind.empty())
            {
#pragma omp parallel for
                for (int index = 0; index < wind.size(); index++)
                {
                    int r = wind[index].i;
                    int c = wind[index].j;
                    NavigateToPose::Goal tempGoal = indexToGoal(r, c);

                    mtx.lock();
                    bool pathExists = checkGoal(tempGoal);
                    mtx.unlock();
                    if (pathExists)
                    {
                        double entAux = informationGain(wind[index]);
                        mtx.lock();
                        if (entAux > ent || (ent == entAux && cells[r][c].distance > maxDist))
                        {

                            ent = entAux;
                            i = r;
                            j = c;
                            maxDist = cells[r][c].distance;
                            goal = tempGoal;
                        }
                        mtx.unlock();
                    }
                }
            }
            else
            {
                spdlog::error("Set of open nodes is empty! Are you certain the source is reachable?");
            }
            computingInfoGain = false;
            double ti = (node->now() - tstart).seconds();
            spdlog::info("Time infotaxis: {}", ti);
            spdlog::info("Number of considered cells: {}", wind.size());
        }
        else
        {
            // Graph exploration

            if (!openMoveSet.empty())
            {
                double max = 0;
                double maxDist = 0;
                for (auto& p : openMoveSet)
                {
                    if (probability(p) > max || (probability(p) == max && cells[p.x][p.y].distance > maxDist))
                    {

                        NavigateToPose::Goal tempGoal = indexToGoal(p.x, p.y);
                        if (checkGoal(tempGoal))
                        {
                            i = p.x;
                            j = p.y;
                            max = probability(p);
                            maxDist = cells[p.x][p.y].distance;
                            goal = tempGoal;
                        }
                    }
                }
            }
            else
            {
                spdlog::error("Set of open nodes is empty! Are you certain the source is reachable?");
            }
        }

        exploredCells++;
        closedMoveSet.insert(Vector2Int(i, j));
        openMoveSet.erase(Vector2Int(i, j));
        moveTo(goal);
    }

    void GrGSL::moveTo(NavigateToPose::Goal goal)
    {

        inMotion = true;

        spdlog::info("MOVING TO {:.2f},{:.2f}", goal.pose.pose.position.x, goal.pose.pose.position.y);

        sendGoal(goal);

        currentPosIndex = coordinatesToIndex(goal.pose.pose.position.x, goal.pose.pose.position.y);
        // close nearby cells to avoid repeating the same pose with only minor variations
        if (!allowMovementRepetition)
        {
            int i = currentPosIndex.x, j = currentPosIndex.y;
            int oI = std::max(0, i - 1);
            int fI = std::min((int)cells.size() - 1, i + 1);
            int oJ = std::max(0, j - 1);
            int fJ = std::min((int)cells[0].size() - 1, j + 1);

            for (int r = oI; r <= fI; r++)
            {
                for (int c = oJ; c <= fJ; c++)
                {
                    Vector2Int p(r, c);
                    if (cells[r][c].free)
                    {
                        openMoveSet.erase(p);
                        closedMoveSet.insert(p);
                    }
                }
            }
        }
    }

    NavigateToPose::Goal GrGSL::indexToGoal(int i, int j)
    {
        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = node->now();

        Eigen::Vector2d pos = indexToCoordinates(i, j);
        Eigen::Vector2d coordR = indexToCoordinates(currentPosIndex.x, currentPosIndex.y);

        double move_angle = (atan2(pos.y() - coordR.y(), pos.x() - coordR.x()));
        goal.pose.pose.position.x = pos.x();
        goal.pose.pose.position.y = pos.y();
        goal.pose.pose.orientation = Utils::createQuaternionMsgFromYaw(angles::normalize_angle(move_angle));
#ifdef USE_NAV_ASSISTANT
        goal.turn_before_nav = true;
#endif
        return goal;
    }

    //----------------
    // INFOTAXIS
    //----------------

    double GrGSL::informationGain(WindVector windVec)
    {
        auto predictionCells = cells; // temp copy of the matrix of cells that we can modify to simulate the effect of a measurement

        double entH = 0;
        estimateProbabilitiesfromGasAndWind(predictionCells, true, true, windVec.angle, Vector2Int(windVec.i, windVec.j));
        entH = KLD(predictionCells, cells);

        predictionCells = cells;
        estimateProbabilitiesfromGasAndWind(predictionCells, false, true, windVec.angle, Vector2Int(windVec.i, windVec.j));
        double entM = 0;
        entM = KLD(predictionCells, cells);

        return cells[windVec.i][windVec.j].weight * entH + (1 - cells[windVec.i][windVec.j].weight) * entM;
    }

    // Kullback-Leibler Divergence
    double GrGSL::KLD(std::vector<std::vector<Cell>>& a, std::vector<std::vector<Cell>>& b)
    {
        double total = 0;
        for (int r = 0; r < a.size(); r++)
        {
            for (int c = 0; c < a[0].size(); c++)
            {
                double aux =
                    a[r][c].weight * log(a[r][c].weight / b[r][c].weight) + (1 - a[r][c].weight) * log((1 - a[r][c].weight) / (1 - b[r][c].weight));
                total += std::isnan(aux) ? 0 : aux;
            }
        }
        return total;
    }

    std::vector<WindVector> GrGSL::estimateWind()
    {
        // ask the gmrf_wind service for the estimated wind vector in cell i,j
        auto request = std::make_shared<WindEstimation::Request>();

        std::vector<Vector2Int> indices;
        for (auto& p : openMoveSet)
        {
            if (cells[p.x][p.y].distance < 10)
            {
                Eigen::Vector2d coords = indexToCoordinates(p.x, p.y);
                request->x.push_back(coords.x());
                request->y.push_back(coords.y());
                indices.push_back(p);
            }
        }

        std::vector<WindVector> result(indices.size());

        auto future = clientWind->async_send_request(request);
        auto future_result = rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(1));
        if (future_result == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            for (int ind = 0; ind < indices.size(); ind++)
            {
                result[ind].i = indices[ind].x;
                result[ind].j = indices[ind].y;
                result[ind].speed = response->u[ind];
                result[ind].angle = angles::normalize_angle(response->v[ind] + M_PI);
            }
        }
        else
        {
            spdlog::warn("CANNOT READ ESTIMATED WIND VECTORS");
        }
        return result;
    }

    //----------------
    // AUX
    //----------------
    double GrGSL::probability(const Vector2Int& indices)
    {
        return cells[indices.x][indices.y].weight;
    }

    void GrGSL::showWeights()
    {
        Marker points = emptyMarker();

        for (int a = 0; a < cells.size(); a++)
        {
            for (int b = 0; b < cells[0].size(); b++)
            {
                if (cells[a][b].free)
                {
                    auto coords = indexToCoordinates(a, b);
                    Point p;
                    p.x = coords.x();
                    p.y = coords.y();
                    p.z = 0;

                    std_msgs::msg::ColorRGBA color =
                        Utils::valueToColor(probability(Vector2Int(a, b)), 0.00001, 0.1, Utils::valueColorMode::Logarithmic);

                    points.points.push_back(p);
                    points.colors.push_back(color);
                }
            }
        }
        probability_markers->publish(points);
    }

    Vector2Int GrGSL::coordinatesToIndex(double x, double y)
    {
        return Vector2Int((y - map_.info.origin.position.y) / (scale * map_.info.resolution),
                          (x - map_.info.origin.position.x) / (scale * map_.info.resolution));
    }

    Eigen::Vector2d GrGSL::indexToCoordinates(double i, double j)
    {
        return Eigen::Vector2d(map_.info.origin.position.x + (j + 0.5) * scale * map_.info.resolution,
                               map_.info.origin.position.y + (i + 0.5) * scale * map_.info.resolution);
    }

    double GrGSL::gaussian(double distance, double sigma)
    {
        return exp(-0.5 * (pow(distance, 2) / pow(sigma, 2))) / (sigma * sqrt(2 * M_PI));
    }

    Marker GrGSL::emptyMarker()
    {
        Marker points;
        points.header.frame_id = "map";
        points.header.stamp = node->now();
        points.ns = "cells";
        points.id = 0;
        points.type = Marker::POINTS;
        points.action = Marker::ADD;

        Eigen::Vector3d colour = valueToColor(1.0 / numCells, 0, 1);
        points.color.r = colour[0];
        points.color.g = colour[1];
        points.color.b = colour[2];
        points.color.a = 1.0;
        points.scale.x = 0.15;
        points.scale.y = 0.15;
        return points;
    }

    Eigen::Vector3d GrGSL::valueToColor(double val, double lowLimit, double highLimit)
    {
        double r, g, b;
        val = log10(val);
        double range = (log10(highLimit) - log10(lowLimit)) / 4;
        lowLimit = log10(lowLimit);
        if (val < lowLimit + range)
        {
            r = 0;
            g = std::max<double>((val - lowLimit) / (range), 0);
            b = 1;
        }
        else if (val < lowLimit + 2 * range)
        {
            r = 0;
            g = 1;
            b = 1 - std::max<double>((val - (lowLimit + range)) / (range), 0);
        }
        else if (val < lowLimit + 3 * range)
        {
            r = (val - (lowLimit + 2 * range)) / (range);
            g = 1;
            b = 0;
        }
        else
        {
            r = 1;
            g = 1 - std::max<double>(0, (val - (lowLimit + 3 * range)) / (range));
            b = 0;
        }
        return Eigen::Vector3d(r, g, b);
    }

    State GrGSL::getState()
    {
        return current_state;
    }

    Eigen::Vector2d GrGSL::expectedValueSource(double proportionBest)
    {

        std::vector<CellData> data;
        for (int i = 0; i < cells.size(); i++)
        {
            for (int j = 0; j < cells[0].size(); j++)
            {
                if (cells[i][j].free)
                {
                    CellData cd(Vector2Int(i, j), probability(Vector2Int(i, j)));
                    data.push_back(cd);
                }
            }
        }

        std::sort(data.begin(), data.end(), [](CellData a, CellData b) { return a.probability > b.probability; });

        double averageX = 0, averageY = 0;
        double sum = 0;

        for (int i = 0; i < data.size() * proportionBest; i++)
        {
            CellData& cd = data[i];
            Eigen::Vector2d coord = indexToCoordinates(cd.indices.x, cd.indices.y);
            averageX += cd.probability * coord.x();
            averageY += cd.probability * coord.y();
            sum += cd.probability;
        }
        return Eigen::Vector2d(averageX / sum, averageY / sum);
    }

    double GrGSL::varianceSourcePosition()
    {
        Eigen::Vector2d expected = expectedValueSource(1);
        double x = 0, y = 0;
        for (int i = 0; i < cells.size(); i++)
        {
            for (int j = 0; j < cells[0].size(); j++)
            {
                if (cells[i][j].free)
                {
                    Eigen::Vector2d coords = indexToCoordinates(i, j);
                    double p = probability(Vector2Int(i, j));
                    x += pow(coords.x() - expected.x(), 2) * p;
                    y += pow(coords.y() - expected.y(), 2) * p;
                }
            }
        }
        return x + y;
    }

    int GrGSL::checkSourceFound()
    {

        if (current_state != State::WAITING_FOR_MAP)
        {
            rclcpp::Duration time_spent = node->now() - start_time;
            if (time_spent.seconds() > max_search_time)
            {
                Eigen::Vector2d locationAll = expectedValueSource(1);
                Eigen::Vector2d location = expectedValueSource(0.05);
                save_results_to_file(0, location.x(), location.y(), locationAll.x(), locationAll.y());
                return 0;
            }

            if (!reached)
            {
                if (sqrt(pow(current_robot_pose.pose.pose.position.x - ground_truth_x, 2) +
                         pow(current_robot_pose.pose.pose.position.y - ground_truth_y, 2)) < 0.5)
                {
                    t1 = time_spent.seconds();
                    reached = true;
                }
            }
            double variance = varianceSourcePosition();
            std::cout << "Variance: " << variance << "\n";
            Eigen::Vector2d locationAll = expectedValueSource(1);
            bool converged = false;
            // if(estimatedSourceLocations.size()>=convergence_steps){
            //     converged = true;
            //     for(int i=0; i<estimatedSourceLocations.size(); i++){
            //         double distance = (locationAll-estimatedSourceLocations[i]).norm();
            //         if(distance > convergence_thr){
            //             std::cout<<"Convergence distance: "<< distance <<"\n";
            //             converged = false;
            //         }
            //     }
            //     estimatedSourceLocations.pop_front();
            // }

            if (variance < convergence_thr)
                converged = true;
            Eigen::Vector2d location = expectedValueSource(0.05);
            estimatedSourceLocations.push_back(locationAll);
            errorOverTimeAll.push_back((locationAll - Eigen::Vector2d(source_pose_x, source_pose_y)).norm());
            errorOverTime.push_back((location - Eigen::Vector2d(source_pose_x, source_pose_y)).norm());

            if (converged)
            {
                save_results_to_file(1, location.x(), location.y(), locationAll.x(), locationAll.y());
                return 1;
            }
        }
        return -1;
    }

    void GrGSL::save_results_to_file(int result, double i, double j, double allI, double allJ)
    {
        nav_client->async_cancel_all_goals();

        // 1. Search time.
        rclcpp::Duration time_spent = node->now() - start_time;
        double search_t = time_spent.seconds();

        std::string str = fmt::format("RESULT IS: Success={}, Search_t={} \n", result, search_t);
        spdlog::info(str);

        double error = sqrt(pow(ground_truth_x - i, 2) + pow(ground_truth_y - j, 2));
        double errorAll = sqrt(pow(ground_truth_x - allI, 2) + pow(ground_truth_y - allJ, 2));
        // Save to file
        if (results_file != "")
        {
            std::ofstream file;
            file.open(results_file, std::ios_base::app);
            if (result == 1)
                file << t1 << " " << search_t << " " << errorAll << " " << error << " " << exploredCells << " " << varianceSourcePosition() << "\n";
            else
                file << "FAILED " << t1 << " " << search_t << " " << errorAll << " " << error << " " << exploredCells << " "
                     << varianceSourcePosition() << "\n";
        }
        else
            spdlog::warn("No file provided for logging result. Skipping it.");

        if (errors_file != "")
        {
            std::ofstream file;
            file.open(errors_file, std::ios_base::app);
            file << "\n\n========================\n";
            for (int i = 0; i < errorOverTime.size(); i++)
                file << i << ", " << errorOverTime[i] << "\n";

            file << "------------------------\n";
            for (int i = 0; i < errorOverTimeAll.size(); i++)
                file << i << ", " << errorOverTimeAll[i] << "\n";
        }
        else
            spdlog::warn("No file provided for logging errors. Skipping it.");

        if (path_file != "")
        {
            std::ofstream file;
            file.open(path_file, std::ios_base::app);
            file << "------------------------\n";
            for (PoseWithCovarianceStamped p : robot_poses_vector)
                file << p.pose.pose.position.x << ", " << p.pose.pose.position.y << "\n";
        }
        else
            spdlog::warn("No file provided for logging path. Skipping it.");
    }

} // namespace GrGSL
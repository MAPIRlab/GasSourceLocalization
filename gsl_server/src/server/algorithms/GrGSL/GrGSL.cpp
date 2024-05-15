#include <gsl_server/algorithms/GrGSL/GrGSL.hpp>
#include <gsl_server/algorithms/GrGSL/MovingStateGrGSL.hpp>
#include <gsl_server/Utils/Math.hpp>
#include <gsl_server/Utils/RosUtils.hpp>
#include <angles/angles.h>
#include <fstream>

namespace GSL
{
    GrGSL::Cell::Cell(bool _free, double _weight) : free(_free), weight(_weight), auxWeight(0), distance(0)
    {}

    void GrGSL::Initialize()
    {
        Algorithm::Initialize();

        markers.probability_markers = node->create_publisher<Marker>("probability_markers", 10);
        markers.estimation_markers = node->create_publisher<Marker>("estimation_markers", 10);

        exploredCells = 0;

        waitForMapState = std::make_unique<WaitForMapState>(this);
        waitForGasState = std::make_unique<WaitForGasState>(this);
        stopAndMeasureState = std::make_unique<StopAndMeasureState>(this);
        movingState = std::make_unique<MovingStateGrGSL>(this);
        stateMachine.forceSetState(waitForMapState.get());
    }

    void GrGSL::declareParameters()
    {
        Algorithm::declareParameters();
        settings.useDiffusionTerm = getParam<bool>("useDiffusionTerm", false); //experimental way to extract useful info from low-wind measurements, not very reliable
        settings.stdev_hit = getParam<double>("stdev_hit", 1.0);
        settings.stdev_miss = getParam<double>("stdev_miss", 2.0);
        settings.infoTaxis = getParam<bool>("infoTaxis", false);
        settings.allowMovementRepetition = getParam<bool>("allowMovementRepetition", true);
        settings.convergence_thr = getParam<double>("convergence_thr", 0.5); // threshold for source declaration
        markers.markersHeight = getParam<float>("markers_height", 0);
    }

    void GrGSL::OnUpdate()
    {
        Algorithm::OnUpdate();
        functionQueue.run();
    }

    void GrGSL::onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        Algorithm::onGetMap(msg);
        functionQueue.submit(std::bind(&GrGSL::initializeMap, this));
    }

    void GrGSL::initializeMap()
    {
        // i is y, j is x

        std::vector<std::vector<bool>> boolMap(map.info.height, std::vector<bool>(map.info.width));
        int index = 0;
        for (int i = 0; i < boolMap.size(); i++)
        {
            for (int j = 0; j < boolMap[0].size(); j++)
            {
                boolMap[i][j] = map.data[index] == 0;
                index++;
            }
        }

        int scale = getParam<int>("scale", 65); // scale for dynamic map reduction
        grid.resize(ceil((float)map.info.height / scale));
        for (auto& cell : grid)
        {
            cell.resize(ceil((float)map.info.width / scale), Cell(false, 0));
        }

        gridMetadata.cellSize = map.info.resolution * scale;
        gridMetadata.origin.x = map.info.origin.position.x;
        gridMetadata.origin.y = map.info.origin.position.y;
        gridMetadata.numFreeCells = 0;

        int cellsI = 0, cellsJ = 0;
        for (int i = 0; i < boolMap.size(); i += scale)
        {
            cellsJ = 0;
            for (int j = 0; j < boolMap[0].size(); j += scale)
            {
                bool cellFree = true;
                for (int row = i; row < boolMap.size() && row < i + scale; row++)
                {
                    for (int col = j; col < boolMap[0].size() && col < j + scale; col++)
                    {
                        if (!boolMap[row][col])
                        {
                            cellFree = false;
                        }
                    }
                }
                Vector2 coord = gridMetadata.indexToCoordinates(cellsI, cellsJ, true);
                grid[cellsI][cellsJ].free = cellFree;
                grid[cellsI][cellsJ].weight = 1;
                grid[cellsI][cellsJ].auxWeight = -1; // this is used to prune the cells that are free but unreachable
                cellsJ++;

                if (cellFree)
                    gridMetadata.numFreeCells++;
            }
            cellsI++;
        }

        // remove "free" cells that are actually unreachable
        HashSet openPropagationSet;
        HashSet activePropagationSet;
        HashSet closedPropagationSet;

        Vector2Int currentIndices = gridMetadata.coordinatesToIndex(currentRobotPose.pose.pose);
        grid[currentIndices.x][currentIndices.y].auxWeight = 1;
        activePropagationSet.insert(currentIndices);
        propagateProbabilities(grid, openPropagationSet, closedPropagationSet, activePropagationSet);
        for (int i = 0; i < grid.size(); i++)
        {
            for (int j = 0; j < grid[0].size(); j++)
            {
                if (grid[i][j].auxWeight == -1)
                {
                    grid[i][j].weight = 0;
                    grid[i][j].free = false;
                    gridMetadata.numFreeCells--;
                }
            }
        }

        positionOfLastHit = {currentRobotPose.pose.pose.position.x, currentRobotPose.pose.pose.position.y};
        normalizeWeights(grid);
    }
	
    void GrGSL::processGasAndWindMeasurements(double concentration, double wind_speed, double wind_direction)
    {
        bool gasHit = concentration > thresholdGas;
        bool significantWind = wind_speed > thresholdWind;

        if (gasHit)
            positionOfLastHit = Vector2(currentRobotPose.pose.pose.position.x, currentRobotPose.pose.pose.position.y);

        if (gasHit && significantWind)
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "GAS HIT");
        else if (gasHit)
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "GAS BUT NO WIND");
        else if(significantWind)
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "ONLY WIND");
		else
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "NOTHING");
		
		if(gasHit)
        	estimateProbabilitiesfromGasAndWind(grid, gasHit, significantWind, wind_direction, gridMetadata.coordinatesToIndex(currentRobotPose.pose.pose));
		else
        	estimateProbabilitiesfromGasAndWind(grid, gasHit, true, wind_direction, gridMetadata.coordinatesToIndex(currentRobotPose.pose.pose));

        dynamic_cast<MovingStateGrGSL*>(movingState.get())->chooseGoalAndMove();
        showWeights();
    }

    void GrGSL::estimateProbabilitiesfromGasAndWind(std::vector<std::vector<Cell>>& map, bool hit, bool advection, double wind_direction,
                                                    Vector2Int robot_pos)
    {
		if(!advection && !(settings.useDiffusionTerm && hit))
			return;

        // ADVECTION
        // this part is always done, to calculate the distance field to the current robot position
        // the actual advection-based source probabilities are discarded if the wind speed is too low for the direction to be reliable
        HashSet openPropagationSet;
        HashSet activePropagationSet;
        HashSet closedPropagationSet;

        int i = robot_pos.x, j = robot_pos.y;

        int oI = std::max(0, i - 2);
        int fI = std::min((int)map.size() - 1, i + 2);
        int oJ = std::max(0, j - 2);
        int fJ = std::min((int)map[0].size() - 1, j + 2);

        // estimate the probabilities for the immediate neighbours
        Vector2 coordR = gridMetadata.indexToCoordinates(i, j);
        double upwind_dir = angles::normalize_angle(wind_direction + M_PI);
        double move_dir = atan2((positionOfLastHit.y - coordR.y), (positionOfLastHit.x - coordR.x)) + M_PI;

        for (int r = oI; r <= fI; r++)
        {
            for (int c = oJ; c <= fJ; c++)
            {
                if (!map[r][c].free)
                    continue;
                if (std::abs(c - j) > 1 || std::abs(r - i) > 1)
                {
                    Vector2 coordP = gridMetadata.indexToCoordinates(r, c);
                    double dist;
                    double cell_vector = atan2((coordR.y - coordP.y), (coordR.x - coordP.x));

                    if (hit)
                    {
                        dist = Utils::evaluate1DGaussian(atan2(sin(upwind_dir - cell_vector), cos(upwind_dir - cell_vector)), settings.stdev_hit);
                    }
                    else
                    {
                        dist = Utils::evaluate1DGaussian(atan2(sin(move_dir - cell_vector), cos(move_dir - cell_vector)), settings.stdev_miss);
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
                    map[r][c].auxWeight = Utils::evaluate1DGaussian((hit ? 0 : M_PI), (hit ? settings.stdev_hit : settings.stdev_miss));
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
			mapFunctionToCells(map, [&sum](Cell& cell, size_t row, size_t column)
				{sum += cell.auxWeight;}
			);
			mapFunctionToCells(map, [&sum](Cell& cell, size_t row, size_t column)
				{cell.auxWeight /= sum;}
			, MapFunctionMode::Parallel);
        }
        else
        {
			mapFunctionToCells(map, [](Cell& cell, size_t row, size_t column)
				{cell.auxWeight = 0;}, MapFunctionMode::Parallel);
        }

        // DIFFUSION
        if(settings.useDiffusionTerm && hit)
		{
        	// calculate and normalize these probabilities before combining them with the advection ones
        	std::vector<std::vector<double>> diffusionProb = std::vector<std::vector<double>>(map.size(), std::vector<double>(map[0].size(), 0));
			double sum = 0;
			mapFunctionToCells(map, [&sum, &diffusionProb](Cell& cell, size_t row, size_t column)
			{
				diffusionProb[row][column] = std::max(0.1, Utils::evaluate1DGaussian(cell.distance, 3));
				sum += diffusionProb[row][column];
			});
			
			if(sum>0)
            {
				mapFunctionToCells(map, [&sum, &diffusionProb](Cell& cell, size_t row, size_t column)
				{
					diffusionProb[row][column] /= sum;
					cell.auxWeight += diffusionProb[row][column];
				}, MapFunctionMode::Parallel);
            }
        }

		// BAYESIAN FILTER
		mapFunctionToCells(map, [](Cell& cell, size_t row, size_t column)
		{
			cell.weight *= cell.auxWeight;
			cell.auxWeight = 0;
		}, MapFunctionMode::Parallel);

        normalizeWeights(map);
    }

    void GrGSL::propagateProbabilities(std::vector<std::vector<Cell>>& map, HashSet& openPropagationSet, HashSet& closedPropagationSet,
                                       HashSet& activePropagationSet)
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

    void GrGSL::calculateWeight(std::vector<std::vector<Cell>>& map, int i, int j, Vector2Int p, HashSet& openPropagationSet,
                                HashSet& closedPropagationSet, HashSet& activePropagationSet)
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

    double GrGSL::informationGain(const WindVector& windVec)
    {
        // Kullback-Leibler Divergence
        static auto KLD = [this](std::vector<std::vector<Cell>>& a, std::vector<std::vector<Cell>>& b) -> double {
            double total = 0;
            for (int r = 0; r < a.size(); r++)
            {
                for (int c = 0; c < a[0].size(); c++)
                {
                    double aux = a[r][c].weight * log(a[r][c].weight / b[r][c].weight) +
                                 (1 - a[r][c].weight) * log((1 - a[r][c].weight) / (1 - b[r][c].weight));
                    total += std::isnan(aux) ? 0 : aux;
                }
            }
            return total;
        };

        auto predictionCells = grid; // temp copy of the matrix of cells that we can modify to simulate the effect of a measurement

        double entH = 0;
        estimateProbabilitiesfromGasAndWind(predictionCells, true, true, windVec.angle, Vector2Int(windVec.i, windVec.j));
        entH = KLD(predictionCells, grid);

        predictionCells = grid;
        estimateProbabilitiesfromGasAndWind(predictionCells, false, true, windVec.angle, Vector2Int(windVec.i, windVec.j));
        double entM = 0;
        entM = KLD(predictionCells, grid);

        return grid[windVec.i][windVec.j].weight * entH + (1 - grid[windVec.i][windVec.j].weight) * entM;
    }

    double GrGSL::probability(const Vector2Int& indices) const
    {
        return grid[indices.x][indices.y].weight;
    }

    GSLResult GrGSL::checkSourceFound()
    {
        if (stateMachine.getCurrentState() == waitForMapState.get() || stateMachine.getCurrentState() == waitForGasState.get())
            return GSLResult::Running;

        rclcpp::Duration time_spent = node->now() - start_time;
        if (time_spent.seconds() > resultLogging.max_search_time)
        {
            Vector2 locationAll = expectedValueSource(1);
            Vector2 location = expectedValueSource(0.05);
            saveResultsToFile(GSLResult::Failure);
            return GSLResult::Failure;
        }

        if (resultLogging.navigationTime == -1)
        {
            if (sqrt(pow(currentRobotPose.pose.pose.position.x - resultLogging.source_pose.x, 2) +
                     pow(currentRobotPose.pose.pose.position.y - resultLogging.source_pose.y, 2)) < 0.5)
            {
                resultLogging.navigationTime = time_spent.seconds();
            }
        }

        double variance = varianceSourcePosition();
        GSL_INFO("Variance: {}", variance);

        Vector2 locationAll = expectedValueSource(1);

        Vector2 location = expectedValueSource(0.05);

        if (variance < settings.convergence_thr)
        {
            saveResultsToFile(GSLResult::Success);
            return GSLResult::Success;
        }

        return GSLResult::Running;
    }

    Vector2 GrGSL::expectedValueSource(double proportionBest)
    {
        struct CellData
        {
            Vector2Int indices;
            double probability;
            CellData(Vector2Int ind, double prob)
            {
                indices = ind;
                probability = prob;
            }
        };
        std::vector<CellData> data;
        for (int i = 0; i < grid.size(); i++)
        {
            for (int j = 0; j < grid[0].size(); j++)
            {
                if (grid[i][j].free)
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
            Vector2 coord = gridMetadata.indexToCoordinates(cd.indices.x, cd.indices.y);
            averageX += cd.probability * coord.x;
            averageY += cd.probability * coord.y;
            sum += cd.probability;
        }
        return Vector2(averageX / sum, averageY / sum);
    }

    double GrGSL::varianceSourcePosition()
    {
        Vector2 expected = expectedValueSource(1);
        double x = 0, y = 0;
        for (int i = 0; i < grid.size(); i++)
        {
            for (int j = 0; j < grid[0].size(); j++)
            {
                if (grid[i][j].free)
                {
                    Vector2 coords = gridMetadata.indexToCoordinates(i, j);
                    double p = probability(Vector2Int(i, j));
                    x += pow(coords.x - expected.x, 2) * p;
                    y += pow(coords.y - expected.y, 2) * p;
                }
            }
        }
        return x + y;
    }

    void GrGSL::saveResultsToFile(GSLResult result)
    {
        // 1. Search time.
        rclcpp::Duration time_spent = node->now() - start_time;
        double search_t = time_spent.seconds();

        
        Vector2 sourceLocationAll = expectedValueSource(1);
        Vector2 sourceLocation = expectedValueSource(0.05);

        double error = sqrt(pow(resultLogging.source_pose.x - sourceLocation.x, 2) + pow(resultLogging.source_pose.y - sourceLocation.y, 2));
        double errorAll = sqrt(pow(resultLogging.source_pose.x - sourceLocationAll.x, 2) + pow(resultLogging.source_pose.y - sourceLocationAll.y, 2));
        
		std::string resultString = fmt::format("RESULT IS: Success={}, Search_t={:.2f}, Error={:.2f}", (int)result, search_t, error);
        GSL_INFO_COLOR(fmt::terminal_color::blue, "{}", resultString);
		
        // Save to file
        if (resultLogging.results_file != "")
        {
            std::ofstream file;
            file.open(resultLogging.results_file, std::ios_base::app);
            if (result != GSLResult::Success)
                file << "FAILED ";

            file << resultLogging.navigationTime << " " << search_t << " " << errorAll << " " << error << " " << exploredCells << " "
                 << varianceSourcePosition() << "\n";
            file.close();
        }
        else
            GSL_WARN("No file provided for logging result. Skipping it.");

        if (resultLogging.path_file != "")
        {
            std::ofstream file;
            file.open(resultLogging.path_file, std::ios_base::app);
            file << "------------------------\n";
            for (PoseWithCovarianceStamped p : resultLogging.robot_poses_vector)
                file << p.pose.pose.position.x << ", " << p.pose.pose.position.y << "\n";
            file.close();
        }
        else
            GSL_WARN("No file provided for logging path. Skipping it.");
    }

    void GrGSL::showWeights()
    {
        constexpr auto emptyMarker = []() {
            Marker points;
            points.header.frame_id = "map";
            points.ns = "cells";
            points.id = 0;
            points.type = Marker::POINTS;
            points.action = Marker::ADD;

            points.color.r = 1.0;
            points.color.g = 0.0;
            points.color.b = 1.0;
            points.color.a = 1.0;
            points.scale.x = 0.15;
            points.scale.y = 0.15;
            return points;
        };

        Marker points = emptyMarker();
        points.header.stamp = node->now();

        for (int a = 0; a < grid.size(); a++)
        {
            for (int b = 0; b < grid[0].size(); b++)
            {
                if (grid[a][b].free)
                {
                    auto coords = gridMetadata.indexToCoordinates(a, b);
                    Point p;
                    p.x = coords.x;
                    p.y = coords.y;
                    p.z = 0;

                    std_msgs::msg::ColorRGBA color =
                        Utils::valueToColor(probability(Vector2Int(a, b)), 0.00001, 0.1, Utils::valueColorMode::Logarithmic);

                    points.points.push_back(p);
                    points.colors.push_back(color);
                }
            }
        }
        markers.probability_markers->publish(points);
    }


	void GrGSL::mapFunctionToCells(std::vector<std::vector<Cell>>& cells, std::function<void(Cell&, size_t, size_t)> function, MapFunctionMode mode)
    {
        if (mode == MapFunctionMode::Parallel)
        {
#pragma omp parallel for collapse(2)
            for (int r = 0; r < cells.size(); r++)
            {
                for (int c = 0; c < cells[0].size(); c++)
                {
                    if (cells[r][c].free)
                        function(cells[r][c], r,c);
                }
            }
        }
		else
		{
			for (int r = 0; r < cells.size(); r++)
            {
                for (int c = 0; c < cells[0].size(); c++)
                {
                    if (cells[r][c].free)
                        function(cells[r][c], r,c);
                }
            }
		}
    }

} // namespace GSL
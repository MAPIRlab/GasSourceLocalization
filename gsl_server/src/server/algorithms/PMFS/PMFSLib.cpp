#include <DDA/DDA.h>
#include <angles/angles.h>
#include <gsl_server/Utils/Math.hpp>
#include <gsl_server/Utils/NQAQuadtree.hpp>
#include <gsl_server/algorithms/PMFS/PMFSLib.hpp>
#include <gsl_server/core/Logging.hpp>

namespace GSL
{

    //---------------
    // P(H)
    //---------------

    void PMFSLib::estimateHitProbabilities(Grid<HitProbability>& hitProb, const VisibilityMap& visibilityMap,
                                           PMFS_internal::HitProbabilitySettings& settings, bool hit, double downwindDirection, double windSpeed,
                                           Vector2Int robotPosition)
    {
        // receiving propagation this step. Can still be modified by better estimations.
        HashSet openPropagationSet;
        // causing propagation this step. No longer modifyable
        HashSet activePropagationSet;
        // old news
        HashSet closedPropagationSet;

        int i = robotPosition.x, j = robotPosition.y;

        int oI = std::max(0, i - settings.localEstimationWindowSize);
        int fI = std::min((int)hitProb.metadata.height - 1, i + settings.localEstimationWindowSize);
        int oJ = std::max(0, j - settings.localEstimationWindowSize);
        int fJ = std::min((int)hitProb.metadata.width - 1, j + settings.localEstimationWindowSize);

        Vector2Int ij(i, j);

        double kernel_rotation_wind = -angles::normalize_angle(
            downwindDirection + M_PI / 2); // the orientation of the anemometer's frame of reference and the way it uses angles is weird, man
        HitProbKernel kernel = {
            kernel_rotation_wind,
            Vector2(settings.kernelSigma + settings.kernelStretchConstant * windSpeed,                             // semi-major ellipse axis
                    settings.kernelSigma / (1 + settings.kernelStretchConstant * windSpeed / settings.kernelSigma) // semi-minor axis
                    ),
            (hit ? 0.6f : 0.1f)};

        for (int r = oI; r <= fI; r++)
        {
            for (int c = oJ; c <= fJ; c++)
            {
                Vector2Int rc(r, c);
                if (visibilityMap.isVisible(ij, rc) != Visibility::Visible)
                    continue;

                // the neighbours that are between 1 and 3 cells away
                if (std::abs(c - j) >= 1 || std::abs(r - i) >= 1)
                {
                    if (hitProb.occupancyAt(r, c) == Occupancy::Free)
                        activePropagationSet.insert(rc);
                    else
                        closedPropagationSet.insert(rc);
                }
                // the ones that are right next to the robot
                else
                    closedPropagationSet.insert(rc);

                hitProb.dataAt(r, c).distanceFromRobot = vmath::length(Vector2(rc - ij));
                hitProb.dataAt(r, c).originalPropagationDirection = vmath::normalized(Vector2(rc - ij)); // TODO check this. Should be coordinates,
                                                                                                         // no?
                hitProb.dataAt(r, c).auxWeight = applyFalloffLogOdds(Vector2(rc - ij), kernel, settings);
            }
        }

        // propagate these short-range estimations to the entire environment using the navigation map
        // also calculate the distance field
        propagateProbabilities(hitProb, settings, openPropagationSet, closedPropagationSet, activePropagationSet, kernel);

        double logOddsPrior = std::log(settings.prior / (1 - settings.prior));
        for (size_t i = 0; i < hitProb.data.size(); i++)
        {
            if (hitProb.occupancy[i] != Occupancy::Free)
                continue;

            HitProbability& cell = hitProb.data[i];
            // BAYESIAN BINARY FILTER. this is the important part
            cell.logOdds += cell.auxWeight - logOddsPrior;
            GSL_ASSERT(!std::isnan(cell.logOdds));

            // confidence
            cell.omega += Utils::evaluate1DGaussian(cell.distanceFromRobot, settings.confidenceSigmaSpatial);
            double exponent = -cell.omega / std::pow(settings.confidenceMeasurementWeight, 2);
            cell.confidence = 1 - std::exp(exponent);
        }
    }

    double PMFSLib::propagateProbabilities(Grid<HitProbability>& hitProb, const PMFS_internal::HitProbabilitySettings& settings,
                                           HashSet& openPropagationSet, HashSet& closedPropagationSet, HashSet& activePropagationSet,
                                           const HitProbKernel& kernel)
    {
        double total = 0;

        auto calculateNewAuxWeight = [&openPropagationSet, &closedPropagationSet, &activePropagationSet, &kernel, &settings,
                                      &hitProb](int i, int j, Vector2Int previousCellIndices) {
            HitProbability& previousCell = hitProb.dataAt(previousCellIndices.x, previousCellIndices.y);
            HitProbability& currentCell = hitProb.dataAt(i, j);
            Vector2Int ij(i, j);

            float newDistance = previousCell.distanceFromRobot +
                                ((i == previousCellIndices.x || j == previousCellIndices.y) ? 1 : sqrt(2)); // distance of this new path to the cell
            float newWeight = applyFalloffLogOdds(previousCell.originalPropagationDirection * newDistance, kernel, settings);

            // if the cell can still receive propagation
            if (closedPropagationSet.find(ij) == closedPropagationSet.end() && activePropagationSet.find(ij) == activePropagationSet.end())
            {
                // if there already was a path to this cell
                if (openPropagationSet.find(ij) != openPropagationSet.end())
                {
                    // if the distance is the same, keep the best probability!
                    if (std::abs(newDistance - currentCell.distanceFromRobot) < 0.001 && newWeight > currentCell.auxWeight)
                    {
                        currentCell.auxWeight = newWeight;
                        currentCell.originalPropagationDirection = previousCell.originalPropagationDirection;
                    }
                    // else, keep the shortest path
                    else if (newDistance < currentCell.distanceFromRobot)
                    {
                        currentCell.auxWeight = newWeight;
                        currentCell.distanceFromRobot = newDistance;
                        currentCell.originalPropagationDirection = previousCell.originalPropagationDirection;
                    }
                }
                // if this is the first time we reach this cell
                else
                {
                    currentCell.auxWeight = newWeight;
                    currentCell.distanceFromRobot = newDistance;
                    currentCell.originalPropagationDirection = previousCell.originalPropagationDirection;
                    openPropagationSet.insert(ij);
                }
            }
        };

        while (!activePropagationSet.empty())
        {
            while (!activePropagationSet.empty())
            {
                Vector2Int p = *activePropagationSet.begin();

                if (hitProb.freeAt(p.x, p.y))
                    total += hitProb.dataAt(p.x, p.y).auxWeight;

                activePropagationSet.erase(activePropagationSet.begin());
                closedPropagationSet.insert(p);

                int oR = std::max(0, p.x - 1);
                int fR = std::min((int)hitProb.metadata.height - 1, p.x + 1);
                int oC = std::max(0, p.y - 1);
                int fC = std::min((int)hitProb.metadata.width - 1, p.y + 1);

                // 8-neighbour propagation
                for (int i = oR; i <= fR; i++)
                {
                    for (int j = oC; j <= fC; j++)
                    {
                        if (hitProb.freeAt(i, j))
                            calculateNewAuxWeight(i, j, p);
                    }
                }
            }

            activePropagationSet.clear();
            // the cells that were in the open set now are removed from it and can no longer receive propagation
            // therefore, the probabilities for these cells are now locked, and we can update the logodds
            for (const auto& par : openPropagationSet)
            {
                activePropagationSet.insert(par);
            }

            openPropagationSet.clear();
        }
        return total;
    }

    double PMFSLib::applyFalloffLogOdds(Vector2 originalVectorScaled, const HitProbKernel& kernel,
                                        const PMFS_internal::HitProbabilitySettings& settings)
    {
        double sampleGaussian = Utils::evaluate2DGaussian(originalVectorScaled, kernel.sigma, kernel.angle);
        double maxPossible = Utils::evaluate2DGaussian({0, 0}, kernel.sigma, kernel.angle);
        float prob = Utils::clamp(Utils::lerp(settings.prior, kernel.valueAt1, sampleGaussian / maxPossible), 0.001, 0.999);
        return std::log(prob / (1 - prob));
    }

    void PMFSLib::initMetadata(GridMetadata& metadata, const OccupancyGrid& map, int scale)
    {
        metadata.cellSize = map.info.resolution * scale;
        metadata.origin.x = map.info.origin.position.x;
        metadata.origin.y = map.info.origin.position.y;
        metadata.numFreeCells = 0;
        metadata.height = floor((float)map.info.height / scale);
        metadata.width = floor((float)map.info.width / scale);
        metadata.scale = scale;
    }

    void PMFSLib::initializeMap(Algorithm& algorithm, Grid<HitProbability> grid, PMFS_internal::Simulations& simulations,
                                VisibilityMap& visibilityMap)
    {
        // create the PMFS cells
        {
            GridUtils::reduceOccupancyMap(algorithm.map.data, algorithm.map.info.width, grid.occupancy, grid.metadata);
            for(HitProbability& hp : grid.data)
                hp.auxWeight = -1;
            GSL_TRACE("Created grid");
        }

        std::vector<std::vector<uint8_t>> occupancyMap(grid.metadata.height, std::vector<uint8_t>(grid.metadata.width));
        // prune unreachable free cells
        {
            // get an arbitrary seed value into the cell the robot is currently in and propagagate it. If a cell has not received that value by the
            // end, it must not be reachable
            {
                HashSet openPropagationSet;
                HashSet activePropagationSet;
                HashSet closedPropagationSet;
                Vector2Int currentIndices = grid.metadata.coordinatesToIndex(algorithm.currentRobotPose.pose.pose.position.x,
                                                                             algorithm.currentRobotPose.pose.pose.position.y);
                grid.dataAt(currentIndices.x, currentIndices.y).auxWeight = 0; // the arbitrary value
                activePropagationSet.insert(currentIndices);
                PMFSLib::propagateProbabilities(grid, PMFS_internal::HitProbabilitySettings(), openPropagationSet, closedPropagationSet,
                                                activePropagationSet, {1, {1, 0}, 0});
            }

            for (int i = 0; i < grid.metadata.height; i++)
            {
                for (int j = 0; j < grid.metadata.width; j++)
                {
                    if (!grid.freeAt(i, j) || grid.dataAt(i, j).auxWeight == -1)
                    {
                        grid.dataAt(i, j).logOdds = DBL_MIN;
                        grid.occupancyAt(i, j) = Occupancy::Obstacle;
                    }
                    else
                        grid.metadata.numFreeCells++;
                    occupancyMap[i][j] = grid.occupancyAt(i, j) == Occupancy::Free ? 1 : 0;
                }
            }
            GSL_TRACE("Pruned unreachable cells");
        }

        // precomputed visibility map
        {
            for (int i = 0; i < grid.metadata.height; i++)
            {
                for (int j = 0; j < grid.metadata.width; j++)
                {
                    Vector2Int ij(i, j);
                    if (!grid.freeAt(i, j))
                    {
                        visibilityMap.emplace(ij, {});
                        continue;
                    }
                    int oI = std::max(0, i - (int)visibilityMap.range) - i;
                    int fI = std::min((int)grid.metadata.height - 1, i + (int)visibilityMap.range) - i;
                    int oJ = std::max(0, j - (int)visibilityMap.range) - j;
                    int fJ = std::min((int)grid.metadata.width - 1, j + (int)visibilityMap.range) - j;

                    std::vector<Vector2Int> visibleCells;
                    double totalX = 0;
                    double totalY = 0;
                    for (int r = oI; r <= fI; r++)
                    {
                        for (int c = oJ; c <= fJ; c++)
                        {
                            Vector2Int thisCell(i + r, j + c);
                            Vector2 start = grid.metadata.indexToCoordinates(ij);
                            Vector2 end = grid.metadata.indexToCoordinates(thisCell);
                            if (thisCell == ij || pathFree(grid.metadata, grid.occupancy, start, end))
                                visibleCells.push_back(thisCell);
                        }
                    }
                    visibilityMap.emplace(ij, visibleCells);
                }
            }
            GSL_TRACE("Created visibility map");
        }

        simulations.initializeMap(occupancyMap);
        simulations.visibilityMap = &visibilityMap;
        simulations.varianceOfHitProb.resize(grid.metadata.height * grid.metadata.width, 0);
    }

    void PMFSLib::initializeWindPredictions(Algorithm& algorithm, Grid<Vector2> grid,
                                            std::shared_ptr<WindEstimation::Request>& GMRFRequest
                                                IF_GADEN(, std::shared_ptr<gaden_player::srv::WindPosition::Request>& groundTruthWindRequest))
    {
        grid.data.resize(grid.metadata.height * grid.metadata.width);
        std::string anemometer_frame = algorithm.getParam<std::string>("anemometer_frame", "anemometer_frame");

        geometry_msgs::msg::TransformStamped tfm;
        bool has_tf = false;
        do
        {
            try
            {
                tfm = algorithm.tfBufffer.buffer.lookupTransform("map", anemometer_frame, rclcpp::Time(0));
                has_tf = true;
            }
            catch (std::exception& e)
            {
                GSL_ERROR("TF error when looking up map -> anemometer:\n{}", e.what());
                rclcpp::spin_some(algorithm.node);
            }
        } while (!has_tf);

        float anemometer_Z = tfm.transform.translation.z;
        GSL_INFO("anemometer z is {}", anemometer_Z);

        GMRFRequest = std::make_shared<WindEstimation::Request>();
        IF_GADEN(groundTruthWindRequest = std::make_shared<gaden_player::srv::WindPosition::Request>());

        for (int i = 0; i < grid.metadata.height; i++)
        {
            for (int j = 0; j < grid.metadata.width; j++)
            {
                if (grid.freeAt(i, j))
                {
                    Vector2 coords = grid.metadata.indexToCoordinates(i, j);
                    GMRFRequest->x.push_back(coords.x);
                    GMRFRequest->y.push_back(coords.y);
#ifdef USE_GADEN
                    groundTruthWindRequest->x.push_back(coords.x);
                    groundTruthWindRequest->y.push_back(coords.y);
                    groundTruthWindRequest->z.push_back(anemometer_Z);
#endif
                }
            }
        }
    }

    void PMFSLib::estimateWind(bool useGroundTruth, Grid<Vector2> estimatedWind, std::shared_ptr<rclcpp::Node> node,
                               PMFS_internal::GMRFWind& gmrf IF_GADEN(, PMFS_internal::GroundTruthWind& groundTruth))
    {
        // if not compiled with gaden support, you have no choice but to use GMRF :)
#ifdef USE_GADEN
        if (!useGroundTruth)
        {
#endif
            // ask the gmrf_wind service for the estimated wind vector in cell i,j
            auto future = gmrf.client->async_send_request(gmrf.request);
            auto result = rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5));

            if (result == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = future.get();
                for (int ind = 0; ind < gmrf.request->x.size(); ind++)
                {
                    Vector2Int pair = estimatedWind.metadata.coordinatesToIndex(gmrf.request->x[ind], gmrf.request->y[ind]);
                    estimatedWind.dataAt(pair.x, pair.y) = Vector2(std::cos(response->v[ind]), std::sin(response->v[ind])) * (float)response->u[ind];
                }
            }
            else
                GSL_WARN("CANNOT READ ESTIMATED WIND VECTORS");

#ifdef USE_GADEN
        }

        else
        {
            auto future = groundTruth.client->async_send_request(groundTruth.request);
            auto result = rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5));
            if (result == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = future.get();
                for (int ind = 0; ind < groundTruth.request->x.size(); ind++)
                {
                    Vector2Int pair = estimatedWind.metadata.coordinatesToIndex(groundTruth.request->x[ind], groundTruth.request->y[ind]);
                    estimatedWind.dataAt(pair.x, pair.y) = Vector2(response->u[ind], response->v[ind]);
                }
            }
            else
                GSL_WARN("CANNOT READ ESTIMATED WIND VECTORS");
        }
#endif
    }

    bool PMFSLib::pathFree(GridMetadata metadata, const std::vector<Occupancy>& occupancy, const Vector2& origin, const Vector2& end)
    {
        Vector2Int originInd = metadata.coordinatesToIndex(origin);
        Vector2Int endInd = metadata.coordinatesToIndex(end);

        // check there are no obstacles between origin and end
        if (!(occupancy[metadata.indexOf(originInd)] == Occupancy::Free && occupancy[metadata.indexOf(endInd)] == Occupancy::Free))
            return false;
        Vector2 direction = end - origin;
        DDA::_2D::RayCastInfo raycastInfo = DDA::_2D::castRay<GSL::Occupancy>(
            origin, direction, vmath::length(direction),
            DDA::_2D::Map<GSL::Occupancy>(occupancy, metadata.origin, metadata.cellSize, {metadata.width, metadata.height}),
            [](const GSL::Occupancy& occ) { return occ == GSL::Occupancy::Free; });

        return !raycastInfo.hitSomething;
    }

    void PMFSLib::GetSimulationSettings(Algorithm& algorithm, PMFS_internal::SimulationSettings& settings)
    {
        settings.useWindGroundTruth = algorithm.getParam<bool>("useWindGroundTruth", false);
        settings.sourceDiscriminationPower = algorithm.getParam<double>("sourceDiscriminationPower", 1);
        settings.refineFraction = algorithm.getParam<double>("refineFraction", 10);
        settings.stepsBetweenSourceUpdates = algorithm.getParam<int>("stepsSourceUpdate", 10);
        settings.maxRegionSize = algorithm.getParam<int>("maxRegionSize", 5);
        settings.deltaTime = algorithm.getParam<double>("deltaTime", 0.2);
        settings.noiseSTDev = algorithm.getParam<double>("noiseSTDev", 0.5);
        settings.iterationsToRecord = algorithm.getParam<int>("iterationsToRecord", 200);
        settings.maxWarmupIterations = algorithm.getParam<int>("maxWarmupIterations", 500);
    }

    void PMFSLib::GetHitProbabilitySettings(Algorithm& algorithm, PMFS_internal::HitProbabilitySettings& settings)
    {
        settings.prior = algorithm.getParam<double>("hitPriorProbability", 0.1);
        settings.maxUpdatesPerStop = algorithm.getParam<int>("maxUpdatesPerStop", 3);
        settings.kernelSigma = algorithm.getParam<double>("kernelSigma", 0.5);
        settings.kernelStretchConstant = algorithm.getParam<double>("kernelStretchConstant", 1);
        settings.confidenceMeasurementWeight = algorithm.getParam<double>("confidenceMeasurementWeight", 0.5);
        settings.confidenceSigmaSpatial = algorithm.getParam<double>("confidenceSigmaSpatial", 0.5);
        settings.localEstimationWindowSize = algorithm.getParam<int>("localEstimationWindowSize", 2);
    }

    void PMFSLib::InitializePublishers(PMFS_internal::PublishersAndSubscribers& pubs, rclcpp::Node::SharedPtr node)
    {
        pubs.markers.sourceProbabilityMarkers = node->create_publisher<Marker>("probabilityMarkers", 1);
        pubs.markers.hitProbabilityMarkers = node->create_publisher<Marker>("hitProbabilityMarkers", 1);
        pubs.markers.confidenceMarkers = node->create_publisher<Marker>("confidenceMarkers", 1);
        pubs.markers.windArrowMarkers = node->create_publisher<MarkerArray>("arrowMarkers", 1);

        pubs.markers.quadtreePublisher = node->create_publisher<MarkerArray>("quadtree", 1);

        pubs.gmrfWind.client = node->create_client<gmrf_wind_mapping::srv::WindEstimation>("/WindEstimation");
        IF_GADEN(pubs.groundTruthWind.client = node->create_client<gaden_player::srv::WindPosition>("/wind_value"));
    }

} // namespace GSL

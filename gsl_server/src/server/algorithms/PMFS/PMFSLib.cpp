#include <gsl_server/algorithms/PMFS/PMFSLib.hpp>
#include <gsl_server/core/logging.hpp>
#include <gsl_server/Utils/Math.hpp>
#include <gsl_server/Utils/NQAQuadtree.hpp>
#include <angles/angles.h>

namespace GSL
{

    //---------------
    // P(H)
    //---------------

    void PMFSLib::estimateHitProbabilities(Grid<HitProbability>& localVariable, const std::unordered_map<Vector2Int, HashSet>& visibilityMap, PMFS_internal::Settings::HitProbabilitySettings& settings, bool hit,
                                           double downwind_direction, double wind_speed, Vector2Int robot_pos)
    {

        // receiving propagation this step. Can still be modified by better estimations.
        HashSet openPropagationSet;
        // causing propagation this step. No longer modifyable
        HashSet activePropagationSet;
        // old news
        HashSet closedPropagationSet;

        int i = robot_pos.x, j = robot_pos.y;

        int oI = std::max(0, i - settings.localEstimationWindowSize);
        int fI = std::min(localVariable.metadata.height - 1, i + settings.localEstimationWindowSize);
        int oJ = std::max(0, j - settings.localEstimationWindowSize);
        int fJ = std::min(localVariable.metadata.width - 1, j + settings.localEstimationWindowSize);

        Vector2Int ij(i, j);

        double kernel_rotation_wind = -angles::normalize_angle(
            downwind_direction + M_PI / 2); // the orientation of the anemometer's frame of reference and the way it uses angles is weird, man
        HitProbKernel kernel = {
            kernel_rotation_wind,
            Vector2(settings.kernel_sigma + settings.kernel_stretch_constant * wind_speed, // semi-major ellipse axis
                    settings.kernel_sigma /
                        (1 + settings.kernel_stretch_constant * wind_speed / settings.kernel_sigma) // semi-minor axis
                    ),
            (hit ? 0.6f : 0.1f)};

        for (int r = oI; r <= fI; r++)
        {
            for (int c = oJ; c <= fJ; c++)
            {
                Vector2Int rc(r, c);
                const HashSet& set = visibilityMap.at(ij);
                if (set.find(rc) == set.end())
                    continue;

                // the neighbours that are between 1 and 3 cells away
                if (std::abs(c - j) >= 1 || std::abs(r - i) >= 1)
                {
                    if (localVariable.occupancyAt(r, c) == Occupancy::Free)
                        activePropagationSet.insert(rc);
                    else
                        closedPropagationSet.insert(rc);
                }
                // the ones that are right next to the robot
                else
                    closedPropagationSet.insert(rc);

                localVariable.dataAt(r, c).auxWeight = applyFalloffLogOdds(rc - ij, kernel, settings);
                localVariable.dataAt(r, c).distanceFromRobot = glm::length(Vector2(rc - ij));
                localVariable.dataAt(r, c).originalPropagationDirection = glm::normalize(Vector2(rc - ij));
            }
        }

        // propagate these short-range estimations to the entire environment using the navigation map
        // also calculate the distance field
        propagateProbabilities(localVariable, settings, openPropagationSet, closedPropagationSet, activePropagationSet, kernel);

        double logOddsPrior = std::log(settings.prior / (1 - settings.prior));
        for (HitProbability& hitprob : localVariable.data)
        {
            // BAYESIAN BINARY FILTER. this is the important part
            hitprob.logOdds += hitprob.auxWeight - logOddsPrior;
            GSL_ASSERT(!std::isnan(hitprob.logOdds));

            // confidence
            double& omega = hitprob.omega;
            omega += Utils::evaluate1DGaussian(hitprob.distanceFromRobot, settings.confidence_sigma_spatial);
            double exponent = -omega / std::pow(settings.confidence_measurement_weight, 2);
            hitprob.confidence = 1 - std::exp(exponent);
        }
    }

    double PMFSLib::propagateProbabilities(Grid<HitProbability>& var, const PMFS_internal::Settings::HitProbabilitySettings& settings,
                                           HashSet& openPropagationSet, HashSet& closedPropagationSet, HashSet& activePropagationSet,
                                           const HitProbKernel& kernel)
    {
        double total = 0;

        auto calculateNewAuxWeight = [&openPropagationSet, &closedPropagationSet, &activePropagationSet, &kernel, &settings,
                                      &var](int i, int j, Vector2Int previousCellIndices) {
            HitProbability& previousCell = var.dataAt(previousCellIndices.x, previousCellIndices.y);
            HitProbability& currentCell = var.dataAt(i, j);
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

                if (var.freeAt(p.x, p.y))
                    total += var.dataAt(p.x, p.y).auxWeight;

                activePropagationSet.erase(activePropagationSet.begin());
                closedPropagationSet.insert(p);

                int oR = std::max(0, p.x - 1);
                int fR = std::min(var.metadata.height - 1, p.x + 1);
                int oC = std::max(0, p.y - 1);
                int fC = std::min(var.metadata.width - 1, p.y + 1);

                // 8-neighbour propagation
                for (int i = oR; i <= fR; i++)
                {
                    for (int j = oC; j <= fC; j++)
                    {
                        if (var.freeAt(i, j))
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
                                        const PMFS_internal::Settings::HitProbabilitySettings& settings)
    {
        double sampleGaussian = Utils::evaluate2DGaussian(originalVectorScaled, kernel.sigma, kernel.angle);
        double maxPossible = Utils::evaluate2DGaussian({0, 0}, kernel.sigma, kernel.angle);
        float prob = Utils::clamp(Utils::lerp(settings.prior, kernel.valueAt1, sampleGaussian / maxPossible), 0.001, 0.999);
        return std::log(prob / (1 - prob));
    }

    void PMFSLib::initMetadata(GridMetadata& metadata, const OccupancyGrid& map,  int scale)
    {
        metadata.cellSize = map.info.resolution * scale;
        metadata.origin.x = map.info.origin.position.x;
        metadata.origin.y = map.info.origin.position.y;
        metadata.numFreeCells = 0;
        metadata.height = floor((float)map.info.height / scale);
        metadata.width = floor((float)map.info.width / scale);
    }

    void PMFSLib::initializeMap(Algorithm& algorithm, Grid<HitProbability> grid, const PMFS_internal::Settings& settings,
                                PMFS_internal::Simulations& simulations, std::unordered_map<Vector2Int, HashSet>& visibilityMap)
    {
        std::vector<std::vector<uint8_t>> mapa(algorithm.map.info.height, std::vector<uint8_t>(algorithm.map.info.width));
        {
            int index = 0;
            for (int i = 0; i < mapa.size(); i++)
            {
                for (int j = 0; j < mapa[0].size(); j++)
                {
                    mapa[i][j] = algorithm.map.data[index];
                    index++;
                }
            }
        }


        // create the cells
        {
            int scale = algorithm.getParam<int>("scale", 65); // scale for dynamic map reduction
            for (int i = 0; i < grid.metadata.height; i++)
            {
                for (int j = 0; j < grid.metadata.width; j++)
                {
                    bool squareIsFree = true;

                    for (int row = i * scale; row < (i + 1) * scale; row++)
                    {
                        for (int col = j * scale; col < (j + 1) * scale; col++)
                        {
                            if (mapa[row][col] != 0)
                            {
                                squareIsFree = false;
                            }
                        }
                    }
                    grid.occupancyAt(i,j) = squareIsFree? Occupancy::Free : Occupancy::Obstacle;
                    grid.dataAt(i,j).logOdds = std::log(settings.hitProbability.prior / (1 - settings.hitProbability.prior));
                    grid.dataAt(i,j).auxWeight = -1; // this is used to prune the cells that are free but unreachable
                }
            }
        }

        // pruning unreachable free cells
        {
            {
                HashSet openPropagationSet;
                HashSet activePropagationSet;
                HashSet closedPropagationSet;
                Vector2Int currentIndices = grid.metadata.coordinatesToIndex(algorithm.currentRobotPose.pose.pose.position.x,
                                                                             algorithm.currentRobotPose.pose.pose.position.y);
                grid.dataAt(currentIndices.x, currentIndices.y).auxWeight = 0;
                activePropagationSet.insert(currentIndices);
                PMFSLib::propagateProbabilities(grid, settings.hitProbability, openPropagationSet, closedPropagationSet, activePropagationSet, {1, {1, 0}, 0});
            }

            std::vector<std::vector<uint8_t>> occupancyMap(grid.metadata.height, std::vector<uint8_t>(grid.metadata.width));
            for (int i = 0; i < grid.metadata.height; i++)
            {
                for (int j = 0; j < grid.metadata.width; j++)
                {
                    if (!grid.freeAt(i,j) || grid.dataAt(i,j).auxWeight == -1)
                    {
                        grid.dataAt(i,j).logOdds = DBL_MIN;
                        grid.occupancyAt(i,j) = Occupancy::Obstacle;
                    }
                    occupancyMap[i][j] = grid.occupancyAt(i,j) == Occupancy::Free ? 1 : 0;
                }
            }

            simulations.quadtree = std::make_unique<Utils::NQA::Quadtree>(occupancyMap);
            simulations.QTleaves = simulations.quadtree->fusedLeaves(settings.simulation.maxRegionSize);

            simulations.mapSegmentation.resize(grid.metadata.height, std::vector<Utils::NQA::Node*>(grid.metadata.width, nullptr));

            GSL_INFO("Number of cells after fusing quadtree: {0}", simulations.QTleaves.size());
            // generate the image of indices so you can map a cell in the map to the corresponding leaf of the quatree
            for (int i = 0; i < simulations.QTleaves.size(); i++)
            {
                Utils::NQA::Node& node = simulations.QTleaves[i];
                Vector2Int start = node.origin;
                Vector2Int end = node.origin + node.size;

                for (int r = start.x; r < end.x; r++)
                {
                    for (int c = start.y; c < end.y; c++)
                    {
                        GSL_ASSERT_MSG(simulations.mapSegmentation[r][c] == nullptr, "fused cells are overlapping");
                        simulations.mapSegmentation[r][c] = &node;
                    }
                }
            }
        }

        // precomputed visibility map
        {
            int visibility_map_range = std::max(settings.movement.openMoveSetExpasion, settings.hitProbability.localEstimationWindowSize);
            for (int i = 0; i < grid.metadata.height; i++)
            {
                for (int j = 0; j < grid.metadata.width; j++)
                {
                    Vector2Int ij(i, j);
                    if (grid.freeAt(i,j))
					{
						visibilityMap.emplace(ij, HashSet{});
                        continue;
					}
                    int oI = std::max(0, i - visibility_map_range) - i;
                    int fI = std::min(grid.metadata.height - 1, i + visibility_map_range) - i;
                    int oJ = std::max(0, j - visibility_map_range) - j;
                    int fJ = std::min(grid.metadata.width - 1, j + visibility_map_range) - j;

                    HashSet visibleCells;
                    double totalX = 0;
                    double totalY = 0;
                    for (int r = oI; r <= fI; r++)
                    {
                        for (int c = oJ; c <= fJ; c++)
                        {
                            Vector2Int thisCell(i + r, j + c);
                            if (pathFree(grid, ij, thisCell))
                            {
                                visibleCells.insert(thisCell);
                            }
                        }
                    }
                    visibilityMap.emplace(ij, visibleCells);
                }
            }
        }

        


        simulations.varianceOfHitProb.resize(grid.metadata.height, std::vector<double>(grid.metadata.width, 0));
    }

    void PMFSLib::initializeWindPredictions(Algorithm& algorithm, Grid<Vector2> grid, std::shared_ptr<WindEstimation::Request>& GMRFRequest
#ifdef USE_GADEN
        , std::shared_ptr<gaden_player::srv::WindPosition::Request>& groundTruthWindRequest
#endif
    )
    {
        grid.data.resize(grid.metadata.height * grid.metadata.width);
        std::string anemometer_frame = algorithm.getParam<std::string>("anemometer_frame", "anemometer_frame");

        geometry_msgs::msg::TransformStamped tfm;
        bool has_tf = false;
        do
        {
            try
            {
                tfm = algorithm.tf_buffer.buffer.lookupTransform("map", anemometer_frame, rclcpp::Time(0));
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
#ifdef USE_GADEN
        groundTruthWindRequest = std::make_shared<gaden_player::srv::WindPosition::Request>();
#endif

        for (int i = 0; i < grid.metadata.height; i++)
        {
            for (int j = 0; j < grid.metadata.width; j++)
            {
                if (grid.freeAt(i,j))
                {
                    grid.metadata.numFreeCells++;
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
                               PMFS_internal::PublishersAndSubscribers::GMRFWind& gmrf
#ifdef USE_GADEN
                               , PMFS_internal::PublishersAndSubscribers::GroundTruthWind& groundTruth
#endif
    )
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



    bool PMFSLib::indicesInBounds(const Vector2Int indices, const GridMetadata& metadata)
    {
        return indices.x >= 0 && indices.x < metadata.width && indices.y >= 0 && indices.y < metadata.height;
    }

} // namespace GSL

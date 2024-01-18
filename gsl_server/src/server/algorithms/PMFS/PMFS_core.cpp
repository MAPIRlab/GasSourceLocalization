#include <gsl_server/algorithms/PMFS/PMFS.hpp>
#include <gsl_server/Utils/Math.hpp>
#include <angles/angles.h>


namespace GSL
{
    using hashSet = std::unordered_set<Vector2Int>;
    using Cell = PMFS_internal::Cell;

    void PMFS::OnUpdate()
    {
        if(!paused)
            Algorithm::OnUpdate();
        else
        {
            showWeights();
            dynamic_cast<MovingStatePMFS*>(movingState.get())->publishMarkers();
        }

        functionQueue.run();
    }

    void PMFS::processGasAndWindMeasurements(double concentration, double wind_speed, double wind_direction)
    {
        static int number_of_updates = 0;

        if (concentration > thresholdGas)
        {
            // Gas & wind
            estimateHitProbabilities(grid, true, wind_direction, wind_speed, currentPosIndex());
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "GAS HIT");
        }
        else
        {
            // Nothing
            estimateHitProbabilities(grid, false, wind_direction, wind_speed, currentPosIndex());
           GSL_INFO_COLOR(fmt::terminal_color::yellow, "NOTHING ");
        }

        estimateWind(settings.simulation.useWindGroundTruth);
        plotWindVectors();

        number_of_updates++;

        if (number_of_updates >= settings.hitProbability.max_updates_per_stop)
        {
            number_of_updates = 0;
            bool timeToSimulate = iterationsCounter >= settings.movement.initialExplorationMoves &&
                                  iterationsCounter % settings.simulation.steps_between_source_updates == 0;
            if (timeToSimulate)
            {
                // simulations.compareRefineFractions();
                simulations.updateSourceProbability(settings.simulation.refineFraction);
            }

            auto movingStatePMFS = dynamic_cast<MovingStatePMFS*>(movingState.get());
            if (iterationsCounter > settings.movement.initialExplorationMoves)
                movingStatePMFS->currentMovement = MovingStatePMFS::MovementType::Search;
            else
                movingStatePMFS->currentMovement = MovingStatePMFS::MovementType::Exploration;

            movingStatePMFS->chooseGoalAndMove();
            movingStatePMFS->publishMarkers();
        }
        else
            stateMachine.forceResetState(stopAndMeasureState.get());

        showWeights();
    }
    

    //---------------
    // P(H)
    //---------------

    void PMFS::estimateHitProbabilities(std::vector<std::vector<Cell>>& localVariable, bool hit, double downwind_direction, double wind_speed,
                                        Vector2Int robot_pos, bool infotaxis_sim)
    {

        // receiving propagation this step. Can still be modified by better estimations.
        hashSet openPropagationSet;
        // causing propagation this step. No longer modifyable
        hashSet activePropagationSet;
        // old news
        hashSet closedPropagationSet;

        int i = robot_pos.x, j = robot_pos.y;

        int oI = std::max(0, i - settings.hitProbability.localEstimationWindowSize);
        int fI = std::min((int)localVariable.size() - 1, i + settings.hitProbability.localEstimationWindowSize);
        int oJ = std::max(0, j - settings.hitProbability.localEstimationWindowSize);
        int fJ = std::min((int)localVariable[0].size() - 1, j + settings.hitProbability.localEstimationWindowSize);

        Vector2Int ij(i, j);

        double kernel_rotation_wind = -angles::normalize_angle(
            downwind_direction + M_PI / 2); // the orientation of the anemometer's frame of reference and the way it uses angles is weird, man
        
        constexpr double measurementStrength = 10;
        HitProbKernel kernel = {
            kernel_rotation_wind,
            Vector2(settings.hitProbability.kernel_sigma + settings.hitProbability.kernel_stretch_constant * wind_speed, // semi-major ellipse axis
                    settings.hitProbability.kernel_sigma /
                        (1 + settings.hitProbability.kernel_stretch_constant * wind_speed / settings.hitProbability.kernel_sigma) // semi-minor axis
                    ),
            (hit ? settings.hitProbability.prior+0.02 * measurementStrength : settings.hitProbability.prior-0.01 * measurementStrength)};

        for (int r = oI; r <= fI; r++)
        {
            for (int c = oJ; c <= fJ; c++)
            {
                Vector2Int rc(r, c);
                hashSet& set = visibilityMap.at(ij);
                if (set.find(rc) == set.end())
                    continue;

                // the neighbours that are between 1 and 3 cells away
                if (std::abs(c - j) >= 1 || std::abs(r - i) >= 1)
                {
                    if (localVariable[r][c].free)
                        activePropagationSet.insert(rc);
                    else
                        closedPropagationSet.insert(rc);
                }
                // the ones that are right next to the robot
                else
                    closedPropagationSet.insert(rc);

                localVariable[r][c].hitProbability.auxWeight = applyFalloffLogOdds(rc - ij, kernel);
                localVariable[r][c].distanceFromRobot = glm::length(Vector2(rc - ij));
                localVariable[r][c].hitProbability.originalPropagationDirection = glm::normalize(Vector2(rc - ij));
            }
        }

        // propagate these short-range estimations to the entire environment using the navigation map
        // also calculate the distance field
        propagateProbabilities(localVariable, openPropagationSet, closedPropagationSet, activePropagationSet, kernel);

        double logOddsPrior = std::log(settings.hitProbability.prior / (1 - settings.hitProbability.prior));
        for (int r = 0; r < localVariable.size(); r++)
        {
            for (int c = 0; c < localVariable[0].size(); c++)
            {
                // BAYESIAN BINARY FILTER. this is the important part
                localVariable[r][c].hitProbability.logOdds += localVariable[r][c].hitProbability.auxWeight - logOddsPrior;
                GSL_ASSERT(!std::isnan(localVariable[r][c].hitProbability.logOdds));

                // confidence
                if (!infotaxis_sim)
                {
                    double& omega = localVariable[r][c].hitProbability.omega;
                    omega += Utils::evaluate1DGaussian(localVariable[r][c].distanceFromRobot, settings.hitProbability.confidence_sigma_spatial);
                    double exponent = -omega / std::pow(settings.hitProbability.confidence_measurement_weight, 2);
                    localVariable[r][c].hitProbability.confidence = 1 - std::exp(exponent);
                }
            }
        }
    }

    double PMFS::propagateProbabilities(std::vector<std::vector<Cell>>& var, hashSet& openPropagationSet, hashSet& closedPropagationSet,
                                        hashSet& activePropagationSet, const HitProbKernel& kernel)
    {
        double total = 0;

        auto calculateNewAuxWeight = [this, &openPropagationSet, &closedPropagationSet, &activePropagationSet, &kernel,
                                      &var](int i, int j, Vector2Int previousCellIndices) {
            Cell& previousCell = var[previousCellIndices.x][previousCellIndices.y];
            Cell& currentCell = var[i][j];
            Vector2Int ij(i, j);

            float newDistance = previousCell.distanceFromRobot +
                                ((i == previousCellIndices.x || j == previousCellIndices.y) ? 1 : sqrt(2)); // distance of this new path to the cell
            float newWeight = applyFalloffLogOdds(previousCell.hitProbability.originalPropagationDirection * newDistance, kernel);

            // if the cell can still receive propagation
            if (closedPropagationSet.find(ij) == closedPropagationSet.end() && activePropagationSet.find(ij) == activePropagationSet.end())
            {

                // if there already was a path to this cell
                if (openPropagationSet.find(ij) != openPropagationSet.end())
                {
                    // if the distance is the same, keep the best probability!
                    if (std::abs(newDistance - currentCell.distanceFromRobot) < 0.001 && newWeight > currentCell.hitProbability.auxWeight)
                    {
                        currentCell.hitProbability.auxWeight = newWeight;
                        currentCell.hitProbability.originalPropagationDirection = previousCell.hitProbability.originalPropagationDirection;
                    }
                    // else, keep the shortest path
                    else if (newDistance < currentCell.distanceFromRobot)
                    {
                        currentCell.hitProbability.auxWeight = newWeight;
                        currentCell.distanceFromRobot = newDistance;
                        currentCell.hitProbability.originalPropagationDirection = previousCell.hitProbability.originalPropagationDirection;
                    }
                }
                // if this is the first time we reach this cell
                else
                {
                    currentCell.hitProbability.auxWeight = newWeight;
                    currentCell.distanceFromRobot = newDistance;
                    currentCell.hitProbability.originalPropagationDirection = previousCell.hitProbability.originalPropagationDirection;
                    openPropagationSet.insert(ij);
                }
            }
        };

        while (!activePropagationSet.empty())
        {
            while (!activePropagationSet.empty())
            {
                Vector2Int p = *activePropagationSet.begin();

                if (var[p.x][p.y].free)
                    total += var[p.x][p.y].hitProbability.auxWeight;

                activePropagationSet.erase(activePropagationSet.begin());
                closedPropagationSet.insert(p);

                int oR = std::max(0, p.x - 1);
                int fR = std::min((int)var.size() - 1, p.x + 1);
                int oC = std::max(0, p.y - 1);
                int fC = std::min((int)var[0].size() - 1, p.y + 1);

                // 8-neighbour propagation
                for (int i = oR; i <= fR; i++)
                {
                    for (int j = oC; j <= fC; j++)
                    {
                        calculateNewAuxWeight(i, j, p);
                    }
                }
            }

            activePropagationSet.clear();
            // the cells that were in the open set now are removed from it and can no longer receive propagation
            // therefore, the probabilities for these cells are now locked, and we can update the logodds
            for (const auto& par : openPropagationSet)
            {

                // if the cell is free, it gets to propagate its estimations. Otherwise, even though it has received a propagation and has a
                // probability itself, it does not propagate
                if (var[par.x][par.y].free)
                    activePropagationSet.insert(par);
                else
                    closedPropagationSet.insert(par);
            }

            openPropagationSet.clear();
        }
        return total;
    }

    double PMFS::applyFalloffLogOdds(Vector2 originalVectorScaled, const HitProbKernel& kernel)
    {
        double sampleGaussian = Utils::evaluate2DGaussian(originalVectorScaled, kernel.sigma, kernel.angle);
        double maxPossible = Utils::evaluate2DGaussian({0, 0}, kernel.sigma, kernel.angle);
        float prob = Utils::clamp(Utils::lerp(settings.hitProbability.prior, kernel.valueAt1, sampleGaussian / maxPossible), 0.001, 0.999);
        return std::log(prob / (1 - prob));
    }

    float PMFS::gasCallback(olfaction_msgs::msg::GasSensor::SharedPtr msg) 
    {
        float ppm = Algorithm::gasCallback(msg);
#if USE_GUI
        ui.addConcentrationReading(ppm);
#endif
        return ppm;
    }


} // namespace GSL
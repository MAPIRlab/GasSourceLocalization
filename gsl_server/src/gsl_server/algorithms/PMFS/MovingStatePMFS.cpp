#include "gsl_server/algorithms/Common/Utils/RosUtils.hpp"
#include "gsl_server/core/Logging.hpp"
#include "gsl_server/core/Navigation.hpp"
#include "gsl_server/core/Profiling.hpp"
#include "gsl_server/core/VectorsImpl/vmath_DDACustomVec.hpp"
#include <algorithm>
#include <angles/angles.h>
#include <gsl_server/algorithms/Common/Grid2D.hpp>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/PMFS/MovingStatePMFS.hpp>
#include <gsl_server/algorithms/PMFS/PMFS.hpp>
#include <gsl_server/algorithms/PMFS/internal/HitProbability.hpp>
#include <std_msgs/msg/detail/color_rgba__struct.hpp>

namespace GSL
{
    MovingStatePMFS::MovingStatePMFS(Algorithm* _algorithm)
        : MovingState(_algorithm)
    {
        pmfs = dynamic_cast<PMFS*>(_algorithm);

        publishers.explorationValue = pmfs->node->create_publisher<Marker>("explorationValue", 1);
        publishers.varianceHit = pmfs->node->create_publisher<Marker>("varianceHit", 1);
        publishers.movementSets = pmfs->node->create_publisher<Marker>("movementSets", 1);
    }

    void MovingStatePMFS::chooseGoalAndMove()
    {
        // ScopedStopwatch watch("movement");

        if (movesCounter > pmfs->settings.movement.initialExplorationMoves)
            currentMovement = MovingStatePMFS::MovementType::Search;
        else
            currentMovement = MovingStatePMFS::MovementType::Exploration;

        auto& gridMetadata = pmfs->gridMetadata;

        // Add nearby cells to the open set
        {
            int i = pmfs->gridMetadata.coordinatesToIndices(pmfs->currentRobotPose.pose.pose).x;
            int j = pmfs->gridMetadata.coordinatesToIndices(pmfs->currentRobotPose.pose.pose).y;

            int openMoveSetExpasion = pmfs->settings.movement.openMoveSetExpasion;
            int oC = std::max(0, i - openMoveSetExpasion);
            int fC = std::min((int)gridMetadata.dimensions.x - 1, i + openMoveSetExpasion);
            int oR = std::max(0, j - openMoveSetExpasion);
            int fR = std::min((int)gridMetadata.dimensions.y - 1, j + openMoveSetExpasion);

            for (int col = oC; col <= fC; col++)
            {
                for (int row = oR; row <= fR; row++)
                {
                    Vector2Int p(col, row);
                    if (closedMoveSet.find(p) == closedMoveSet.end() && pmfs->visibilityMap->isVisible({i, j}, p) == Visibility::Visible)
                        openMoveSet.insert(p);
                }
            }
        }

        // remove this cell from the open set
        openMoveSet.erase(pmfs->gridMetadata.coordinatesToIndices(pmfs->currentRobotPose.pose.pose));

        // Find the cell with the highest estimated information value
        //------------------------------------------------------
        calculateMutualInformationGas();
        NavigateToPose::Goal goal;

        // We have a small random chance of using the explorationValue instead of the proper information value even in the second phase
        // because it is beneficial to have at least some measurements spanning a large area of the map
        float explorationC = Utils::uniformRandom(0, 1);

        struct PositionEval
        {
            Vector2Int indices;
            double evaluation;
        };

        auto compareEval = [](const PositionEval& a, const PositionEval& b)
        {
            return a.evaluation > b.evaluation;
        };
        std::set<PositionEval, decltype(compareEval)> evaluations;

        for (size_t i = 0; i < pmfs->sourceProbability.size(); i++)
        {
            if (pmfs->occupancy[i] != Occupancy::Free)
                continue;
            Vector2Int indices = gridMetadata.indices2D(i);
            double explorationTerm = explorationValue(indices.x, indices.y);

            double interest = currentMovement == MovementType::Exploration || explorationC < pmfs->settings.movement.explorationProbability
                                  ? explorationTerm
                                  : informationValue(indices.x, indices.y);

            // this is to include the navigation distance in the evaluation
            double evaluation = interest / std::pow(pmfs->hitProbability[i].distanceFromRobot + 0.1f, pmfs->settings.movement.distanceWeight);

            evaluations.insert({.indices = indices, .evaluation = interest});
        }

        for (const PositionEval& eval : evaluations)
        {
            NavigateToPose::Goal tempGoal = indexToGoal(eval.indices.x, eval.indices.y);
            if (checkGoal(tempGoal))
            {
                goal = tempGoal;
                break;
            }
        }

        if (closedMoveSet.find(pmfs->gridMetadata.coordinatesToIndices(pmfs->currentRobotPose.pose.pose)) == closedMoveSet.end())
            openMoveSet.insert(pmfs->gridMetadata.coordinatesToIndices(pmfs->currentRobotPose.pose.pose));

        movesCounter++;
        publishMarkers();

        sendGoal(goal);
    }

    // Calculating this is a bit complex, specially if you want to do it in a reasonable amount of time
    // The main idea is to evaluate how much information about the source a specific cell gives by considering the case in which you know its hit frequency perfectly (confidence of 1)
    // You calculate the source prob distribution in that hypothetical case, and take the change in entropy with respect to the current entropy
    // The thing is, you can set the confidence to 1, but with what hit frequency?
    // Well, we can calculate a probability distribution for the hit frequency from the *current* confidence, and use that to then calculate the expected value of the entropy reduction
    // And that's what this does. There is some additional complexity in making it run fast (explained below), where we avoid re-comparing entire maps after modifying a single cell

    // Also, for speed reasons we are currently using the result of the coarsest simulation level instead of the final source distribution.
    // Could probably be changed without it becoming too slow. Probably.
    void MovingStatePMFS::calculateMutualInformationGas()
    {
        ScopedStopwatch watch("MutualInformation");
        ZoneScopedN("MutualInformation");

        // normalize the current source probs
        {
            double sum = 0;
            for (const auto& simResult : pmfs->simulations.resultsFirstLevel)
            {
                if (!simResult.valid)
                    continue;
                sum += simResult.sourceProb;
            }
            for (auto& simResult : pmfs->simulations.resultsFirstLevel)
                simResult.sourceProb /= sum;
        }

        // get the probability of each hit frequency in each cell, according to the current p(h_i) and the confidence value
        constexpr size_t discretizationLevels = PMFS_internal::HitProbability::numBuckets;
        std::vector<std::array<double, discretizationLevels>> probF(pmfs->hitProbability.size());
        for (size_t i = 0; i < probF.size(); i++)
            probF[i] = pmfs->hitProbability[i].frequencyDistribution();

        // use the hit frequency probabilities to calculate the conditional entropy
        std::vector<double> conditionalEntropy(pmfs->sourceProbability.size(), 0.0);
        {
            ZoneScopedN("ConditionalEntropy");

            // The cell index needs to be the outer loop, because we need to normalize the source probabilities given knowledge of this cell
#pragma omp parallel for
            for (size_t i = 0; i < conditionalEntropy.size(); i++)
            {
                if (pmfs->occupancy[i] != Occupancy::Free)
                    continue;

                // iterate over the list of possible hit frequency values for this one cell
                for (size_t bucket = 0; bucket < discretizationLevels; bucket++)
                {
                    double probabilityOfFreq = probF[i][bucket];
                    double freq = (bucket + 0.5) * (1. / discretizationLevels);

                    // We need to normalize the source probabilities before calculating the entropy
                    //-----------------------------------------------------

                    // store the source probs for later normalization
                    std::vector<long double> sourceProbs(pmfs->simulations.resultsFirstLevel.size(), 0.0);
                    long double sum = 0; // for normalizing

                    for (size_t simulationIndex = 0; simulationIndex < pmfs->simulations.resultsFirstLevel.size(); simulationIndex++)
                    {
                        const auto& simResult = pmfs->simulations.resultsFirstLevel[simulationIndex];
                        if (!simResult.valid)
                            continue;

                        // instead of calculating the source probability by comparing the two maps (the simulated one, and the measured one with a single cell modified),
                        // we use the already calculated source prob. If we divide p(s_k | f) by the current p(s_k | f_i) and then multiply by the modified p(s_k | f_i*),
                        // we get the same result but avoid re-comparing all the untouched cells

                        // current p(s_k | f_i)
                        double probGivenThisCell = pmfs->simulations.probabilityFromSingleCell(pmfs->hitProbability[i], simResult.hitMap[i]);

                        PMFS_internal::HitProbability localCopy = pmfs->hitProbability[i];
                        localCopy.setProbability(freq);
                        localCopy.confidence = 1;
                        double probWithNewFreq = pmfs->simulations.probabilityFromSingleCell(localCopy, simResult.hitMap[i]);

                        // source prob after modifying this cell in the map
                        // we store it in a vector because we need to normalize before calculating the entropy
                        sourceProbs[simulationIndex] = (simResult.sourceProb / probGivenThisCell) * probWithNewFreq;
                        sum += sourceProbs[simulationIndex];
                    }

                    // calculate the entropy for this particular conditional
                    double entropyThisFreq = 0;
                    for (size_t simulationIndex = 0; simulationIndex < pmfs->simulations.resultsFirstLevel.size(); simulationIndex++)
                    {
                        const auto& simResult = pmfs->simulations.resultsFirstLevel[simulationIndex];
                        if (!simResult.valid)
                            continue;

                        long double sourceProbWithFreq = sourceProbs[simulationIndex] / sum;
                        entropyThisFreq -= sourceProbWithFreq * std::log(sourceProbWithFreq);
                    }

                    // the conditional entropy is an expected value, so multiply by the probability of this frequency and add to a running total
                    conditionalEntropy[i] += probabilityOfFreq * entropyThisFreq;
                }
            }
        }

        // calculate the mutual information: H(S) - H(S|F)
        {
            mutualInformationGas.clear();
            mutualInformationGas.resize(pmfs->sourceProbability.size(), 0.0);
            double entropyS = 0;
            for (size_t i = 0; i < pmfs->sourceProbability.size(); i++)
            {
                if (pmfs->occupancy[i] != Occupancy::Free)
                    continue;
                double p = pmfs->sourceProbability[i];
                entropyS -= p * std::log(p);
            }

            for (size_t i = 0; i < conditionalEntropy.size(); i++)
            {
                if (pmfs->occupancy[i] != Occupancy::Free)
                    continue;
                mutualInformationGas[i] = entropyS - conditionalEntropy[i];
            }
        }

        // visualization with rviz markers
        {
            double min = DBL_MAX;
            for (size_t i = 0; i < mutualInformationGas.size(); i++)
            {
                if (pmfs->occupancy[i] != Occupancy::Free)
                    continue;
                min = std::min(min, mutualInformationGas[i]);
            }

            auto maxVal = *std::max_element(mutualInformationGas.begin(), mutualInformationGas.end());
            GSL_INFO("Max mutual info: {:.3f}", maxVal);

            std::vector<std_msgs::msg::ColorRGBA> colors(mutualInformationGas.size());
            for (size_t i = 0; i < mutualInformationGas.size(); i++)
                colors[i] = Utils::valueToColor(mutualInformationGas[i], min, maxVal, Utils::valueColorMode::Linear);

            Utils::publishDebugMarkers(
                Grid2D<std_msgs::msg::ColorRGBA>(colors, pmfs->occupancy, pmfs->gridMetadata),
                "MutualInformation");
        }
    }

    double MovingStatePMFS::explorationValue(int i, int j)
    {
        // the exploration value is the sum of the uncertainty about the hit probability for all cells around (i,j)

        Vector2Int ij(i, j);
        auto range = pmfs->visibilityMap->at(ij);

        double sum = 0;
        for (const auto& p : range)
        {
            float distance = vmath::length(Vector2(ij - p)); // not the navigable distance, but we are close enough that it does not matter
            sum += (1 - pmfs->hitProbability[pmfs->gridMetadata.indexOf(p)].confidence) * std::exp(-distance);
            GSL_ASSERT(sum > 0);
        }
        return sum;
    }

    double MovingStatePMFS::informationValue(int i, int j)
    {
        auto& gridMetadata = pmfs->gridMetadata;
        Vector2Int ij(i, j);
        auto range = pmfs->visibilityMap->at(ij);

        double sum = 0;
        for (const auto& p : range)
        {
            // double varianceTerm = mutualInformationGas[gridMetadata.indexOf(indices)];
            double varianceTerm = pmfs->simulations.varianceOfHitProb[gridMetadata.indexOf(i, j)] * (1 - pmfs->hitProbability[gridMetadata.indexOf(i, j)].confidence);

            float distance = vmath::length(Vector2(ij - p)); // not the navigable distance, but we are close enough that it does not matter
            sum += varianceTerm * std::exp(-distance);
            GSL_ASSERT(sum > 0);
        }
        return sum;
    }

    NavigateToPose::Goal MovingStatePMFS::indexToGoal(int i, int j)
    {
        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = pmfs->node->now();

        Vector2 pos = pmfs->gridMetadata.indicesToCoordinates(i, j);
        Vector2 coordR(pmfs->currentRobotPose.pose.pose.position.x, pmfs->currentRobotPose.pose.pose.position.y);

        double move_angle = (std::atan2(pos.y - coordR.y, pos.x - coordR.x));
        goal.pose.pose.position.x = pos.x;
        goal.pose.pose.position.y = pos.y;
        goal.pose.pose.orientation = Utils::createQuaternionMsgFromYaw(angles::normalize_angle(move_angle));
#ifdef USE_NAV_ASSISTANT
        goal.turn_before_nav = true;
#endif
        return goal;
    }

    void MovingStatePMFS::debugMoveTo(int i, int j)
    {
        GSL_INFO("Sending a goal from UI. This is not part of the normal execution flow of the algorithm");
        currentGoal = std::nullopt;
        NavigateToPose::Goal goal = indexToGoal(i, j);
        sendGoal(goal);
    }

    void MovingStatePMFS::Fail()
    {
        // There is a somewhat inconsistent behaviour when a goal times out and is manually cancelled.
        // sometimes we get a callback notifying of the cancelation, sometimes not
        // when we do, Fail() gets called twice: once by us, once inside the callback
        // if this is the second time, we should not do anything
        if (!currentGoal.has_value())
            return;

        Vector2Int indicesGoal = pmfs->gridMetadata.coordinatesToIndices(currentGoal.value().pose.pose);
        openMoveSet.erase(indicesGoal);
        closedMoveSet.insert(indicesGoal);
        MovingState::Fail();
    }

    void MovingStatePMFS::publishMarkers()
    {
        Grid2DMetadata& gridMetadata = pmfs->gridMetadata;
        Grid2D<PMFS_internal::HitProbability> grid(pmfs->hitProbability, pmfs->occupancy, gridMetadata);

        Marker explorationMarker = Utils::emptyMarker({0.2, 0.2}, pmfs->node->get_clock());

        Marker varianceMarker = explorationMarker;

        Marker movementSetsMarker = explorationMarker;

        double maxExpl = -DBL_MAX;
        double maxVar = -DBL_MAX;
        double minExpl = DBL_MAX;
        double minVar = DBL_MAX;
        for (int b = 0; b < gridMetadata.dimensions.y; b++)
        {
            for (int a = 0; a < gridMetadata.dimensions.x; a++)
            {
                if (!grid.freeAt(a, b))
                    continue;
                maxExpl = std::max(maxExpl, explorationValue(a, b));
                maxVar = std::max(maxVar, pmfs->simulations.varianceOfHitProb[gridMetadata.indexOf({a, b})] * (1 - grid.dataAt(a, b).confidence));

                minExpl = std::min(minExpl, explorationValue(a, b));
                minVar = std::min(minVar, pmfs->simulations.varianceOfHitProb[gridMetadata.indexOf({a, b})] * (1 - grid.dataAt(a, b).confidence));
            }
        }

        for (int b = 0; b < gridMetadata.dimensions.y; b++)
        {
            for (int a = 0; a < gridMetadata.dimensions.x; a++)
            {
                if (!grid.freeAt(a, b))
                    continue;
                auto coords = gridMetadata.indicesToCoordinates(a, b);
                Point p;
                p.x = coords.x;
                p.y = coords.y;
                p.z = pmfs->settings.visualization.markers_height;

                std_msgs::msg::ColorRGBA explorationColor;
                std_msgs::msg::ColorRGBA varianceColor;

                if (openMoveSet.find(Vector2Int(a, b)) == openMoveSet.end())
                {
                    explorationColor.r = 0;
                    explorationColor.g = 0;
                    explorationColor.b = 0;
                    explorationColor.a = 1;
                }
                else
                {
                    explorationColor = Utils::valueToColor(explorationValue(a, b), minExpl, maxExpl, Utils::valueColorMode::Linear);
                }
                varianceColor =
                    Utils::valueToColor(pmfs->simulations.varianceOfHitProb[gridMetadata.indexOf({a, b})] * (1 - grid.dataAt(a, b).confidence),
                                        minVar, maxVar, Utils::valueColorMode::Linear);

                explorationMarker.points.push_back(p);
                explorationMarker.colors.push_back(explorationColor);

                p.z = pmfs->settings.visualization.markers_height - 0.1;
                varianceMarker.points.push_back(p);
                varianceMarker.colors.push_back(varianceColor);

                movementSetsMarker.points.push_back(p);
                if (openMoveSet.find({a, b}) != openMoveSet.end())
                    movementSetsMarker.colors.push_back(Utils::create_color(0, 1, 0, 1));
                else if (closedMoveSet.find(Vector2Int(a, b)) != closedMoveSet.end())
                    movementSetsMarker.colors.push_back(Utils::create_color(1, 0, 0, 1));
                else
                    movementSetsMarker.colors.push_back(Utils::create_color(0, 0, 1, 1));
            }
        }
        publishers.explorationValue->publish(explorationMarker);
        publishers.varianceHit->publish(varianceMarker);
        publishers.movementSets->publish(movementSetsMarker);
    }

} // namespace GSL
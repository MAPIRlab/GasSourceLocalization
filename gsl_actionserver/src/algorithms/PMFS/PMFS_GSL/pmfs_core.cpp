#include <algorithms/PMFS/PMFS.h>
namespace PMFS{



void PMFS_GSL::processGasWindObservations()
{
    static int number_of_updates = 0;
    if( (ros::Time::now() - time_stopped).toSec() >= settings.stop_and_measure_time )
    {
        AveragedMeasurement averaged= getAveragedMeasurement();

        if (averaged.concentration > settings.th_gas_present)
        {
            //Gas & wind
            estimateHitProbabilities(cells, true, averaged.wind_direction, averaged.wind_speed, currentPosIndex);
            if (verbose) spdlog::warn(" GAS HIT");
        }
        else
        {
            //Nothing
            estimateHitProbabilities(cells, false, averaged.wind_direction, averaged.wind_speed, currentPosIndex);
            if (verbose) spdlog::warn(" NOTHING ");
        }

        estimateWind(settings.simulation.useWindGroundTruth);
        

        time_stopped = ros::Time::now();
        number_of_updates++;    
        if(number_of_updates >= settings.hitProbability.max_updates_per_stop)
        {
            iterationsCounter++;
            previous_state=current_state;
            current_state= iterationsCounter > settings.movement.initialExplorationMoves? State::MOVING : State::EXPLORATION;
            number_of_updates = 0;
            bool timeToSimulate = iterationsCounter >= settings.movement.initialExplorationMoves && iterationsCounter % settings.simulation.steps_between_source_updates==0 ;
            if( timeToSimulate ){
                //simulations.compareRefineFractions();
                simulations.updateSourceProbability(settings.simulation.refineFraction);
            }
        }
    }
}


//---------------
    //P(H)
//---------------

void PMFS_GSL::estimateHitProbabilities(std::vector<std::vector<Cell> >& localVariable,
                                    bool hit,
                                    double downwind_direction,
                                    double wind_speed,
                                    Vector2Int robot_pos,
                                    bool infotaxis_sim){

    //receiving propagation this step. Can still be modified by better estimations.
    hashSet openPropagationSet;
    //causing propagation this step. No longer modifyable
    hashSet activePropagationSet;
    //old news
    hashSet closedPropagationSet;

    int i=robot_pos.x, j=robot_pos.y;

    int oI=std::max(0,i-settings.localEstimationWindowSize);
    int fI=std::min((int) localVariable.size()-1,i+settings.localEstimationWindowSize);
    int oJ=std::max(0,j-settings.localEstimationWindowSize);
    int fJ=std::min((int) localVariable[0].size()-1,j+settings.localEstimationWindowSize);
    
    Vector2Int ij(i,j);

    double kernel_rotation_wind = -angles::normalize_angle(downwind_direction+ M_PI/2); // the orientation of the anemometer's frame of reference and the way it uses angles is weird, man 
    HitProbKernel kernel = { 
        kernel_rotation_wind, 
        Utils::Vector2(
            settings.hitProbability.kernel_sigma + settings.hitProbability.kernel_stretch_constant * wind_speed, //semi-major ellipse axis 
            settings.hitProbability.kernel_sigma / (1 + settings.hitProbability.kernel_stretch_constant * wind_speed / settings.hitProbability.kernel_sigma) //semi-minor axis
        ),
        (hit?0.6f:0.1f) 
    };
    

    for(int r=oI;r<=fI;r++){
        for(int c=oJ;c<=fJ;c++){
            Vector2Int rc(r,c);
            hashSet& set = visibilityMap.at(ij);
            if(set.find(rc) == set.end())
                continue;
            
            //the neighbours that are between 1 and 3 cells away
            if(std::abs(c-j)>=1||std::abs(r-i)>=1){
                if(localVariable[r][c].free)
                    activePropagationSet.insert(rc);
                else
                    closedPropagationSet.insert(rc);
            }
            //the ones that are right next to the robot
            else          
                closedPropagationSet.insert(rc);

            localVariable[r][c].hitProbability.auxWeight= applyFalloffLogOdds(rc-ij, kernel);
            localVariable[r][c].distanceFromRobot=(rc-ij).norm();
            localVariable[r][c].hitProbability.originalPropagationDirection = Utils::Vector2(rc-ij).normalized();
        }
    }

    
    //propagate these short-range estimations to the entire environment using the navigation map
    //also calculate the distance field
    propagateProbabilities(localVariable, openPropagationSet, closedPropagationSet, activePropagationSet, kernel);

    double logOddsPrior = std::log(settings.hitProbability.prior / (1-settings.hitProbability.prior));
    for(int r = 0; r<localVariable.size();r++){
        for(int c = 0; c<localVariable[0].size();c++){
            //BAYESIAN BINARY FILTER. this is the important part
            localVariable[r][c].hitProbability.logOdds += localVariable[r][c].hitProbability.auxWeight - logOddsPrior;
            GSL_ASSERT(!std::isnan(localVariable[r][c].hitProbability.logOdds));

            //confidence
            if(!infotaxis_sim){
                double& omega = localVariable[r][c].hitProbability.omega;
                omega += Utils::evaluate1DGaussian(localVariable[r][c].distanceFromRobot, settings.hitProbability.confidence_sigma_spatial);
                double exponent = -omega / std::pow(settings.hitProbability.confidence_measurement_weight,2);
                localVariable[r][c].hitProbability.confidence = 1 - std::exp(exponent);
            }
        }
    }
}


double PMFS_GSL::propagateProbabilities(std::vector<std::vector<Cell> >& var,
                                    hashSet& openPropagationSet,
                                    hashSet& closedPropagationSet,
                                    hashSet& activePropagationSet, 
                                    const HitProbKernel& kernel)
{
    double total = 0;

    auto calculateNewAuxWeight = [this, & openPropagationSet,
                                &closedPropagationSet,
                                &activePropagationSet, 
                                &kernel,
                                &var]
                                (int i, int j, Vector2Int previousCellIndices)
    {
            Cell& previousCell = var[previousCellIndices.x][previousCellIndices.y];
            Cell& currentCell = var[i][j];
            Vector2Int ij (i,j);

            double newDistance = previousCell.distanceFromRobot +
                        ((i==previousCellIndices.x||j==previousCellIndices.y)?1:sqrt(2)); //distance of this new path to the cell
            double newWeight = applyFalloffLogOdds(previousCell.hitProbability.originalPropagationDirection * newDistance, kernel);


            //if the cell can still receive propagation
            if(closedPropagationSet.find(ij)==closedPropagationSet.end()&&
            activePropagationSet.find(ij)==activePropagationSet.end()){

                //if there already was a path to this cell
                if(openPropagationSet.find(ij)!=openPropagationSet.end()){            
                    //if the distance is the same, keep the best probability!
                    if(std::abs(newDistance-currentCell.distanceFromRobot)<0.001 && newWeight> currentCell.hitProbability.auxWeight)
                    { 
                        currentCell.hitProbability.auxWeight=newWeight;
                        currentCell.hitProbability.originalPropagationDirection = previousCell.hitProbability.originalPropagationDirection;
                    }
                    //else, keep the shortest path
                    else if(newDistance<currentCell.distanceFromRobot){ 
                        currentCell.hitProbability.auxWeight=newWeight;
                        currentCell.distanceFromRobot=newDistance;
                        currentCell.hitProbability.originalPropagationDirection = previousCell.hitProbability.originalPropagationDirection;
                    }
                }
                //if this is the first time we reach this cell
                else{
                    currentCell.hitProbability.auxWeight=newWeight;
                    currentCell.distanceFromRobot = newDistance;
                    currentCell.hitProbability.originalPropagationDirection = previousCell.hitProbability.originalPropagationDirection;
                    openPropagationSet.insert(ij);
                }
            }
    };
    
    while(!activePropagationSet.empty()){
        while(!activePropagationSet.empty()){
            Vector2Int p = *activePropagationSet.begin();

            if(var[p.x][p.y].free)
                total += var[p.x][p.y].hitProbability.auxWeight;
                
            activePropagationSet.erase(activePropagationSet.begin());
            closedPropagationSet.insert(p);

            int oR=std::max(0,p.x-1);
            int fR=std::min((int) var.size()-1,p.x+1);
            int oC=std::max(0,p.y-1);
            int fC=std::min((int) var[0].size()-1,p.y+1);
            
            //8-neighbour propagation
            for(int i=oR;i<=fR;i++){
                for(int j=oC;j<=fC;j++){
                    calculateNewAuxWeight(i,j, p);
                }
            }
        }
        
        activePropagationSet.clear();
        //the cells that were in the open set now are removed from it and can no longer receive propagation
        //therefore, the probabilities for these cells are now locked, and we can update the logodds
        for(const auto& par : openPropagationSet){

            //if the cell is free, it gets to propagate its estimations. Otherwise, even though it has received a propagation and has a probability itself, it does not propagate
            if(var[par.x][par.y].free)
                activePropagationSet.insert(par);
            else
                closedPropagationSet.insert(par);
        }

        openPropagationSet.clear();
    }
    return total;
}


double PMFS_GSL::applyFalloffLogOdds(Utils::Vector2 originalVectorScaled, const HitProbKernel& kernel){
    double sampleGaussian = Utils::evaluate2DGaussian(originalVectorScaled, kernel.sigma, kernel.angle);
    double maxPossible = Utils::evaluate2DGaussian({0,0}, kernel.sigma, kernel.angle);
    float prob = Utils::clamp(Utils::lerp(settings.hitProbability.prior, kernel.valueAt1, sampleGaussian / maxPossible), 0.001, 0.999 );
    return std::log( prob/(1-prob) );
}

}
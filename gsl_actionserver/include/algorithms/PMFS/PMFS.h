#pragma once
#include <gsl_algorithm.h>
#include <pmfs_helper_classes.h>
#include <pmfs_settings.h>
#include <pmfs_publishers_and_subscribers.h>
#include <pmfs_UI.h>
#include <pmfs_simulations.h>
#include <gsl_imgui.h>
#include <gsl_implot.h>

#include <NQAQuadtree.h>
#include <fstream>      // std::ofstream
#include <iostream>
#include <cmath>
#include <deque>
#include <bits/stdc++.h>
#include <unordered_set>
#include <std_msgs/String.h>
#include <memory>
 
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
namespace PMFS{

typedef actionlib::SimpleActionClient<navigation_assistant::nav_assistantAction> MoveBaseClient;


/*
This class is very long. 
The implementation of its methods is split over multiple files (all of the ones in the src/PMFS/pmfsGSL folder). Comments in this file indicate where to find each implementation.

Some additional functionality (and a lot of data) is stored in other structs or classes --e.g. GridSettings, Simulations-- in the src/PMFS folder
*/
class PMFS_GSL : public GSLAlgorithm
{
    public:

        //initialization
        PMFS_GSL(ros::NodeHandle *nh);
        ~PMFS_GSL();
        void initialize();

        //core
        void processGasWindObservations();
        
        //utils
        Grid_state getState();
        int checkSourceFound() override;
        void runSubmitedQueue();
        
        //movement
        virtual void setGoal();

        //visualization
        void showWeights();
        void plotWindVectors();
        void showDebugInfo();
        void renderImgui();
        void debugMapSegmentation();

        
        
            bool finished = false; //set from the server side
        struct DebugStuff{
            bool GoalSet = false;
            Vector2Int debugGoal;
            bool paused = false;
        };
        DebugStuff debugStuff;
    protected:

        Utils::Time::Clock clock;
        Grid_state previous_state, current_state;
        GridSettings settings;
        PublishersAndSubscribers pubs;

        std::vector<std::vector<Cell> > cells;
        int numFreeCells;
        std::unordered_map<Vector2Int,hashSet, Vector2Int::Vec2IntHash, Vector2Int::Vec2IntCompare> visibilityMap;
        int iterationsCounter;
        Vector2Int currentPosIndex;
        
        hashSet openMoveSet;
        hashSet closedMoveSet;

        int exploredCells;
        bool reached;
        double t1;
        double ground_truth_x, ground_truth_y;


        //-------------Core-------------
            void estimateHitProbabilities(std::vector<std::vector<Cell> >& hitLocalVariable,
                                        bool hit,
                                        double wind_direction,
                                        double wind_speed,
                                        Vector2Int robot_pos,
                                        bool infotaxis_sim = false);
            
            //returns the sum of all auxWeights, for normalization purposes
            double propagateProbabilities(std::vector<std::vector<Cell> >& var,
                                        hashSet& openSet,
                                        hashSet& closedSet,
                                        hashSet& activeSet, 
                                        const HitProbKernel& kernel); 
            double applyFalloffLogOdds(Utils::Vector2 originalVectorScaled,  const HitProbKernel& kernel);

        //-------------Movement-------------
            void moveTo(navigation_assistant::nav_assistantGoal goal);
            navigation_assistant::nav_assistantGoal indexToGoal(int i, int j);
            void cancel_navigation(bool succeeded); 
            double explorationValue(int i, int j);
            void updateSets();


        //-------------Utils-------------
            void estimateWind(bool groundTruth);
            std::vector<std::vector<Utils::Vector2> > estimatedWindVectors;
            virtual double sourceProbability(int i, int j);
            void normalizeSourceProb(std::vector<std::vector<Cell> >& variable);
            Vector2Int coordinatesToIndex(double x, double y) const;
            Utils::Vector2 indexToCoordinates(int i, int j, bool centerOfCell=true) const;
            bool pathFree(const Vector2Int& origin, const Vector2Int& end);
            bool indicesInBounds(const Vector2Int indices) const;

            //Termination condition
            std::vector<SourceEstimationError> errorOverTimeAll;
            std::vector<SourceEstimationError> errorOverTime;
            Utils::Vector2 expectedValueSource();
            Utils::Vector2 expectedValueSource(double proportionBest);
            double varianceSourcePosition();
            
            std::deque<Utils::Vector2> estimatedSourceLocations;
            void save_results_to_file(int result, double i, double j, double allI, double allJ);

            std::mutex mainThreadQueueMtx;
            std::vector<std::function<void()> > mainThreadExecQueue;
            void submitToMainThread(const std::function<void()>& func);



        //---------------Callbacks--------------
            void gasCallback(const olfaction_msgs::gas_sensorPtr& msg) override;
            void windCallback(const olfaction_msgs::anemometerPtr& msg) override;
            void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) override;
            void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const navigation_assistant::nav_assistantResultConstPtr &result) override;

                

            //Measurements
            float last_concentration_reading=-1;
            ros::Time time_stopped; 
            std::vector<float> stop_and_measure_gas_v;
            std::vector<float> stop_and_measure_windS_v;
            std::vector<float> stop_and_measure_windD_v;
            AveragedMeasurement getAveragedMeasurement();


        //---------------Visualization--------------
            void createUI();
            void createPlots();


        //the simulation thing
        friend class SimulationSource;
        friend class Simulations;
        Simulations simulations;
};

}
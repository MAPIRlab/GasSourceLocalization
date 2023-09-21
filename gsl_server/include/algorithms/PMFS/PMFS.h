#pragma once
#include <algorithms/gsl_algorithm.h>
#include <algorithms/PMFS/pmfs_helper_classes.h>
#include <algorithms/PMFS/pmfs_settings.h>
#include <algorithms/PMFS/pmfs_publishers_and_subscribers.h>
#include <algorithms/PMFS/pmfs_UI.h>
#include <algorithms/PMFS/pmfs_simulations.h>

#include <NQA/NQAQuadtree.h>
#include <fstream> // std::ofstream
#include <iostream>
#include <cmath>
#include <deque>
#include <bits/stdc++.h>
#include <unordered_set>
#include <std_msgs/msg/string.hpp>
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
namespace PMFS
{

	/*
	This class is very long.
	The implementation of its methods is split over multiple files (all of the ones in the src/PMFS/pmfsGSL folder). Comments in this file indicate where to find each implementation.

	Some additional functionality (and a lot of data) is stored in other structs or classes --e.g. GridSettings, Simulations-- in the src/PMFS folder
	*/
	class PMFS_GSL : public GSLAlgorithm
	{
	public:
		// initialization
		PMFS_GSL(std::shared_ptr<rclcpp::Node> _node);
		~PMFS_GSL();
		virtual void initialize() override;
		void setUpMaps();

		// core
		void processGasWindObservations();

		// utils
		State getState();
		int checkSourceFound() override;
		void runSubmitedQueue();

		// movement
		virtual void setGoal();

		// visualization
		void showWeights();
		void plotWindVectors();
		void showDebugInfo();
		void renderImgui();
		void debugMapSegmentation();

		bool finished = false; // set from the server side
		struct DebugStuff
		{
			bool GoalSet = false;
			Vector2Int debugGoal;
			bool paused = false;
		};
		DebugStuff debugStuff;

	protected:
		Utils::Time::Clock clock;
		State previous_state, current_state;
		GridSettings settings;
		PublishersAndSubscribers pubs;

		std::vector<std::vector<Cell>> cells;
		int numFreeCells;
		std::unordered_map<Vector2Int, hashSet, Vector2Int::Vec2IntHash, Vector2Int::Vec2IntCompare> visibilityMap;
		int iterationsCounter;
		Vector2Int currentPosIndex;

		hashSet openMoveSet;
		hashSet closedMoveSet;

		int exploredCells;
		bool reached;
		double t1;
		double ground_truth_x, ground_truth_y;

		//-------------Initialization-------------
		void declareParameters() override;

		//-------------Core-------------
		void estimateHitProbabilities(std::vector<std::vector<Cell>>& hitLocalVariable,
			bool hit,
			double wind_direction,
			double wind_speed,
			Vector2Int robot_pos,
			bool infotaxis_sim = false);

		// returns the sum of all auxWeights, for normalization purposes
		double propagateProbabilities(std::vector<std::vector<Cell>>& var,
			hashSet& openSet,
			hashSet& closedSet,
			hashSet& activeSet,
			const HitProbKernel& kernel);
		double applyFalloffLogOdds(Utils::Vector2 originalVectorScaled, const HitProbKernel& kernel);

		//-------------Movement-------------
		void moveTo(NavAssistant::Goal& goal);
		NavAssistant::Goal indexToGoal(int i, int j);
		void cancel_navigation(bool succeeded);
		double explorationValue(int i, int j);
		void updateSets();

		//-------------Utils-------------
		void estimateWind(bool groundTruth);
		std::vector<std::vector<Utils::Vector2>> estimatedWindVectors;
		virtual double sourceProbability(int i, int j);
		void normalizeSourceProb(std::vector<std::vector<Cell>>& variable);
		Vector2Int coordinatesToIndex(double x, double y) const;
		Utils::Vector2 indexToCoordinates(int i, int j, bool centerOfCell = true) const;
		bool pathFree(const Vector2Int& origin, const Vector2Int& end);
		bool indicesInBounds(const Vector2Int indices) const;

		// Termination condition
		std::vector<SourceEstimationError> errorOverTimeAll;
		std::vector<SourceEstimationError> errorOverTime;
		Utils::Vector2 expectedValueSource();
		Utils::Vector2 expectedValueSource(double proportionBest);
		double varianceSourcePosition();

		std::deque<Utils::Vector2> estimatedSourceLocations;
		void save_results_to_file(int result, double i, double j, double allI, double allJ);

		std::mutex mainThreadQueueMtx;
		std::vector<std::function<void()>> mainThreadExecQueue;
		void submitToMainThread(const std::function<void()>& func);

		//---------------Callbacks--------------
		void gasCallback(const olfaction_msgs::msg::GasSensor::SharedPtr msg) override;
		void windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg) override;
		void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) override;
		void goalDoneCallback(const rclcpp_action::ClientGoalHandle<NavAssistant>::WrappedResult& result) override;

		// Measurements
		float last_concentration_reading = -1;
		rclcpp::Time time_stopped;
		std::vector<float> stop_and_measure_gas_v;
		std::vector<float> stop_and_measure_windS_v;
		std::vector<float> stop_and_measure_windD_v;
		AveragedMeasurement getAveragedMeasurement();

		//---------------Visualization--------------
		void createUI();
		void createPlots();

		// the simulation thing
		friend class SimulationSource;
		friend class Simulations;
		Simulations simulations;
	};

}
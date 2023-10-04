#pragma once

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <eigen3/Eigen/Dense>
#include <numeric>
#include <random>
#include <algorithms/gsl_surge_spiral.h>

class Particle
{
public:
    Particle(double x, double y, double weight);
    ~Particle();
    double x, y, weight;

private:
};

class ParticleFilter : public SurgeSpiralPT
{
public:
    ParticleFilter(std::shared_ptr<rclcpp::Node> _node);
    ~ParticleFilter();
    virtual void initialize() override;

    int numberOfParticles; // how many particles we are going to generate
    int maxEstimations;    // how many estimations we want to keep track of
    int numberOfWindObs;
    FILE* log;
    double convergenceThr; // distance at which we consider the estimations have converged
    double Rconv;          // convergence threshold for the particles
    double mu;             // sensor sensibility
    double Sp;             // area covered by the source
    bool firstObserv;

    std::vector<Particle> particles;                // current set of particles
    std::vector<Utils::Vector2> estimatedLocations; // record of estimated positions
    std::vector<Utils::Vector2> allEstimations;     // record of estimated positions
    std::vector<Utils::Vector2> historicWind;       // record of measured wind
    rclcpp::Time lastWindObservation;

    rclcpp::Publisher<Marker>::SharedPtr particle_markers;
    rclcpp::Publisher<Marker>::SharedPtr estimation_markers;
    rclcpp::Publisher<Marker>::SharedPtr average_estimation_marker;

    // plume-tracking logic
    void windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg) override;
    void checkState() override;

    int iterationsToConverge;

    // aux functions
    bool cellIsFree(double x, double y);
    Utils::Vector2 average_vector(std::vector<Utils::Vector2>& data);
    Marker emptyMarker();
    Utils::Vector2 standardDeviationWind(int first);
    Utils::Vector2 sourceLocalizationEstimation();

    // particle filter
    void estimateLocation();
    void updateWeights(bool hit);
    bool isDegenerated();
    void resample();
    double probability(bool hit, Particle& particle);
    void generateParticles(Marker& points);
    bool particlesConverge();
    int checkSourceFound() override;
    Eigen::Vector3d valueToColor(double val);

    void declareParameters() override;
};
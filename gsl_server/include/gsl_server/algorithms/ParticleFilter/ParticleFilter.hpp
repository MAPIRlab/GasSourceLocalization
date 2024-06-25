#pragma once
#include <gsl_server/algorithms/PlumeTracking/SurgeSpiral/SurgeSpiral.hpp>
#include <deque>

namespace GSL
{
    class ParticleFilter : public SurgeSpiral
    {
    public:
        ParticleFilter(std::shared_ptr<rclcpp::Node> _node) : SurgeSpiral(_node)
        {
        }
        ~ParticleFilter()
        {
        }

        void Initialize() override;
        void declareParameters() override;

    protected:
        PoseStamped windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg);

    private:
        struct Particle
        {
            double x, y, weight;
            Particle(double _x, double _y, double _weight) : x(_x), y(_y), weight(_weight)
            {
            }
        };

        std::vector<Particle> particles;

        int numberOfParticles;
        int maxEstimations;
        int numberOfWindObs;
        double convergenceThr;
        double deltaT;

        double mu;
        double Sp;
        double Rconv;

        std::vector<Vector2> estimatedLocations; // record of estimated source positions over time
        std::deque<Vector2> historicWind;        // record of measured wind

        rclcpp::Publisher<Marker>::SharedPtr particle_markers;
        rclcpp::Publisher<Marker>::SharedPtr estimationMarkers;
        rclcpp::Publisher<Marker>::SharedPtr average_estimation_marker;

        void processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection);
        void generateParticles();

        void estimateSourceLocation();
        void updateWeights(bool hit);
        bool isDegenerated();
        void resample();
        double probability(bool hit, Particle& particle);
        bool particlesConverge();

        Vector2 standardDeviationWind(int first);
        void publishMarkers();
    };
} // namespace GSL
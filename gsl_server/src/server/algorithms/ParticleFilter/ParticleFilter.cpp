#include <gsl_server/algorithms/ParticleFilter/ParticleFilter.h>
#include <gsl_server/Utils/Math.h>
#include <gsl_server/Utils/RosUtils.h>
#include <gsl_server/core/Vectors.h>

namespace GSL
{
    void ParticleFilter::initialize()
    {
        SurgeSpiral::initialize();

        particle_markers = node->create_publisher<Marker>("particle_markers", 10);
        estimation_markers = node->create_publisher<Marker>("estimation_markers", 10);
        average_estimation_marker = node->create_publisher<Marker>("average_estimation_marker", 10);
    }

    void ParticleFilter::declareParameters()
    {
        SurgeSpiral::declareParameters();
        numberOfParticles = getParam<int>("numberOfParticles", 500);
        maxEstimations = getParam<int>("maxEstimations", 20);
        numberOfWindObs = getParam<int>("numberOfWindObs", 30);
        convergenceThr = getParam<double>("convergenceThr", 0.5);
        deltaT = getParam<double>("deltaT", 1);

        mu = getParam<double>("mu", 0.9);
        Sp = getParam<double>("Sp", 0.01);
        Rconv = getParam<double>("Rconv", 0.5);
    }

    PoseStamped ParticleFilter::windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg)
    {
        static rclcpp::Time lastWindObservation(0);
        static std::vector<float> windSpeed_v;
        static std::vector<float> windDirection_v;

        PoseStamped downwind_map = SurgeSpiral::windCallback(msg);
        windSpeed_v.push_back(msg->wind_speed);
        windDirection_v.push_back(Utils::getYaw(downwind_map.pose.orientation));

        if (node->now().seconds() - lastWindObservation.seconds() >= deltaT)
        {
            lastWindObservation = node->now();
            // store the measurement in the list of wind history as a (x,y) vector
            double speed = Utils::getAverageVector(windSpeed_v);
            double angle = Utils::getAverageDirection(windDirection_v);
            historicWind.push_back(Vector2(speed * cos(angle), speed * sin(angle)));

            if (historicWind.size() > numberOfWindObs)
                historicWind.pop_front();

            windSpeed_v.clear();
            windDirection_v.clear();
        }

        return downwind_map;
    }

    void ParticleFilter::processGasAndWindMeasurements(double concentration, double wind_speed, double wind_direction)
    {
        SurgeSpiral::processGasAndWindMeasurements(concentration, wind_speed, wind_direction);

        if (particles.size() == 0)
            generateParticles();

        if (particlesConverge())
            estimateSourceLocation();
        updateWeights(true);
        if (isDegenerated())
            resample();

        publishMarkers();
    }

    void ParticleFilter::generateParticles()
    {
        std::vector<Vector2> stDev;
        std::vector<Vector2> estSourcePos;
        estSourcePos.push_back(Vector2(current_robot_pose.pose.pose.position.x, current_robot_pose.pose.pose.position.y));
        stDev.push_back(Vector2(0.1, 0.1));
        for (int i = historicWind.size(); i >= 0; i--)
        {
            stDev.push_back(standardDeviationWind(i));
            estSourcePos.push_back(estSourcePos.back() - historicWind[i] * (float)deltaT);
        }

        while (particles.size() < numberOfParticles)
        {
            int i = Utils::uniformRandom(0, estSourcePos.size()); // choose which OS to use with uniform probability

            // generate a point in that OS according to a gaussian
            double x = Utils::randomFromGaussian(estSourcePos[i].x, stDev[i].x);
            double y = Utils::randomFromGaussian(estSourcePos[i].y, stDev[i].y);

            if (isPositionFree({x, y}))
                particles.emplace_back(x, y, 0);
        }
    }

    void ParticleFilter::updateWeights(bool hit)
    {
        double sum = 0;
        for (int i = 0; i < particles.size(); i++)
        {
            double prob = probability(hit, particles[i]);
            particles[i].weight = particles[i].weight * prob;
            sum += particles[i].weight;
        }
        for (int i = 0; i < particles.size(); i++)
        {
            particles[i].weight = particles[i].weight / sum;
        }
    }

    void ParticleFilter::estimateSourceLocation()
    {
        double averageX = 0;
        double averageY = 0;

        for (const Particle& p : particles)
        {
            averageX += p.weight * p.x;
            averageY += p.weight * p.y;
        }

        if (!std::isnan(averageX) && !std::isnan(averageY))
        {
            estimatedLocations.push_back(Vector2(averageX, averageY));
            if (estimatedLocations.size() > maxEstimations)
                estimatedLocations.erase(estimatedLocations.begin());
        }
    }

    bool ParticleFilter::isDegenerated()
    {
        double neff = 0;
        double sum = 0;
        for (const Particle& p : particles)
        {
            neff += std::pow(p.weight, 2);
            sum += p.weight;
        }
        double effectiveP = 1.0 / neff;
        GSL_INFO("Effective particles {} Total particles: {}", std::floor(effectiveP), particles.size());

        return effectiveP < particles.size() * 0.25 || std::isnan(effectiveP);
    }

    void ParticleFilter::resample()
    {
        GSL_WARN("[Particle Filter] Resampling");
        std::vector<Particle> newParticles;

        for (int i = 0; i < particles.size(); i++)
        {
            if (newParticles.size() >= 2 * numberOfParticles)
                break;

            if (particles[i].weight > (1.0 / particles.size()))
            {
                // if the particle is good, use it to generate new ones
                int numberToSpawn = std::min(5, (int)(particles[i].weight * particles.size()));
                int count = 0;
                while (count < numberToSpawn)
                {
                    double x = Utils::randomFromGaussian(0, 1) + particles[i].x;
                    double y = Utils::randomFromGaussian(0, 1) + particles[i].y;
                    if (isPositionFree({x, y}))
                    {
                        newParticles.emplace_back(x, y, 0);
                        count++;
                    }
                }
            }
            // if the particle is bad, *maybe* kill it
            else if (rand() % 3 == 0)
            {
                newParticles.push_back(particles[i]);
            }
        }

        particles.clear();
        particles.swap(newParticles);

        // refill up to the desired number of particles with random ones if needed
        generateParticles();

        double w = 1.0 / particles.size();
        for (Particle& p : particles)
            p.weight = w;
    }

    double ParticleFilter::probability(bool hit, Particle& particle)
    {
        double total = 1;
        Vector2 stDev = standardDeviationWind(0);

        for (int windIndex = 0; windIndex < historicWind.size(); windIndex++)
        {
            double sx = 0, sy = 0;
            for (int j = windIndex; j < historicWind.size(); j++)
            {
                sx += historicWind[j].x * deltaT;
                sy += historicWind[j].y * deltaT;
            }

            double deltaIX = current_robot_pose.pose.pose.position.x - particle.x - sx;
            double deltaIY = current_robot_pose.pose.pose.position.y - particle.y - sy;
            double t1 = exp(-pow(deltaIX, 2) / (2 * pow(stDev.x, 2) * (historicWind.size() - windIndex) * deltaT));
            double t2 = exp(-pow(deltaIY, 2) / (2 * pow(stDev.y, 2) * (historicWind.size() - windIndex) * deltaT));

            double density = 1.0 / (2 * M_PI * stDev.x * stDev.y * (historicWind.size() - windIndex) * deltaT) * t1 * t2;
            total = total * (1 - mu * Sp * density);
        }

        if (hit)
            return (1 - total);
        else
            return total;
    }

    Vector2 ParticleFilter::standardDeviationWind(int first)
    {
        int n = historicWind.size();
        Vector2 average;
        {
            double sumX = 0;
            double sumY = 0;
            for (Vector2& vec : historicWind)
            {
                sumX += vec.x;
                sumY += vec.y;
            }
            average = Vector2(sumX / n, sumY / n);
        }

        double sumX = 0, sumY = 0;
        for (int i = first; i < historicWind.size(); i++)
        {
            sumX += pow(historicWind[i].x - average.x, 2);
            sumY += pow(historicWind[i].y - average.y, 2);
        }
        return Vector2(sqrt(sumX / n), sqrt(sumY / n));
    }

    bool ParticleFilter::particlesConverge()
    {
        Vector2 average(0, 0);

        for (const Particle& p : particles)
        {
            average.x += p.weight * p.x;
            average.y += p.weight * p.y;
        }
        double var = 0;
        for (const Particle& p : particles)
        {
            var += std::pow(glm::length(Vector2(p.x, p.y) - average), 2) * p.weight;
        }
        // return true;
        return std::sqrt(var) <= Rconv;
    }

    void ParticleFilter::publishMarkers()
    {
        {
            Marker parts;
            parts.header.frame_id = "map";
            parts.header.stamp = node->now();
            parts.ns = "particles";
            parts.id = 0;
            parts.type = Marker::POINTS;
            parts.action = Marker::ADD;
            parts.scale.x = 0.1;
            parts.scale.y = 0.1;
            for (const Particle& p : particles)
            {
                Point point;
                point.x = p.x;
                point.y = p.y;

                std_msgs::msg::ColorRGBA color = Utils::valueToColor(p.weight, -10, 0, Utils::valueColorMode::Logarithmic);

                parts.points.push_back(point);
                parts.colors.push_back(color);
            }
            particle_markers->publish(parts);
        }

        {

            Marker estimation;
            estimation.header.frame_id = "map";
            estimation.header.stamp = node->now();
            estimation.ns = "estimations";
            estimation.id = 1;
            estimation.type = Marker::POINTS;
            estimation.action = Marker::ADD;
            estimation.color.b = 1.0;
            estimation.color.a = 1.0;
            estimation.scale.x = 0.1;
            estimation.scale.y = 0.1;

            for (const Vector2& est : estimatedLocations)
            {
                Point p;
                p.x = est.x;
                p.y = est.y;
                estimation.points.push_back(p);
            }
            estimation_markers->publish(estimation);
        }
    }
} // namespace GSL
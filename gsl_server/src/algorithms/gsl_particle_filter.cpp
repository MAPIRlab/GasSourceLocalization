#include "algorithms/gsl_particle_filter.h"

Particle::Particle(double a, double b, double c)
{
	x = a;
	y = b, weight = c;
}

Particle::~Particle()
{
}

ParticleFilter::ParticleFilter(std::shared_ptr<rclcpp::Node> _node) : SurgeSpiralPT(_node)
{}

ParticleFilter::~ParticleFilter() {}

void ParticleFilter::initialize()
{
	SurgeSpiralPT::initialize();

	particle_markers = node->create_publisher<Marker>("particle_markers", 10);
	estimation_markers = node->create_publisher<Marker>("estimation_markers", 10);
	average_estimation_marker = node->create_publisher<Marker>("average_estimation_marker", 10);

	lastWindObservation = node->now();
	windDirection_v.clear();
	windSpeed_v.clear();
	firstObserv = false;
	iterationsToConverge = 0;
}

void ParticleFilter::declareParameters()
{
	SurgeSpiralPT::declareParameters();
	numberOfParticles = getParam<int>("numberOfParticles", 500);
	maxEstimations = getParam<int>("maxEstimations", 20);
	numberOfWindObs = getParam<int>("numberOfWindObs", 30);
	convergenceThr = getParam<double>("convergenceThr", 0.5);
	deltaT = getParam<double>("deltaT", 1);
	mu = getParam<double>("mu", 0.9);
	Sp = getParam<double>("Sp", 0.01);
	Rconv = getParam<double>("Rconv", 0.5);
}

//------------------------------------

// OVERRIDEN PLUME-TRACKING LOGIC

//------------------------------------

void ParticleFilter::windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg)
{

	windSpeed_v.push_back(msg->wind_speed);

	float downWind_direction = angles::normalize_angle(msg->wind_direction + M_PI);
	// Transform from anemometer ref_system to map ref_system using TF
	geometry_msgs::msg::PoseStamped anemometer_downWind_pose, map_downWind_pose;
	try
	{
		anemometer_downWind_pose.header.frame_id = msg->header.frame_id;
		anemometer_downWind_pose.pose.position.x = 0.0;
		anemometer_downWind_pose.pose.position.y = 0.0;
		anemometer_downWind_pose.pose.position.z = 0.0;
		anemometer_downWind_pose.pose.orientation = Utils::createQuaternionMsgFromYaw(downWind_direction);

		map_downWind_pose = tf_buffer->transform(anemometer_downWind_pose, "map");
	}
	catch (tf2::TransformException& ex)
	{
		spdlog::error("[Particle_Filter] - {} - Error: {}", __FUNCTION__, ex.what());
		return;
	}

	windDirection_v.push_back(Utils::getYaw(map_downWind_pose.pose.orientation));

	// Only if we are in the Stop_and_Measure
	if (this->current_state == PT_state::STOP_AND_MEASURE)
	{
		stop_and_measure_windS_v.push_back(msg->wind_speed);
		stop_and_measure_windD_v.push_back(Utils::getYaw(map_downWind_pose.pose.orientation));
	}

	if (node->now().seconds() - lastWindObservation.seconds() >= deltaT)
	{
		// store the measurement in the list of wind history as a (x,y) vector
		double length = (node->now().seconds() - lastWindObservation.seconds()) / deltaT;
		lastWindObservation = node->now();
		double speed = get_average_vector(windSpeed_v) * length;
		double angle = get_average_wind_direction(windDirection_v);
		historicWind.push_back(Utils::Vector2(speed * cos(angle), speed * sin(angle)));

		if (historicWind.size() > numberOfWindObs)
		{
			historicWind.erase(historicWind.begin());
		}
		windSpeed_v.clear();
		windDirection_v.clear();
	}
}

void ParticleFilter::checkState()
{
	// If we are here, that means we are moving towards a target location reactively
	switch (current_state)
	{
	case PT_state::EXPLORATION:
		// We are looking for gas clues
		if (get_average_vector(gasConcentration_v) > th_gas_present)
		{
			if (verbose)
				spdlog::info("GAS HIT!");
			gasHit = true;
			cancel_navigation(); // Stop Robot
			previous_state = PT_state::EXPLORATION;
			current_state = PT_state::STOP_AND_MEASURE;
			if (verbose)
				spdlog::warn("[GSL-ParticleFilter] New state --> STOP_AND_MEASURE");
		}
		break;
	case PT_state::INSPECTION:
		// Check surroundings of current position
		//  No break conditions, wait till end of inspection
		break;
	case PT_state::UPWIND_SURGE:
		// We are moving within the gas plume
		if (node->now().seconds() - lastUpdateTimestamp.seconds() >= deltaT)
		{
			if (gasConcentration_v.back() >= th_gas_present)
			{
				// if we haven't moved in 3 seconds, the path might not be good, resample everything
				if (node->now().seconds() - recoveryTimestamp.seconds() >= 3)
				{
					current_state = PT_state::STOP_AND_MEASURE;
					cancel_navigation();
					if (verbose)
						spdlog::warn("[GSL-SurgeSpiral] New state --> STOP_AND_MEASURE");
					return;
				}
				lastUpdateTimestamp = node->now();

				double dist = sqrt(pow(current_robot_pose.pose.pose.position.x - movingPose.pose.pose.position.x, 2) +
					pow(current_robot_pose.pose.pose.position.y - movingPose.pose.pose.position.y, 2));

				// only send new goals it we are already moving
				if (dist > 0.1)
				{
					if (verbose)
						spdlog::warn("[GSL-SurgeSpiral] More gas! Surge distance has been reset");
					setSurgeGoal();
					movingTimestamp = node->now();
					movingPose = current_robot_pose;
					recoveryTimestamp = node->now();
				}

				updateWeights(true);
				estimateLocation();
				if (isDegenerated())
				{
					resample();
				}
			}
			else
			{
				updateWeights(false);
				if (isDegenerated())
				{
					resample();
				}
				lastUpdateTimestamp = node->now();
			}
		}
		break;
	case PT_state::CROSSWIND_CAST:
		// We are trying to return to the plume
		if (get_average_vector(gasConcentration_v) > th_gas_present)
		{
			if (verbose)
				spdlog::info("Gas plume found! - Returning to UPWIND_SURGE movement!");
			gasHit = true;
			cancel_navigation(); // Stop Robot
			previous_state = PT_state::CROSSWIND_CAST;
			current_state = PT_state::STOP_AND_MEASURE;
			resetSpiral();
			if (verbose)
				spdlog::warn("[GSL-ParticleFilter] New state --> STOP_AND_MEASURE");
		}
		else
		{
			if (node->now().seconds() - lastUpdateTimestamp.seconds() >= deltaT)
			{
				updateWeights(false);
				if (isDegenerated())
				{
					resample();
				}
				lastUpdateTimestamp = node->now();
			}
		}
		break;
	default:
		current_state = PT_state::STOP_AND_MEASURE;
		cancel_navigation();
		spdlog::error("ERROR: State undefined!");
	}
}

//-----------------------------

// AUXILIARY FUNCTIONS

//-----------------------------

Utils::Vector2 ParticleFilter::average_vector(std::vector<Utils::Vector2>& data)
{
	double sumX = 0;
	double sumY = 0;
	for (Utils::Vector2& vec : data)
	{
		sumX += vec.x;
		sumY += vec.y;
	}
	return Utils::Vector2(sumX / data.size(), sumY / data.size());
}

bool ParticleFilter::cellIsFree(double x, double y)
{
	if (x < map_.info.origin.position.x ||
		x >= map_.info.origin.position.x + map_.info.width * map_.info.resolution ||
		y < map_.info.origin.position.y ||
		y >= map_.info.origin.position.y + map_.info.height * map_.info.resolution)
	{
		return false;
	}

	int h = (x - map_.info.origin.position.x) / map_.info.resolution;
	int v = (y - map_.info.origin.position.y) / map_.info.resolution;

	return map_.data[v * map_.info.width + h] == 0;
}

Utils::Vector2 ParticleFilter::standardDeviationWind(int first)
{
	int n = historicWind.size();
	Utils::Vector2 average = average_vector(historicWind);
	double sumX = 0, sumY = 0;
	for (int i = first; i < historicWind.size(); i++)
	{
		sumX += pow(historicWind[i].x - average.x, 2);
		sumY += pow(historicWind[i].y - average.y, 2);
	}
	return Utils::Vector2(sqrt(sumX / n), sqrt(sumY / n));
}

Marker ParticleFilter::emptyMarker()
{
	Marker points;
	points.header.frame_id = "map";
	points.header.stamp = node->now();
	points.ns = "particles";
	points.id = 0;
	points.type = Marker::POINTS;
	points.action = Marker::ADD;

	Eigen::Vector3d colour = valueToColor(1.0 / numberOfParticles);
	points.color.r = colour[0];
	points.color.g = colour[1];
	points.color.b = colour[2];
	points.color.a = 1.0;
	points.scale.x = 0.1;
	points.scale.y = 0.1;
	return points;
}

Eigen::Vector3d ParticleFilter::valueToColor(double val)
{
	double r, g, b;
	double logaritmo = log10(val);
	if (logaritmo < -6.6)
	{
		r = 0;
		g = 1;
		b = 1 - std::max<double>((logaritmo + 10) / 3.3, 0);
	}
	else if (logaritmo < -3.3)
	{
		r = (logaritmo + 6.6) / 3.3;
		g = 1;
		b = 0;
	}
	else
	{
		r = 1;
		g = 1 - (logaritmo + 3.3) / 3.3;
		b = 0;
	}
	return Eigen::Vector3d(r, g, b);
}

//-----------------------------

// PARTICLE FILTER

//-----------------------------

void ParticleFilter::generateParticles(Marker& points)
{

	std::random_device rd;
	std::mt19937 gen(rd());
	std::normal_distribution<double> dist(0, 1);

	std::vector<Utils::Vector2> stDev;
	std::vector<Utils::Vector2> estSourcePos;
	estSourcePos.push_back(Utils::Vector2(current_robot_pose.pose.pose.position.x, current_robot_pose.pose.pose.position.y));
	stDev.push_back(Utils::Vector2(0.1, 0.1));
	for (int i = historicWind.size(); i >= 0; i--)
	{
		stDev.push_back(standardDeviationWind(i));
		estSourcePos.push_back(estSourcePos.back() - historicWind[i] * deltaT);
	}

	while (particles.size() < numberOfParticles)
	{                                           // Keep generating until we get the desired amount of particles
		int i = rand() % (estSourcePos.size()); // choose which OS to use with uniform probability
		Point p;
		p.x = estSourcePos[i].x + dist(gen) * stDev[i].x; // generate a point in that OS according to a gaussian
		p.y = estSourcePos[i].y + dist(gen) * stDev[i].y;
		p.z = 0;

		if (cellIsFree(p.x, p.y))
		{ // no particles outside of the map nor inside obstacles
			points.points.push_back(p);
			particles.push_back(Particle(p.x, p.y, 0));
		}
	}
	particle_markers->publish(points);
}

double ParticleFilter::probability(bool hit, Particle& particle)
{
	double total = 1;
	Utils::Vector2 stDev = standardDeviationWind(0);

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

	std::sort(particles.begin(), particles.end(), [](Particle p1, Particle p2)
		{ return p1.weight > p2.weight; });

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
		Eigen::Vector3d col = valueToColor(p.weight);
		std_msgs::msg::ColorRGBA color;
		color.a = 1;
		color.r = col[0];
		color.g = col[1];
		color.b = col[2];

		parts.points.push_back(point);
		parts.colors.push_back(color);
	}
	particle_markers->publish(parts);
}

bool ParticleFilter::isDegenerated()
{
	iterationsToConverge++;
	double neff = 0;
	double sum = 0;
	for (const Particle& p : particles)
	{
		neff += std::pow(p.weight, 2);
		sum += p.weight;
	}
	double effectiveP = 1.0 / neff;
	spdlog::info("Effective particles {} Total particles: {}", std::floor(effectiveP), particles.size());

	return effectiveP < particles.size() * 0.25 || std::isnan(effectiveP);
}

void ParticleFilter::resample()
{
	if (verbose)
		spdlog::warn("[Particle Filter] Resampling");
	std::vector<Particle> newParticles;

	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::normal_distribution<double> dist(0.0, 0.05);
	Marker points = emptyMarker();

	for (int i = 0; i < particles.size(); i++)
	{
		if (newParticles.size() >= 2 * numberOfParticles)
		{
			break;
		}
		if (particles[i].weight > (1.0 / particles.size()))
		{ // if the particle is good, use it to generate new ones
			int n = std::min(5, (int)(particles[i].weight * particles.size()));
			int count = 0;
			while (count < n)
			{
				Point p;
				p.x = dist(gen) + particles[i].x;
				p.y = dist(gen) + particles[i].y;
				if (cellIsFree(p.x, p.y))
				{
					points.points.push_back(p);
					newParticles.push_back(Particle(p.x, p.y, 0));
					count++;
				}
			}
		}
		else if (rand() % 3 == 0)
		{ // if the particle is bad, *maybe* kill it
			Point p;
			p.x = particles[i].x;
			p.y = particles[i].y;
			newParticles.push_back(particles[i]);
			points.points.push_back(p);
		}
	}

	particles.clear();
	particles.swap(newParticles);
	generateParticles(points);

	double w = 1.0 / particles.size();
	for (Particle& p : particles)
	{
		p.weight = w;
	}
}

bool ParticleFilter::particlesConverge()
{
	Utils::Vector2 average(0, 0);

	for (const Particle& p : particles)
	{
		average.x += p.weight * p.x;
		average.y += p.weight * p.y;
	}
	double var = 0;
	for (const Particle& p : particles)
	{
		var += pow((Utils::Vector2(p.x, p.y) - average).norm(), 2) * p.weight;
	}
	return true; // sqrt(var)<=Rconv;
}

void ParticleFilter::estimateLocation()
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

	double averageX = 0;
	double averageY = 0;

	for (const Particle& p : particles)
	{
		averageX += p.weight * p.x;
		averageY += p.weight * p.y;
	}

	if (!std::isnan(averageX) && !std::isnan(averageY))
	{
		estimatedLocations.push_back(Utils::Vector2(averageX, averageY));
		allEstimations.push_back(Utils::Vector2(averageX, averageY));
		if (estimatedLocations.size() > maxEstimations)
		{
			estimatedLocations.erase(estimatedLocations.begin());
		}
	}

	for (const Utils::Vector2& est : estimatedLocations)
	{
		Point p;
		p.x = est.x;
		p.y = est.y;
		estimation.points.push_back(p);
	}
	estimation_markers->publish(estimation);
}

Utils::Vector2 ParticleFilter::sourceLocalizationEstimation()
{
	return average_vector(estimatedLocations);
}

int ParticleFilter::checkSourceFound()
{
	if (inExecution)
	{
		if (estimatedLocations.size() < maxEstimations)
		{
			return -1;
		}
		// 1. Check that working time < max allowed time for search
		rclcpp::Duration time_spent = node->now() - start_time;
		if (time_spent.seconds() > max_search_time)
		{
			// Report failure, we were too slow
			spdlog::info("- FAILURE-> Time spent ({} s) > max_search_time = {}", time_spent.seconds(), max_search_time);
			save_results_to_file(0);
			return 0;
		}

		Utils::Vector2 average = average_vector(estimatedLocations);
		Marker estimation;
		estimation.header.frame_id = "map";
		estimation.header.stamp = node->now();
		estimation.ns = "average";
		estimation.id = 2;
		estimation.type = Marker::POINTS;
		estimation.action = Marker::ADD;
		estimation.color.g = 1.0;
		estimation.color.a = 1.0;
		estimation.scale.x = 0.1;
		estimation.scale.y = 0.1;
		Point p;
		p.x = average.x, p.y = average.y;

		estimation.points.push_back(p);
		average_estimation_marker->publish(estimation);

		for (const Utils::Vector2& vec : estimatedLocations)
		{
			if ((vec - average).norm() > convergenceThr)
			{
				return -1;
			}
		}

		double dist = sqrt(pow(p.x - source_pose_x, 2) + pow(p.y - source_pose_y, 2));
		if (dist < distance_found && cellIsFree(p.x, p.y))
		{
			// GSL has finished with success!
			spdlog::info("- SUCCESS -> Time spent ({} s)", time_spent.seconds());
			save_results_to_file(1);
			return 1;
		}
	}

	// In other case, we are still searching (keep going)
	return -1;
}

#include "gsl_server/algorithms/Common/Utils/RosUtils.hpp"
#include "gsl_server/core/Logging.hpp"
#include "gsl_server/core/Macros.hpp"
#include <angles/angles.h>
#include <chrono>
#include <cmath> // For atan2 and M_PI
#include <filesystem>
#include <fmt/color.h>
#include <fstream>
#include <gsl_server/algorithms/Common/Algorithm.hpp>
#include <gsl_server/algorithms/Common/StopAndMeasureState.hpp>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

namespace GSL
{
    StopAndMeasureState::StopAndMeasureState(Algorithm* _algorithm)
        : State(_algorithm)
    {
        measure_time = algorithm->getParam<double>("stop_and_measure_time", 2.0);
    }

    void StopAndMeasureState::OnEnterState(State* previous)
    {
        GSL_TRACE("Entering StopAndMeasure");
        time_stopped = algorithm->node->now();
        gas_v.clear();
        windSpeed_v.clear();
        windDirection_v.clear();
    }

    std::string intToStringWithLeadingZeros(int num, int width = 3)
    {
        std::stringstream ss;
        ss << std::setw(width) << std::setfill('0') << num;
        return ss.str();
    }

    // Function to calculate wind direction
    double calculateWindDirection(double wind_u, double wind_v)
    {
        // Down-Wind angle
        double angle = atan2(wind_v, wind_u) + M_PI;
        return angles::normalize_angle(angle);
    }

    void StopAndMeasureState::OnUpdate()
    {
        // 1. For each batch-i folder with data (there are 100 batches)
        //      2. For each test-sample-j file (there are 500 samples)
        //          3. Load all data points (usually from 4 to 6), and update maps
        //          4. If rem(j,10)==0, estimar posición de fuente
        GSL_TRACE("Entering StopAndMeasure::OnUpdate");
        int total_batches = 10;      // Total number of batches
        int samples_per_batch = 500; // Total number of samples per batch

        // Launch params
        std::string test_folder_path = Utils::getParam<std::string>(algorithm->node, "test_folder", "/mnt/d/Projects/2024_GSL_Challenge_IEEE_ICASSP/train/test");
        float zMin = Utils::getParam<float>(algorithm->node, "zMin", 0.0);
        float zMax = Utils::getParam<float>(algorithm->node, "zMax", 0.1);

        if (!std::filesystem::exists(test_folder_path))
        {
            GSL_ERROR("Test data folder '{}' does not exist! Set the path using the 'test_folder' param", test_folder_path);
            CLOSE_PROGRAM;
        }

        // Results
        std::string results_folder_path = test_folder_path + "/mapir_results";
        if (!std::filesystem::exists(results_folder_path))
        {
            // Create the folder
            std::filesystem::create_directory(results_folder_path);
        }

        GSL_INFO_COLOR(fmt::terminal_color::yellow, "Filtering measurements between z={} and z={}", zMin, zMax);

        int num_samples = 0;
        // 1. Loop over each batch
        for (int batch_i = 1; batch_i <= total_batches; batch_i++)
        {
            // Path to the batch-i folder (adjust to your actual folder structure)
            std::string batch_folder = test_folder_path + "/batch-" + std::to_string(batch_i);
            GSL_INFO("Processing Batch {}...", batch_i);

            // Prepare results file
            std::string results_file_path = results_folder_path + "/batch-" + std::to_string(batch_i) + ".csv";
            std::ofstream results_file(results_file_path);
            results_file << "batch-i,N,mu_x,mu_y,mu_z,sigma00,sigma01,sigma02,sigma11,sigma12,sigma22\n"; // headers

            // 2. Loop over each test sample in the batch
            for (int sample_j = 1; sample_j <= samples_per_batch; sample_j++)
            {
                // Horrible, pero es que si no al nunca salir de esta funcion no funcionan los botones de la GUI
                algorithm->handleUI();
                while (algorithm->isPaused())
                {
                    algorithm->handleUI();
                    rclcpp::sleep_for(std::chrono::milliseconds(30));
                }

                // Path to the test-sample-j CSV file (adjust to your actual file structure)
                std::string sample_file = batch_folder + "/testing-" + intToStringWithLeadingZeros(sample_j) + "_preproc.csv";

                // 3. Load data from the CSV file
                std::ifstream file(sample_file);
                std::string line, cell;

                // Skip the first line (header)
                if (std::getline(file, line))
                {
                    // GSL_INFO("Skipping header: {}", line);
                }

                // Read each line of the CSV (usually from 4 to 6 preprocessed data points)
                while (std::getline(file, line))
                {
                    // GSL_INFO("New Line is {}", line.c_str());

                    std::vector<double> row;
                    std::stringstream lineStream(line);
                    // Columns are:  x	y	z	index	time	MiCS5524	PID-sensor	wind-u	wind-v	wind-w

                    while (std::getline(lineStream, cell, ','))
                    {
                        try
                        {
                            // GSL_INFO("New Data is {}", cell.c_str());
                            //  Trim the cell content (remove any surrounding whitespaces)
                            // cell.erase(cell.find_last_not_of(" \t\n\r") + 1);
                            // cell.erase(0, cell.find_first_not_of(" \t\n\r"));

                            if (!cell.empty())
                            {
                                // Try to convert the cell to a double
                                row.push_back(std::stod(cell)); // Convert the string to double and add to the row
                            }
                            else
                            {
                                GSL_ERROR("Warning: Empty cell encountered, skipping.");
                            }
                        }
                        catch (const std::invalid_argument& e)
                        {
                            GSL_ERROR("Error: Non-numeric data encountered in cell: {}, skipping.'", cell);
                        }
                        catch (const std::out_of_range& e)
                        {
                            GSL_ERROR("Error: Numeric value out of range for cell:{}, skipping. '", cell);
                        }
                    }

                    // For each row, update hit and wind maps
                    double x = row[0];
                    double y = row[1];
                    double z = row[2];
                    double concentration = row[6]; // PID
                    double windSpeed = std::sqrt(row[7] * row[7] + row[8] * row[8]);
                    double windDirection = calculateWindDirection(row[7], row[8]);

                    // FILTER BY HEIGHT
                    if (z > zMin && z < zMax)
                    {
                        num_samples++;
                        // Publish Anemometer to GMRF (topic based)
                        algorithm->publishAnemometer(x, y, windSpeed, windDirection);

                        // Update Gas-Hit maps
                        // GSL_INFO("UPDATING GAS-HIT: (x,y,z)=({},{},{}) avg_gas={}; avg_windSpeed={}; avg_wind_dir={}", x, y, z, concentration, windSpeed, windDirection);
                        algorithm->processGasAndWindMeasurements(x, y, concentration, windSpeed, windDirection);
                        // std::cin.clear();
                        // std::cin.get();
                        // rclcpp::sleep_for(std::chrono::milliseconds(20));
                    }
                }

                // Check if the sample is a multiple of 10
                if (sample_j % 10 == 0)
                {
                    GSL_TRACE("Toca estimar GSL: sample {} in batch-{} with {} new samples", sample_j, batch_i, num_samples);
                    algorithm->updateSourceProbability();

                    // Get Source Location estimation
                    const Vector2 groundTruth(0.675, 0.336);
                    double proportionToInclude = Utils::getParam(algorithm->node, "proportionBest", 1.); 

                    Vector2 expectedValue = algorithm->getExpectedValueSourcePosition(proportionToInclude);
                    Algorithm::CovarianceMatrix variance = algorithm->getVarianceSourcePosition(proportionToInclude);
                    float error = vmath::length(groundTruth - expectedValue);
                    algorithm->addErrorToUI(error);

                    GSL_INFO_COLOR(fmt::terminal_color::bright_blue,
                                   "\nExpected value:{}\nVariance: (X:{}, Y:{}, XY:{}\nError: {}m)",
                                   expectedValue, variance.x, variance.y, variance.covariance, error);
                    num_samples = 0;
                    // std::cin.clear();
                    // std::cin.get();

                    // Save current result to CSV file (requested format)
                    if (results_file.is_open())
                    {
                        // CSV format: Batch-i, N=sample_j, x_est, y_est, z_est, var_00, var_01, var_02, var_11, var_12, var_22
                        results_file << "batch-" + std::to_string(batch_i) + ",";
                        results_file << std::to_string(sample_j) + ",";

                        results_file << std::to_string(expectedValue.x) + ",";
                        results_file << std::to_string(expectedValue.y) + ",";
                        results_file << std::to_string(0.179) + ",";

                        results_file << std::to_string(variance.x) + ",";
                        results_file << std::to_string(variance.covariance) + ",";
                        results_file << std::to_string(0.0) + ",";
                        results_file << std::to_string(variance.y) + ",";
                        results_file << std::to_string(0.0) + ",";
                        results_file << std::to_string(0.01) + ",";

                        results_file << "\n"; // Newline for the next row
                    }
                }
            }
            GSL_TRACE("Batch-{} completed", batch_i);
            results_file.close();
            algorithm->resetMaps();
        }

        GSL_TRACE("ALL BATCHES PROCESSED!! - WORK IS DONE!");
        CLOSE_PROGRAM;

        // // ORIGINAL
        // if ((algorithm->node->now() - time_stopped).seconds() >= measure_time)
        // {
        //     double concentration = average_concentration();
        //     double windSpeed = average_windSpeed();
        //     double windDirection = average_windDirection();

        //     // Borrar todo, leer de archivo 1 medida y llamar a procesGas&Wind + POSITION
        //     GSL_INFO("avg_gas={:.2};  avg_windSpeed={:.2};  avg_wind_dir={:.2}", concentration, windSpeed, windDirection);
        //     algorithm->processGasAndWindMeasurements(concentration, windSpeed, windDirection);
        // }
    }

    void StopAndMeasureState::addGasReading(double concentration)
    {
        if (algorithm->stateMachine.getCurrentState() != this)
            return;
        gas_v.push_back(concentration);
    }

    void StopAndMeasureState::addWindReading(double speed, double direction)
    {
        if (algorithm->stateMachine.getCurrentState() != this)
            return;
        windSpeed_v.push_back(speed);
        windDirection_v.push_back(direction);
    }

    void StopAndMeasureState::OnExitState(State* next)
    {
        if (next != this)
            algorithm->currentResult = algorithm->checkSourceFound();
    }

    double StopAndMeasureState::average_concentration()
    {
        float average = Utils::getAverageFloatCollection(gas_v.begin(), gas_v.end());
        if (average == Utils::INVALID_AVERAGE)
        {
            GSL_WARN("No gas measurements were received during StopAndMeasure!");
            return 0;
        }
        return average;
    }
    double StopAndMeasureState::average_windDirection()
    {
        return Utils::getAverageDirection(windDirection_v.begin(), windDirection_v.end());
    }
    double StopAndMeasureState::average_windSpeed()
    {
        float average = Utils::getAverageFloatCollection(windSpeed_v.begin(), windSpeed_v.end());
        if (average == Utils::INVALID_AVERAGE)
        {
            GSL_WARN("No gas measurements were received during StopAndMeasure!");
            return 0;
        }
        return average;
    }
} // namespace GSL
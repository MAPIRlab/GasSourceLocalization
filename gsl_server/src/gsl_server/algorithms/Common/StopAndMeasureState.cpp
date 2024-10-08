#include "gsl_server/algorithms/PMFS/PMFS.hpp"
#include "gsl_server/core/Logging.hpp"
#include <cmath> // For atan2 and M_PI
#include <fmt/color.h>
#include <fstream>
#include <gsl_server/algorithms/Common/Algorithm.hpp>
#include <gsl_server/algorithms/Common/StopAndMeasureState.hpp>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>

namespace GSL
{
    StopAndMeasureState::StopAndMeasureState(Algorithm* _algorithm) : State(_algorithm)
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
        // atan2 returns the angle in radians, we convert it to degrees
        double angle = atan2(wind_v, wind_u) * 180.0 / M_PI;

        // Convert mathematical angle (from -180° to 180°) to meteorological convention
        double wind_direction = 270.0 - angle;

        // Ensure the result is between 0° and 360°
        if (wind_direction < 0)
        {
            wind_direction += 360.0;
        }

        return wind_direction;
    }

    void StopAndMeasureState::OnUpdate()
    {
        // 1. For each batch-i folder with data (there are 100 batches)
        //      2. For each test-sample-j file (there are 500 samples)
        //          3. Load all data points (usually from 4 to 6), and update maps
        //          4. If rem(j,10)==0, estimar posición de fuente
        GSL_TRACE("Entering StopAndMeasure::OnUpdate");
        int total_batches = 1;       // Total number of batches
        int samples_per_batch = 500; // Total number of samples per batch
        std::string test_folder_path = "/mnt/d/Projects/2024_GSL_Challenge_IEEE_ICASSP/train/test";

        // 1. Loop over each batch
        for (int batch_i = 1; batch_i <= total_batches; batch_i++)
        {
            // Path to the batch-i folder (adjust to your actual folder structure)
            std::string batch_folder = test_folder_path + "/batch-" + std::to_string(batch_i);
            GSL_INFO("Processing Batch {}...", batch_i);

            // 2. Loop over each test sample in the batch
            for (int sample_j = 1; sample_j <= samples_per_batch; sample_j++)
            {
                // Path to the test-sample-j CSV file (adjust to your actual file structure)
                std::string sample_file = batch_folder + "/testing-" + intToStringWithLeadingZeros(sample_j) + "_preproc.csv";

                // 3. Load data from the CSV file
                std::ifstream file(sample_file);
                std::string line, cell;

                // Skip the first line (header)
                if (std::getline(file, line))
                {
                    GSL_INFO("Skipping header: {}", line);
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

                    // For each row, update maps
                    double x = row[0];
                    double y = row[1];
                    double z = row[2];
                    double concentration = row[6]; // PID
                    double windSpeed = std::sqrt(row[7] * row[7] + row[8] * row[8]);
                    double windDirection = calculateWindDirection(row[7], row[8]);

                    // Publish Anemometer to GMRF (topic based)
                    algorithm->publishAnemometer(x, y, windSpeed, windDirection);

                    // Update Gas-Hit maps
                    GSL_INFO("UPDATING GAS-HIT: (x,y,z)=({},{},{}) avg_gas={}; avg_windSpeed={}; avg_wind_dir={}", x, y, z, concentration, windSpeed,
                             windDirection);
                    algorithm->processGasAndWindMeasurements(x, y, concentration, windSpeed, windDirection);
                }

                // Check if the sample is a multiple of 10
                if (sample_j % 10 == 0)
                {
                    GSL_TRACE("Toca estimar GSL: sample {} in batch-{}", sample_j, batch_i);
                    algorithm->updateSourceProbability();

                    // TODO: Show results and save to file
                    // Pepe: la varianza la necesitamos como escalar o con matriz de covarianza?
                    Vector2 expectedValue = dynamic_cast<PMFS*>(algorithm)->expectedValueSource(1.);
                    double variance = dynamic_cast<PMFS*>(algorithm)->varianceSourcePosition();
                    GSL_INFO_COLOR(fmt::terminal_color::blue, "Expected value:{}  --  Variance: {}", expectedValue, variance);
                }
            }
            GSL_TRACE("Batch-{} completed", batch_i);
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
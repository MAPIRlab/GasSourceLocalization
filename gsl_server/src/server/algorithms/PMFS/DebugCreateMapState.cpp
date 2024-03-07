#include <gsl_server/algorithms/PMFS/DebugCreateMapState.hpp>
#include <gsl_server/algorithms/Algorithm.hpp>
#include <gsl_server/algorithms/PMFS/PMFS.hpp>
#include <gsl_server/Utils/Math.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <gaden_player/srv/gas_position.hpp>

namespace GSL
{
    DebugCreateMapState::DebugCreateMapState(Algorithm* _algorithm) : State(_algorithm)
    {
        pmfs = dynamic_cast<PMFS*>(algorithm);
    }

    void DebugCreateMapState::OnEnterState(State* previousState)
    {
        GSL_TRACE("Entering DebugCreateMap");
        gasClient = pmfs->node->create_client<gaden_player::srv::GasPosition>("/odor_value");

        //get z coord of anemometer
        {
            std::string anemometer_frame = pmfs->getParam<std::string>("anemometer_frame", "anemometer_frame");
            geometry_msgs::msg::TransformStamped tfm;
            bool has_tf = false;
            do
            {
                try
                {
                    tfm = pmfs->tf_buffer.buffer.lookupTransform("map", anemometer_frame, rclcpp::Time(0));
                    has_tf = true;
                }
                catch (std::exception& e)
                {
                    GSL_ERROR("TF error when looking up map -> anemometer:\n{}", e.what());
                    rclcpp::spin_some(pmfs->node);
                }
            } while (!has_tf);

            anemometerZ = tfm.transform.translation.z;
            GSL_INFO("anemometer z is {}", anemometerZ);
        }
        iterationsCount = 0;
    }

    void DebugCreateMapState::OnUpdate()
    {
        if(iterationsCount > 5)
        {
            pmfs->stateMachine.forceSetState(pmfs->stopAndMeasureState.get());
            return;
        }

        if(!timer.isDone())
            return;

        timer.Restart(1);

        auto& node = pmfs->node;
        auto& grid = pmfs->grid;
        auto& gridData = pmfs->gridData;

        auto gasRequest = std::make_shared<gaden_player::srv::GasPosition::Request>();
        auto windRequest = std::make_shared<gaden_player::srv::WindPosition::Request>(); 
        
        // Get the measurements from the services
        
        constexpr int stepsBetweenMeasurements = 3;
        for(int i=0; i<grid.size(); i+=stepsBetweenMeasurements)
        {
            for(int j=0; j<grid[0].size(); j+=stepsBetweenMeasurements)
            {
                if(!grid[i][j].free)
                    continue;

                Vector2 position = gridData.indexToCoordinates(i, j, true);
                gasRequest->x.push_back(position.x);
                gasRequest->y.push_back(position.y);
                gasRequest->z.push_back(anemometerZ);

                windRequest->x.push_back(position.x);
                windRequest->y.push_back(position.y);
                windRequest->z.push_back(anemometerZ);
            }
        }

        auto gasFuture = gasClient->async_send_request(gasRequest);
        auto windFuture = pmfs->pubs.clientWindGroundTruth->async_send_request(windRequest);

        rclcpp::spin_until_future_complete(node, gasFuture);
        rclcpp::spin_until_future_complete(node, windFuture);

        auto gasResponse = gasFuture.get();
        auto windResponse = windFuture.get();

        // Update the map
        for (int serviceIndex = 0; serviceIndex < gasRequest->x.size(); serviceIndex++)
        {
            Vector2Int gridIndices = gridData.coordinatesToIndex(gasRequest->x[serviceIndex], gasRequest->y[serviceIndex]);
            bool hit = gasResponse->positions[serviceIndex].concentration[0] >= pmfs->thresholdGas;
            double wind_dir = atan2(windResponse->v[serviceIndex], windResponse->u[serviceIndex]);
            double wind_speed = glm::length(Vector2(windResponse->v[serviceIndex], windResponse->u[serviceIndex]));
            pmfs->estimateHitProbabilities(grid, hit, wind_dir, wind_speed, gridIndices);
        }

        iterationsCount++;
        GSL_INFO("{} iterations", iterationsCount);
    }

    void DebugCreateMapState::OnExitState(State* nextState) 
    {
        printHitmap();
        //pmfs->simulations.printImage(PMFS_internal::SimulationSource(pmfs->resultLogging.source_pose, pmfs));
        exit(-1);
    }

    void DebugCreateMapState::printHitmap()
    {
        // Save image
        auto& grid = pmfs->grid;
        auto& gridData = pmfs->gridData;

        cv::Mat image(cv::Size(grid[0].size(), grid.size()), CV_32FC3, cv::Scalar(0, 0, 0));

        for (int i = 0; i < grid.size(); i++)
        {
            for (int j = 0; j < grid[0].size(); j++)
            {
                if (grid[i][j].free)
                {
                    double hitProb = Utils::logOddsToProbability(grid[i][j].hitProbability.logOdds);
                    image.at<cv::Vec3f>(grid.size() - 1 - i, j) = cv::Vec3f(hitProb * 255, hitProb * 255, hitProb * 255);
                }
                else
                    image.at<cv::Vec3f>(grid.size() - 1 - i, j) = cv::Vec3f(0, 0, 255);
            }
        }

        std::string filename = pmfs->getParam<std::string>("hitmapFilename", "measurements");
        cv::imwrite(fmt::format("{}.png", filename), image);
        GSL_WARN("MAP IMAGE SAVED: {}.png", filename);

        calculateError();
    }

    void DebugCreateMapState::calculateError()
    {
        auto& grid = pmfs->grid;
        auto& gridData = pmfs->gridData;

        std::string filename = pmfs->getParam<std::string>("groundTruthFilename", "");
        cv::Mat groundTruthMat = cv::imread(fmt::format("{}", filename), cv::IMREAD_GRAYSCALE); //uint8

        GSL_INFO("mat type: {}", groundTruthMat.type());

        float heightRatio = static_cast<float>(groundTruthMat.size[0]) / grid.size();
        float widthRatio = static_cast<float>(groundTruthMat.size[1]) / grid[0].size();

        float accum = 0;
        float totalConfidence = 0;
        for (int i = 0; i < grid.size(); i++)
        {
            for (int j = 0; j < grid[0].size(); j++)
            {
                if (grid[i][j].free)
                {
                    int u = (grid.size() - i -1) * heightRatio;
                    int v = j * widthRatio;

                    double hitProb = Utils::logOddsToProbability(grid[i][j].hitProbability.logOdds);
                    float gt = groundTruthMat.at<uint8_t>(u, v) / 255.0;
                    accum += std::abs(hitProb - gt);// * grid[i][j].hitProbability.confidence;
                    totalConfidence += grid[i][j].hitProbability.confidence;
                }
            }
        }


        GSL_INFO("Error Measured: {}", accum / gridData.numFreeCells);

        PMFS_internal::SimulationSource source({pmfs->resultLogging.source_pose.x, pmfs->resultLogging.source_pose.y}, pmfs);
        const PMFS_internal::Settings::SimulationSettings& setts = pmfs->settings.simulation;
        std::vector<std::vector<float>> hitMap(pmfs->grid.size(), std::vector<float>(pmfs->grid[0].size(), 0.0));
        pmfs->simulations.simulateSourceInPosition(source, hitMap, false, setts.maxWarmupIterations, setts.iterationsToRecord, setts.deltaTime, setts.noiseSTDev);
    
        accum = 0;
        for (int i = 0; i < grid.size(); i++)
        {
            for (int j = 0; j < grid[0].size(); j++)
            {
                if (grid[i][j].free)
                {
                    int u = (grid.size() - i -1) * heightRatio;
                    int v = j * widthRatio;

                    double hitProb = hitMap[i][j];
                    float gt = groundTruthMat.at<uint8_t>(u, v) / 255.0;
                    accum += std::abs(hitProb - gt);
                }
            }
        }


        GSL_INFO("Error Simulated: {}", accum / gridData.numFreeCells);
    }
}
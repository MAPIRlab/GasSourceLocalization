#pragma once
#include <gsl_algorithm.h>
#include <gmrf_wind_mapping/WindEstimation.h>

#ifdef USE_GADEN
#include <gaden_player/WindPosition.h>
#endif 

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace PMFS{


struct PublishersAndSubscribers{

    gmrf_wind_mapping::WindEstimation GMRFservice;
    ros::ServiceClient clientWindGMRF;
#ifdef USE_GADEN
    gaden_player::WindPosition groundTruthWindService;
#endif 

    ros::ServiceClient clientWindGroundTruth;

    ros::Publisher gas_type_pub;
    ros::Publisher wind_repub;
    
    struct Markers{
        ros::Publisher source_probability_markers;
        ros::Publisher hitProbabilityMarkers;
        ros::Publisher quadtreePublisher;
        ros::Publisher gradientMarkers;
        ros::Publisher windArrowMarkers;
        ros::Publisher confidenceMarkers;
        
        struct Debug{
            ros::Publisher explorationValue;
            ros::Publisher varianceHit;
            ros::Publisher movementSets;
        };
        Debug debug;
    };
    Markers markers;
    PublishersAndSubscribers(){}
};
}
#include <ros/ros.h>
#include <fstream>      // std::ofstream
#include <actionlib/server/simple_action_server.h>
#include <gsl_actionserver/gsl_action_msgAction.h>
#include <numeric>      // std::inner_product
#include <eigen3/Eigen/Dense>

class CGSLServer
{
protected:
    ros::NodeHandle nh_;   //private nodehandler
    actionlib::SimpleActionServer<gsl_actionserver::gsl_action_msgAction> as_;
    std::string action_name_;

    // create messages that are used to publish feedback/result
    gsl_actionserver::gsl_action_msgFeedback feedback_;
    gsl_actionserver::gsl_action_msgResult result_;

public:
    CGSLServer(std::string name) :
        as_(nh_, name, boost::bind(&CGSLServer::executeCB, this, _1),false),
        action_name_(name)
    {
       
        as_.start();
    }

    ~CGSLServer(void){}

    void executeCB(const gsl_actionserver::gsl_action_msgGoalConstPtr &goal);
private:

    //Implement here all the GSL methods: semantics, CFD, plume_tracking, etc.
    int doSurgeCast();
    int doSpiral();
    int doSurgeSpiral();
    int doParticleFilter();
    int gsl_approach;
    std::string resultsFile;
};

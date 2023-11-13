#include <gsl_server/algorithms/GrGSL/GrGSL.h>

namespace GSL
{
    GrGSL::Cell::Cell(bool _free, double _weight) : free(_free), weight(_weight), auxWeight(0), distance(0)
    {}

    GrGSL::GrGSL(std::shared_ptr<rclcpp::Node> _node) : Algorithm(_node)
    {}

    void GrGSL::initialize()
    {
        Algorithm::initialize();

        probability_markers = node->create_publisher<Marker>("probability_markers", 10);
        estimation_markers = node->create_publisher<Marker>("estimation_markers", 10);

        clientWind = node->create_client<WindEstimation>("/WindEstimation");
        exploredCells = 0;

        waitForMapState = std::make_unique<WaitForMapState>(this);
        stopAndMeasureState = std::make_unique<StopAndMeasureState>(this);
        movingState = std::make_unique<MovingState>(this);
        stateMachine.forceSetState(waitForMapState.get());
    }

    void GrGSL::declareParameters()
    {
        Algorithm::declareParameters();
        scale = getParam<int>("scale", 65); // scale for dynamic map reduction
        stdev_hit = getParam<double>("stdev_hit", 1.0);
        stdev_miss = getParam<double>("stdev_miss", 2.0);
        infoTaxis = getParam<bool>("infoTaxis", false);
        allowMovementRepetition = getParam<bool>("allowMovementRepetition", true);
        convergence_thr = getParam<double>("convergence_thr", 0.5); // threshold for source declaration
        markers_height = getParam<double>("markers_height", 0);
    }

    void GrGSL::OnUpdate()
    {
        Algorithm::OnUpdate();
        functionQueue.run();
    }

    void GrGSL::onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        Algorithm::onGetMap(msg);
    }

} // namespace GSL
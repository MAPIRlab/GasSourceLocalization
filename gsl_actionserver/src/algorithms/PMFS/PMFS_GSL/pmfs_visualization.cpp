#include <PMFS.h>

namespace PMFS{

void PMFS_GSL::showWeights(){
    visualization_msgs::Marker sourceProbMarker=Utils::emptyMarker({0.2, 0.2});

    visualization_msgs::Marker gasProbMarker=Utils::emptyMarker({0.2, 0.2});
    
    visualization_msgs::Marker confidenceMarker = gasProbMarker;

    for(int a=0;a<cells.size();a++){
        for(int b=0;b<cells[0].size();b++){
            if(cells[a][b].free){
                auto coords = indexToCoordinates(a,b);
                geometry_msgs::Point p;
                    p.x=coords.x;
                    p.y=coords.y;
                    p.z=settings.markers_height;


                //SOURCE PROB
                double prob_s = sourceProbability(a,b);
                std_msgs::ColorRGBA color_source = Utils::valueToColor(prob_s, settings.visualization.sourceLimits.x, settings.visualization.sourceLimits.y, settings.visualization.sourceMode);
                sourceProbMarker.points.push_back(p);
                sourceProbMarker.colors.push_back(color_source);
                

                //HIT
                std_msgs::ColorRGBA col_hit = valueToColor(Utils::logOddsToProbability(cells[a][b].hitProbability.logOdds), settings.visualization.hitLimits.x, settings.visualization.hitLimits.y, settings.visualization.hitMode);
                    
                p.z = settings.markers_height-0.1;
                gasProbMarker.points.push_back(p);
                gasProbMarker.colors.push_back(col_hit);


                //CONFIDENCE
                std_msgs::ColorRGBA colorConfidence = valueToColor(cells[a][b].hitProbability.confidence, 0, 1, Utils::valueColorMode::Linear);
                    
                p.z = settings.markers_height-0.1;
                confidenceMarker.points.push_back(p);
                confidenceMarker.colors.push_back(colorConfidence);
            }
        }
    }


    pubs.markers.source_probability_markers.publish(sourceProbMarker);
    pubs.markers.hitProbabilityMarkers.publish(gasProbMarker);
    pubs.markers.confidenceMarkers.publish(confidenceMarker);
}

void PMFS_GSL::showDebugInfo(){

    visualization_msgs::Marker explorationMarker=Utils::emptyMarker({0.2, 0.2});
    
    visualization_msgs::Marker varianceMarker = explorationMarker;

    visualization_msgs::Marker movementSetsMarker = explorationMarker;

    double maxExpl=-DBL_MAX; double maxVar=-DBL_MAX;
    double minExpl=DBL_MAX; double minVar=DBL_MAX;
    for(int a=0;a<cells.size();a++){
        for(int b=0;b<cells[0].size();b++){
            if(!cells[a][b].free)
                continue;
            maxExpl = std::max(maxExpl, explorationValue(a,b));
            maxVar = std::max(maxVar, simulations.varianceOfHitProb[a][b] * (1-cells[a][b].hitProbability.confidence));

            minExpl = std::min(minExpl, explorationValue(a,b));
            minVar = std::min(minVar, simulations.varianceOfHitProb[a][b] * (1-cells[a][b].hitProbability.confidence));
        }
    }

    for(int a=0;a<cells.size();a++){
        for(int b=0;b<cells[0].size();b++){
            if(!cells[a][b].free)
                continue;
            auto coords = indexToCoordinates(a,b);
                geometry_msgs::Point p;
                    p.x=coords.x;
                    p.y=coords.y;
                    p.z=settings.markers_height;

            std_msgs::ColorRGBA explorationColor;
            std_msgs::ColorRGBA advantageColor;
            std_msgs::ColorRGBA varianceColor;

            if(openMoveSet.find( Vector2Int(a,b) ) == openMoveSet.end() ){
                explorationColor.r = 0;
                explorationColor.g = 0;
                explorationColor.b = 0;
                explorationColor.a = 1;
                advantageColor = explorationColor;
            }
            else{
                explorationColor = Utils::valueToColor(explorationValue(a,b), minExpl, maxExpl, Utils::valueColorMode::Linear);
            }
            varianceColor = Utils::valueToColor(simulations.varianceOfHitProb[a][b] * (1-cells[a][b].hitProbability.confidence), minVar, maxVar, Utils::valueColorMode::Linear);


            explorationMarker.points.push_back(p);
            explorationMarker.colors.push_back(explorationColor);

            p.z = settings.markers_height-0.1;
            varianceMarker.points.push_back(p);
            varianceMarker.colors.push_back(varianceColor);

            movementSetsMarker.points.push_back(p);
            if(openMoveSet.find({a,b}) != openMoveSet.end())
                movementSetsMarker.colors.push_back(Utils::create_color(0,1,0,1));
            else if (closedMoveSet.find(Vector2Int(a,b)) != closedMoveSet.end())
                movementSetsMarker.colors.push_back(Utils::create_color(1,0,0,1));
            else
                movementSetsMarker.colors.push_back(Utils::create_color(0,0,1,1));
        }
    }
    pubs.markers.debug.explorationValue.publish(explorationMarker);
    pubs.markers.debug.varianceHit.publish(varianceMarker);
    pubs.markers.debug.movementSets.publish(movementSetsMarker);
}


void PMFS_GSL::debugMapSegmentation(){
    visualization_msgs::MarkerArray segmentMarker;
    for(int i=0; i<simulations.QTleaves.size(); i++){
        NQA::Node* leaf = &simulations.QTleaves[i];
        visualization_msgs::Marker mark = Utils::emptyMarker({0.2, 0.2});
        mark.type = visualization_msgs::Marker::CUBE;
        Utils::Vector2 worldSpaceScale = (Utils::Vector2(leaf->size.y, leaf->size.x)) * settings.scale *map_.info.resolution;

        auto coords = indexToCoordinates(
            leaf->origin.x,
            leaf->origin.y,
            false
        ) + (worldSpaceScale*0.5);

        geometry_msgs::Point p;
            p.x=coords.x;
            p.y=coords.y;
            p.z=0;
        //mark.points.push_back(p);
        mark.pose.position=p;
        mark.id = i;
        mark.scale.x = worldSpaceScale.x-0.05;
        mark.scale.y = worldSpaceScale.y-0.05;
        mark.scale.z = 0.01;
        
        if(leaf->value <=0.5)
            mark.color=Utils::create_color(0,0,0,1);
        else
            mark.color=Utils::create_color(1,1,1,1);
        segmentMarker.markers.push_back(mark);

        pubs.markers.quadtreePublisher.publish(segmentMarker);

        if(!ros::ok())
            break;
    }
    
}



void PMFS_GSL::plotWindVectors(){
    visualization_msgs::MarkerArray arrow_array;
    //Add an ARROW marker for each node
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "WindVector";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    
    //Get max wind vector in the map (to normalize the plot)
    double max_module = 0.0;
    for (size_t i=0; i<estimatedWindVectors.size(); i++)
    {
        for (size_t j=0; j<estimatedWindVectors[0].size(); j++)
        {
            if (estimatedWindVectors[i][j].norm()> max_module)
                max_module = estimatedWindVectors[i][j].norm();
        }
    }

    for (size_t i=0; i<estimatedWindVectors.size(); i++)
    {
        for (size_t j=0; j<estimatedWindVectors[0].size(); j++)
        {
            if (cells[i][j].free)
            {
                double module = estimatedWindVectors[i][j].norm();
                double angle = std::atan2(estimatedWindVectors[i][j].y, estimatedWindVectors[i][j].x);
                if ( module> 0.001)
                {
                    marker.id = i * estimatedWindVectors[0].size() + j;
                    // Set the pose of the marker.
                    Utils::Vector2 coords = indexToCoordinates(i,j);
                    marker.pose.position.x = coords.x;
                    marker.pose.position.y = coords.y;
                    marker.pose.position.z = settings.markers_height;
                    marker.pose.orientation = tf::createQuaternionMsgFromYaw( angle);
                    //shape
                    marker.scale.x = settings.scale*map_.info.resolution *  (module/max_module);      // arrow length,
                    marker.scale.y = 0.03;           // arrow width
                    marker.scale.z = 0.05;           // arrow height
                    // color -> must normalize to [0-199]
                    size_t idx_color = 199 * (module/max_module);
                    marker.color.r = 1;
                    marker.color.g = 0;
                    marker.color.b = 0;
                    marker.color.a = 1.0;

                    //Push Arrow to array
                    arrow_array.markers.push_back(marker);
                }
            }
        }//end for
    }
    pubs.markers.windArrowMarkers.publish(arrow_array);
}



// ImGui stuff

void PMFS_GSL::renderImgui()
{
#ifdef USE_GUI
    GSLIMGUI::setup();
    GSLIMPLOT::setup();

    ros::Rate rate(30);

    while(ros::ok() && !finished){
        GSLIMGUI::StartFrame();
        createUI();
        createPlots();

        GSLIMGUI::Render();
        rate.sleep();
    }

    GSLIMPLOT::close();
    GSLIMGUI::close();
#endif
}

void PMFS_GSL::createUI()
{
#ifdef USE_GUI

    ImGui::Begin("Queries");
    {
        static int x=0;
        static int y=0;
        int selectedVar = GridUI::selectVariable();

        if(GridUI::useCoordinates()){
            static float fx, fy;
            ImGui::InputFloat("X", &fx);
            ImGui::InputFloat("Y", &fy);
            auto indices = coordinatesToIndex(fx, fy);
            x=indices.x;
            y=indices.y;
        }
        else{
            ImGui::InputInt("X", &x);
            ImGui::InputInt("Y", &y);
        }

        

        static std::string result;
        if(ImGui::Button("Print") && indicesInBounds({x,y})){
            if(selectedVar == 0)
                result = GridUI::printCell(cells, x, y);
            else if (selectedVar == 1)
                result = fmt::format("Cell {0},{1}: {2}\n", x, y, cells[x][y].sourceProbability);
        }
        if(ImGui::Button("Simulate leaf") && indicesInBounds({x,y})){
            
            NQA::Node* leaf = simulations.mapSegmentation[x][y];
            if(!leaf)
                spdlog::error("Wrong coordinates!");
            else
                simulations.printImage( SimulationSource(leaf, this) );
        }
        else if(ImGui::Button("Simulate cell") && indicesInBounds({x,y}))
        {
            simulations.printImage( SimulationSource(indexToCoordinates(x,y), this) );
        }

        ImGui::Text(result.c_str());
    }
    ImGui::End();


    ImGui::Begin("Goal");
    {
        
        ImGui::TextWrapped("Set a goal manually to be visited during the next movement phase.");
        if(GridUI::useCoordinates()){
            static float fx, fy;
            ImGui::InputFloat("X", &fx);
            ImGui::InputFloat("Y", &fy);
            debugStuff.debugGoal = coordinatesToIndex(fx, fy);
        }
        else{
            static int x,y;
            ImGui::InputInt("X", &x);
            ImGui::InputInt("Y", &y);
            debugStuff.debugGoal=Vector2Int(x,y);
        }
        if(ImGui::Button("Go")){
            debugStuff.GoalSet=true;
        }
    }
    ImGui::End();

    ImGui::Begin("Source Estimation Power");
    {
        ImGui::InputDouble("Source Power", &settings.simulation.sourceDiscriminationPower);
    }
    ImGui::End();


    ImGui::Begin("Update Source");
    {
        if(ImGui::Button("Update Source Probability")){
            submitToMainThread([this](){ 
                debugStuff.paused = true;
                simulations.updateSourceProbability(settings.simulation.refineFraction);
                showWeights();
                debugStuff.paused = false;
            });
        }
    }
    ImGui::End();
    ImGui::Begin("Quadtree");
    {
        if(ImGui::Button("Show Quadtree")){
            debugMapSegmentation();
        }
    }
    ImGui::End();
    
    ImGui::Begin("Pause/Play");
    {
        static std::string buttonText;
        if(debugStuff.paused)
            buttonText="Play";
        else
            buttonText = "Pause";
        if(ImGui::Button(buttonText.c_str())){
            debugStuff.paused=!debugStuff.paused;
        }
    }
    ImGui::End();


    ImGui::Begin("Markers");
    {
        static int modeHit = settings.visualization.hitMode; static int modeSource = settings.visualization.sourceMode;
        static float hitLimits[2] = {settings.visualization.hitLimits.x, settings.visualization.hitLimits.y};
        static float sourceLimits[2] = {settings.visualization.sourceLimits.x, settings.visualization.sourceLimits.y};

        ImGui::Combo("Hit display mode", &modeHit, "Linear\0Logarithmic\0");
        settings.visualization.hitMode = modeHit==0?Utils::valueColorMode::Linear : Utils::valueColorMode::Logarithmic;
        ImGui::InputFloat2("Hit limits", hitLimits, "%.5f");

        ImGui::Combo("Source display mode", &modeSource, "Linear\0Logarithmic\0");
        settings.visualization.sourceMode = modeSource==0?Utils::valueColorMode::Linear : Utils::valueColorMode::Logarithmic;
        ImGui::InputFloat2("Source limits", sourceLimits, "%.5f");

        settings.visualization.hitLimits.x = hitLimits[0];settings.visualization.hitLimits.y = hitLimits[1];
        settings.visualization.sourceLimits.x = sourceLimits[0];settings.visualization.sourceLimits.y = sourceLimits[1];
    }
    ImGui::End();
#endif
}

void PMFS_GSL::createPlots()
{
#ifdef USE_GUI

    ImGui::Begin("Plots");
    {
        static bool paused=false;
        
        if(ImGui::Button("Toggle Pause"))
            paused=!paused;
            
        ImGui::BulletText("Gas concentration measured over time");
        static ImPlot::ScrollingBuffer sdata1;
        ImVec2 mouse = ImGui::GetMousePos();
        static float t = 0;
        t += ImGui::GetIO().DeltaTime;
        if(last_concentration_reading!=-1)
            sdata1.AddPoint(t, last_concentration_reading);

        static float history = 10.0f;
        ImGui::SliderFloat("History",&history,1,30,"%.1f s");

        static ImPlotAxisFlags flags = ImPlotAxisFlags_AutoFit;
        static ImVec2 xAxisLimits;
        if(!paused)
            xAxisLimits={t - history, t};

        if (ImPlot::BeginPlot("##Concentration", ImVec2(-1,-1))) {
            ImPlot::SetupAxes(NULL, NULL, ImPlotAxisFlags_NoGridLines, flags);
            ImPlot::SetupAxisLimits(ImAxis_X1, xAxisLimits.x, xAxisLimits.y, paused?ImGuiCond_None:ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
            ImPlot::SetNextLineStyle({1, 0, 0, 1}, 2);
            if(sdata1.Data.size()>0)
                ImPlot::PlotLine("Concentration", &sdata1.Data[0].x, &sdata1.Data[0].y, sdata1.Data.size(), 0, sdata1.IndexOfLast, 2*sizeof(float));
            ImPlot::EndPlot();
        }
    }
    ImGui::End();
#endif
}



}
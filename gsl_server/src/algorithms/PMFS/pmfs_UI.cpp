#ifdef USE_GUI
#include <algorithms/PMFS/PMFS.h>

namespace PMFS{

bool GridUI::useCoordinates()
{
    static int selected = 0;
    ImGui::Combo("Use", &selected, "Coordinates\0Indices\0");
    return selected == 0;
}

int GridUI::selectVariable()
{
    static int selected = 0;
    ImGui::Combo("Select Variable", &selected, "Hit\0Source\0");
    return selected;
}


std::string GridUI::printCell(const std::vector<std::vector<Cell> >& cells, const int& x, const int& y)
{

    static std::string queryResult;

    if(x<0||x>cells.size() || y<0 || y>cells[0].size()){
        spdlog::error("Querying cell outside the map!");
    }
    else{
        queryResult = fmt::format("Cell {0},{1}:\n", x, y)+
            fmt::format("free:{} \n", cells[x][y].free)+
            fmt::format("auxWeight:{} \n",Utils::logOddsToProbability( cells[x][y].hitProbability.auxWeight) )+
            fmt::format("weight:{} \n", Utils::logOddsToProbability( cells[x][y].hitProbability.logOdds) );
    }

    return queryResult.c_str();
}

}
#endif
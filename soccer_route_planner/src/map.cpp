#include <soccer_route_planner/map.hpp>

Map::Map() {
    m_InflationRadius = 0.2;
    m_nNodes = 2000;
    info.resolution = 0.05;
    info.height = 9;
    info.width = 6;
    info.origin.position.x = 0;
    info.origin.position.y = 0;
    data.reserve(getCols() * getRows());
}

int8_t &Map::getOccupancy(int row, int column) {
    return data.at(row * getCols() + column);
}

int8_t &Map::getOccupancy(Pose2D position){
    return data.at((position.y - info.origin.position.y)/info.resolution * getCols() + (position.x - info.origin.position.x)/info.resolution);
}

Pose2D Map::getClosestPoint(Pose2D position){
    Pose2D closest;
    closest.x = round(position.x/info.resolution)*info.resolution;
    closest.y = round(position.y/info.resolution)*info.resolution;
    return closest;
}

void Map::UpdateOccupancyMap() {
    // Set all values in m_OccupancyMap to zero
    for (int rows = 0; rows < getRows(); rows++) {
        for (int cols = 0; cols < getCols(); cols++) {
            getOccupancy(rows, cols) = 0;
            // Set border of map to 1
            if ((rows == getRows() - 1) || (cols == getCols() - 1)) {
                getOccupancy(rows, cols) = 100;
            }
        }
    }

    // Populate m_OccupancyMap with
};

void Map::InflateOccupancyMap() {
    // Create temporary map
    std::vector<int8_t> temp_OccupancyMap;
    temp_OccupancyMap.reserve(getRows() * getCols());

    for (int rows = 0; rows < getRows(); rows++) {
        for (int cols = 0; cols < getCols(); cols++) {
            getOccupancy(rows, cols) = 0;
        }
    }
    // For point that is 1 inflate by inflation radius
    for (int rows = 0; rows < getRows(); rows++) {
        for (int cols = 0; cols < getCols(); cols++) {
            if (getOccupancy(rows, cols) == 100) {
                InflatePoint(rows, cols);
            }
        }
    }
    data = temp_OccupancyMap;
}

void Map::InflatePoint(int row, int col) {
    int GridInflationRadius = int(m_InflationRadius / info.resolution);
    // Compute max and min corners
    int max_row = std::min(row + GridInflationRadius, getRows() - 1);
    int min_row = std::max(row - GridInflationRadius, 0);
    int max_col = std::min(col + GridInflationRadius, getCols() - 1);
    int min_col = std::max(col - GridInflationRadius, 0);

    // For points in this box determine if they are within the circle
    for (int rows = min_row; rows <= max_row; rows++) {
        for (int cols = min_col; cols <= max_col; cols++) {
            int d_rows = std::abs(rows - row);
            int d_cols = std::abs(cols - col);
            if (d_rows * d_rows + d_cols * d_cols <= GridInflationRadius * GridInflationRadius) {
                getOccupancy(rows, cols) = 100;
            }
        }
    }
}
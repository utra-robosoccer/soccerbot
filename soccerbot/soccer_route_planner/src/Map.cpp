#include <soccer_route_planner/Map.hpp>

Map::Map()
{
    m_InflationRadius = 0.2;
    m_Resolution = 0.05;
    m_nNodes = 2000;
    m_Height = 9;
    m_Width = 6;
    m_Rows = int(m_Height/m_Resolution);
    m_Cols = int(m_Width/m_Resolution);
    m_OccupancyMap = grid(m_Rows, std::vector<double>(m_Cols));
}

void Map::UpdateOccupancyMap()
{
    // Set all values in m_OccupancyMap to zero
    for (int rows=0; rows < m_Rows; rows++)
    {
        for (int cols=0; cols < m_Cols; cols++)
        {
            m_OccupancyMap[rows][cols] = 0;
            // Set border of map to 1
            if((rows == m_Rows - 1) || (cols == m_Cols - 1))
            {
                m_OccupancyMap[rows][cols] = 1;
            }
        }
    }

    // Populate m_OccupancyMap with
};

void Map::InflateOccupancyMap()
{
    // Create temporary map
    grid temp_OccupancyMap(m_Rows, std::vector<double>(m_Cols));
    for (int rows=0; rows<m_Rows; rows++)
    {
        for (int cols=0; cols<m_Cols; cols++)
        {
            temp_OccupancyMap[rows][cols] = 0;
        }
    }
    // For point that is 1 inflate by inflation radius
    for (int rows=0; rows<m_Rows; rows++)
    {
        for (int cols=0; cols<m_Cols; cols++)
        {
            if (m_OccupancyMap[rows][cols] == 1)
            {
                InflatePoint(rows, cols, temp_OccupancyMap);
            }
        }
    }
    m_OccupancyMap = temp_OccupancyMap;
}

void Map::InflatePoint(int row, int col, grid &OccupancyMap)
{
    int GridInflationRadius = int(m_InflationRadius/m_Resolution);
    // Compute max and min corners
    int max_row = std::min(row + GridInflationRadius, m_Rows-1);
    int min_row = std::max(row - GridInflationRadius, 0);
    int max_col = std::min(col + GridInflationRadius, m_Cols-1);
    int min_col = std::max(col - GridInflationRadius, 0);

    // For points in this box determine if they are within the circle
    for (int rows = min_row; rows <= max_row; rows++)
    {
        for (int cols = min_col; cols <= max_col; cols++)
        {
            int d_rows = std::abs(rows - row);
            int d_cols = std::abs(cols - col);
            if (d_rows * d_rows + d_cols * d_cols <= GridInflationRadius * GridInflationRadius)
            {
                OccupancyMap[rows][cols] = 1;
            }
        }
    }
}

int Map::GetWidth() {
    return m_Width;
}

int Map::GetHeight() {
    return m_Height;
}

#ifndef TRAJECTORY_HPP_
#define TRAJECTORY_HPP_

#include <cmath>
#include <vector>
#include <algorithm>
#include <functional>
#include <iostream>
// #include "rclcpp/rclcpp.hpp"

class SearchConfig
{
public:
    double x_start;
    double y_start;
    double search_range_deg;
    double search_resolution_deg;
    double search_start_mid_deg;
    double search_length;
    // default constructor
    SearchConfig() : x_start(0), y_start(0), search_range_deg(0), search_resolution_deg(0), search_start_mid_deg(0), search_length(0) {}
};

// Extending the GridMap class
class GridMapTrajectory : public GridMap
{
public:
    // Bresenham's Line Generation Algorithm
    bool drawline(std::vector<signed char> &grid_points, double x_start, double y_start, double theta, double length, bool visualize_grid = true)
    {
        double x_end = x_start + length * cos(theta);
        double y_end = y_start + length * sin(theta);
        PointXY c_start = getIndex(x_start, y_start);
        PointXY c_end = getIndex(x_end, y_end);
        int x0 = c_start.x;
        int y0 = c_start.y;
        int x1 = c_end.x;
        int y1 = c_end.y;
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "theta:  " << theta << " length: " << length);
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "meters: " << x_start << ", " << y_start << " -> " << x_end << ", " << y_end);
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "cells:  " << x0 << ", " << y0 << " -> " << x1 << ", " << y1);

        // draw end point for debugging
        // if (x1 < cell_num_x && y1 < cell_num_y)
        // {
        //     grid_points[y1 * cell_num_x + x1] = 127;
        // }

        // Bresenham's line algorithm, which works on negative slopes
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            if (x0 >= 0 && x0 < cell_num_x && y0 >= 0 && y0 < cell_num_y)
            {
                if (grid_points[y0 * cell_num_x + x0] > 50)
                {
                    // not free
                    return false;
                    break;
                }
                // draw line points for debugging
                if (visualize_grid)
                {
                    grid_points[y0 * cell_num_x + x0] = 1;
                }
            }

            if (x0 == x1 && y0 == y1)
                break;

            int e2 = 2 * err;

            if (e2 > -dy)
            {
                err -= dy;
                x0 += sx;
            }

            if (e2 < dx)
            {
                err += dx;
                y0 += sy;
            }
        }
        // free from points
        return true;
    }

    std::vector<PointXYori> angular_search(SearchConfig s, std::vector<signed char> &hpoints, double &max_true_angle)
    {
        double search_range = s.search_range_deg * M_PI / 180;
        double search_resolution = s.search_resolution_deg * M_PI / 180;
        double search_start_mid = s.search_start_mid_deg * M_PI / 180;
        int loop_increment = int(search_range / search_resolution);
        std::vector<bool> search_results(loop_increment);
        for (int loop = 0; loop < loop_increment; loop++)
        {
            double search_ang = -0.5 * search_range + double(loop) * search_resolution;
            search_results[loop] = drawline(hpoints, s.x_start, s.y_start, search_start_mid + search_ang, s.search_length);
        }
        // find the longest continous true (free) segment in search_results
        int max_true1 = 0;
        int max_true_start = 0;
        int max_true_end = 0;
        int true_count = 0;
        int true_end = 0;
        for (int loop = 0; loop < loop_increment; loop++)
        {
            if (search_results[loop])
            {
                true_count += 1;
            }
            else
            {
                if (loop >= 1 and search_results[loop - 1])
                {
                    true_end = loop - 1;
                }
                if (true_count > max_true1)
                {

                    max_true1 = true_count;
                    max_true_end = true_end;
                }
                true_count = 0;
            }
        }
        // if everything is true (false else never evaluated)
        if (true_count > max_true1)
        {
            max_true1 = true_count;
            max_true_end = loop_increment - 1;
        }
        max_true_start = max_true_end - max_true1 + 1;

        // this for loop is only for visualization
        // true and false segments (green and red)
        /*
        for (int i = 0; i < loop_increment; i++)
        {
            if (search_results[i])
            {
                hpoints[(cell_num_y / 2 + i - loop_increment / 2) * cell_num_x] = 110;
            }
            else
            {
                hpoints[(cell_num_y / 2 + i - loop_increment / 2) * cell_num_x] = -128;
            }
        }
        */

        // RCLCPP_INFO_STREAM(this->get_logger(), "max_true: " << max_true << " max_true_start: " << max_true_start << " max_true_end: " << max_true_end);
        int max_true_center = (max_true_start + max_true_end) / 2;
        max_true_angle = -0.5 * search_range + search_start_mid + double(max_true_center) * search_resolution;

        double x_end = s.x_start + s.search_length * cos(max_true_angle);
        double y_end = s.y_start + s.search_length * sin(max_true_angle);
        PointXYori p_end = PointXYori(x_end, y_end, max_true_angle * 180 / M_PI);
        std::vector<PointXYori> p_end_vector;
        p_end_vector.push_back(p_end);
        // TODO: return the second longest segment center as well
        // TODO: remove this test:
        // if (max_true_end - max_true_start < 90){
        //     double r_x_end = s.search_length * cos(max_true_angle);
        //     double r_y_end = s.search_length * sin(max_true_angle);
        //     PointXYori r_end = PointXYori(r_x_end, r_y_end, max_true_start * 180 / M_PI);
        //     p_end_vector.push_back(r_end);
        // }
        return p_end_vector;
    }
};

#endif // TRAJECTORY_HPP_
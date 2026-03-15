#include <cmath>
#include "utils.h"

bool is_zero(double a, double eps)
{
    return abs(a) < eps;
}

int positive_mod(int a, int b)
{
    return (a % b + b) % b;
}

bool points_equal(const vcd::point& p1, const vcd::point& p2)
{
	return is_zero(p1.first - p2.first) && is_zero(p1.second - p2.second);
}

double cross_product(const vcd::point& p1, const vcd::point& p2, const vcd::point& p3)
{
	return (p2.first - p1.first) * (p3.second - p1.second) - (p2.second - p1.second) * (p3.first - p1.first);
}

bool point_on_segment(const vcd::point& p, const vcd::point& p1, const vcd::point& p2)
{
	return cross_product(p1, p2, p) == 0 && std::min(p1.first, p2.first) <= p.first 
        && p.first <= std::max(p1.first, p2.first) && std::min(p1.second, p2.second) <= p.second 
        && p.second <= std::max(p1.second, p2.second);
}

double cal_dist_xy(double x0, double x1, double y0, double y1)
{
    double x_diff = x1 - x0;
    double y_diff = y1 - y0;
	return sqrt(x_diff * x_diff + y_diff * y_diff);
}

bool point_in_polygon(const std::vector<vcd::point>& polygon, const vcd::point& p)
{
    const int n = static_cast<int>(polygon.size());
    int winding_no = 0;

    for (int i = 0; i < n; i++)
    {
        vcd::point p1 = polygon[i];
        vcd::point p2 = polygon[(i + 1) % n];
        
        if (point_on_segment(p, p1, p2))
        {
            return true;
		}

        if (p1.second <= p.second)
        {
            if (p2.second > p.second && cross_product(p1, p2, p) > 0)
            {
                winding_no++;
            }
        }
        else if (p2.second <= p.second && cross_product(p1, p2, p) < 0)
        {
            winding_no--;
        }
	}

    return winding_no != 0;
}

double cal_vertical_midpoint(double y1, double y2)
{
    return (y1 + y2) / 2;
}

void cal_centroid(const std::vector<vcd::point>& vertices, vcd::point& centroid)
{
    /*
    Adapted from https://www.geeksforgeeks.org/dsa/find-the-centroid-of-a-non-self-intersecting-closed-polygon/
    */
    const int n = static_cast<int>(vertices.size());
    double sum_x = 0;
    double sum_y = 0;

    // triangle
    if (n == 3)
    {
        for (int i = 0; i < 3; i++)
        {
            sum_x += vertices[i].first;
            sum_y += vertices[i].second;
        }
        centroid = { sum_x / 3, sum_y / 3 };
        return;
    }

    double signed_area = 0;

    // For all vertices
    for (int i = 0; i < n; i++)
    {
        double x0 = vertices[i].first;
        double y0 = vertices[i].second;
        double x1 = vertices[(i + 1) % n].first;
        double y1 = vertices[(i + 1) % n].second;

        // Calculate value of A using shoelace formula
        double A = (x0 * y1) - (x1 * y0);
        signed_area += A;

        // Calculating coordinates of polygon centroid
        sum_x += (x0 + x1) * A;
        sum_y += (y0 + y1) * A;
    }

    signed_area *= 3;
    centroid = { sum_x / signed_area, sum_y / signed_area };
}

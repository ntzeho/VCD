#pragma once

#include "vcd.h"

bool is_zero(double a, double eps = 1e-9);
int positive_mod(int a, int b);

bool points_equal(const vcd::point& p1, const vcd::point& p2);
bool point_in_polygon(const std::vector<vcd::point>& polygon, const vcd::point& p);
double cal_vertical_midpoint(double y1, double y2);
void cal_centroid(const std::vector<vcd::point>& vertices, vcd::point& centroid);

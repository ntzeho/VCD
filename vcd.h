#pragma once

#include <vector>
#include <set>

namespace vcd
{
	typedef std::pair<double, double> point;

	struct trapezoid
	{
		std::vector<point> vertices;
		std::set<point> midpoints;
		std::set<point> startpoints;
		point centroid;
		bool start = false;
		bool goal = false;
	};

	void vertical_cell_decomposition(const std::vector<point>& corridor, const std::vector<std::vector<point>>& obstacles, const std::set<point>& start_nodes, const point& end_node, std::vector<std::vector<point>>& vcd_paths);
}

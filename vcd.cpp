#include <map>
#include "vcd.h"
#include "utils.h"

namespace vcd
{
    bool line_seg_intersect(point& p1, point& p2, point& p3, point& p4, double* y_i)
    {
        /*
		calculates the intersection point of two lines (p1, p2) and (p3, p4)
		also returns the y-coordinate of the intersection point
        */

        double x1 = p1.first;
		double y1 = p1.second;
		double x2 = p2.first;
		double y2 = p2.second;
		double x3 = p3.first;
		double y3 = p3.second;
		double x4 = p4.first;
		double y4 = p4.second;

        if (is_zero(x1 - x2) && is_zero(x2 - x3) && is_zero(x3 - x4))
        {
            if (is_zero(y2 - y3)) *y_i = y2;
            else *y_i = std::min(y2, y4);
            return true;
        }

        double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
		double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
		double u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / denom;

        if (0 <= t && t <= 1 && 0 <= u && u <= 1)
        {
			*y_i = y1 + t * (y2 - y1);
            return true;
        }
        
        return false;
    }

    int corridor_line_seg_type(const point& p, const std::vector<point>& boundary, const point& ep1, const point& ep2, 
        bool* ep1_status, bool* ep2_status)
    {
        /*
        NOTE: function does not take into account edge cases where corrdior edges are perfectly vertical
        0 - no extension
        1 - down
        2 - up
        3 - both
        */
        double px = p.first;
        double py = p.second;
        point adj_point = { px, py };

        if (ep1.first < px && ep2.first < px)
        {
            // both endpoints on left side
			*ep1_status = false;
            *ep2_status = false;
            adj_point.first += SMALL_COORD_ADJ;
            if (point_in_polygon(boundary, adj_point)) return 3;
			else return 0;
        }
        else if (ep1.first > px && ep2.first > px)
        {
            // both endpoints on right side
            *ep1_status = true;
            *ep2_status = true;
            adj_point.first -= SMALL_COORD_ADJ;
            if (point_in_polygon(boundary, adj_point)) return 3;
            else return 0;
        }

        *ep1_status = false;
        *ep2_status = true;

        if (ep1.second > py && ep2.second > py)
        {
            // both y higher, down
			adj_point.second -= SMALL_COORD_ADJ;
            if (point_in_polygon(boundary, adj_point)) return 1;
            else return 2;
        }
        else if (ep1.second < py && ep2.second < py)
        {
            // both y lower, up
            adj_point.second += SMALL_COORD_ADJ;
            if (point_in_polygon(boundary, adj_point)) return 2;
			else return 1;
        }

        adj_point.second += SMALL_COORD_ADJ;
        if (point_in_polygon(boundary, adj_point)) return 2;
		else return 1;
    }

    int obstacle_line_seg_type(const point& p, const std::vector<point>& obstacle, const point& ep1, const point& ep2,
        bool* ep1_status, bool* ep2_status)
    {
        /*
        NOTE: function does not take into account edge cases where obstacle edges are perfectly vertical
        0 - no extension
        1 - down
        2 - up
        3 - both
        */
        double px = p.first;
        double py = p.second;
        point adj_point = { px, py };

        if (ep1.first < px && ep2.first < px)
        {
            // both endpoints on left side
            *ep1_status = false;
            *ep2_status = false;
            adj_point.first += SMALL_COORD_ADJ;
            if (point_in_polygon(obstacle, adj_point)) return 0;
            else return 3;
        }
        else if (ep1.first > px && ep2.first > px)
        {
            // both endpoints on right side
            *ep1_status = true;
            *ep2_status = true;
            adj_point.first -= SMALL_COORD_ADJ;
            if (point_in_polygon(obstacle, adj_point)) return 0;
            else return 3;
        }

        *ep1_status = false;
        *ep2_status = true;

        if (ep1.second > py && ep2.second > py)
        {
            // both y higher
            adj_point.second -= SMALL_COORD_ADJ;
            if (point_in_polygon(obstacle, adj_point)) return 2;
            else return 1;
        }
        else if (ep1.second < py && ep2.second < py)
        {
            // both y lower
            adj_point.second += SMALL_COORD_ADJ;
            if (point_in_polygon(obstacle, adj_point)) return 1;
            else return 2;
        }

        adj_point.second += SMALL_COORD_ADJ;
        if (point_in_polygon(obstacle, adj_point)) return 1;
        else return 2;
    }

    int cal_vertex_type(int line_seg_type, bool ep1_status, bool ep2_status)
    {
        /*
        1 - vertex to left of both endpoints | convex
		2 - vertex to left of both endpoints | concave
		3 - vertex to right of both endpoints | convex
		4 - vertex to right of both endpoints | concave
		5 - vertex to left of one endpoint and right of another | line drawn down
		6 - vertex to left of one endpoint and right of another | line drawn up
        */
        
        switch (line_seg_type)
        {
        case 0:
        {
            if (ep1_status && ep2_status) return 2;
            return 4;
        }
        case 1:
        {
            return 5;
        }
        case 2:
        {
            return 6;
        }
        case 3:
        {
			if (ep1_status && ep2_status) return 1;
			return 3;
        }
        default:
        {
            return -1; // should not happen
        }
        }
	}

    void remove_segment(std::vector<std::set<point>>& current_edges, std::set<point>& line_seg)
    {
        current_edges.erase(std::remove(current_edges.begin(), current_edges.end(), line_seg), current_edges.end());
    }

    void reset_seg_point(const point& p, int seg_idx, std::vector<std::set<point>>& current_edges, std::map<point, std::set<point>>& all_segments)
    {
        /*
        after a vertex with line segment endpoint is processed, need to update the line segment to reflect the new endpoint
        */

        std::set<point> current_edge = current_edges[seg_idx];
		point start_point = *current_edge.begin();
		point end_vertex = *current_edge.rbegin();
		current_edges[seg_idx] = { p, end_vertex };

		std::set<point> current_edge_all_segments = all_segments[end_vertex];
		point start_vertex_all_segments = *current_edge_all_segments.begin();
		point end_vertex_all_segments = *current_edge_all_segments.rbegin();
        if (is_zero(start_point.first - start_vertex_all_segments.first)) all_segments[end_vertex] = { p, end_vertex_all_segments };
		else all_segments[end_vertex] = { p, start_vertex_all_segments };
	}

    void cal_trapezoid_midpoints(trapezoid& cell, std::map<double, std::set<double>>& midpoints, const std::set<point> start_node_set, const point& end_node)
    {
        // trapezoid cell vertices arranged in pairs wiht same x-coord, if triangle last 2 form a vertical edge
        std::vector<point> vertices = cell.vertices;
        std::set<point> cell_midpoints;
        point midpoint;
        double lowest_m_y, highest_m_y, min_y, max_y, xm;
        std::set<double> y_range;

        for (int i = static_cast<int>(vertices.size()) - 1; i > 0; i -= 2)
        {
            xm = vertices[i].first;
			min_y = std::min(vertices[i].second, vertices[i - 1].second);
			max_y = std::max(vertices[i].second, vertices[i - 1].second);
            midpoint.first = xm;
            y_range = midpoints[xm];
            lowest_m_y = *y_range.begin();
            highest_m_y = *y_range.rbegin();
            
            if (y_range.size() == 1)
            {
                // only 1 midpoint
                midpoint.second = lowest_m_y;
                cell_midpoints.insert(midpoint);
            }
            else if (min_y > lowest_m_y)
            {
                // choose higher midpoint
                midpoint.second = highest_m_y;
                cell_midpoints.insert(midpoint);
            }
            else if (max_y < highest_m_y)
            {
                // choose lower midpoint
                midpoint.second = lowest_m_y;
                cell_midpoints.insert(midpoint);
			}
            else
            {
                // both midpoints valid
                midpoint.second = lowest_m_y;
                cell_midpoints.insert(midpoint);
                midpoint.second = highest_m_y;
                cell_midpoints.insert(midpoint);
            }
		}

		cell.midpoints = cell_midpoints;
        
        for (const point start_node : start_node_set)
        {
            if (point_in_polygon(vertices, start_node))
            {
                cell.startpoints.insert(start_node);
                cell.start = true;
            }
		}
        
        if (point_in_polygon(vertices, end_node)) cell.goal = true;

    }

    void connect_nodes(const std::set<point>& start_node_set, const point& end_node, trapezoid& trapezoid, std::vector<std::vector<point>>& vcd_paths)
    {

    }

	void vertical_cell_decomposition(const std::vector<point>& corridor, int corridor_length, const std::vector<std::vector<point>>& obstacles, 
        const std::set<point>& start_nodes, const point& end_node, std::vector<std::vector<point>>& vcd_paths)
	{
        /*
        Obtains convex obstacle-free cells in corridor and connects start and end nodes to obtain all homotopically distinct paths
        */

        std::map<point, std::set<point>> all_segments;
        std::map<point, int> obstacle_vertex_to_id;
        std::set<point> all_vertices;

        std::vector<std::set<point>> current_edges;
        std::set<point> line_seg_endpoints;
        std::set<point>::iterator point_set_iterator;
        std::set<double>::iterator x_seg_iterator;
        std::set<point> seg1, seg2;

        // for vcd algo, pt - top point, pb - bottom point
        point ep1, ep2, sweep_bottom, sweep_top, ls1, ls2, pt, pt_l, pb, pb_l, v_l_t, v_l_b, v_l;
        trapezoid trapezoid1, trapezoid2;
        std::vector<trapezoid> cells;
        std::map<double, std::set<double>> midpoints;
        std::map<double, std::set<double>> past_x_segments;

		bool ep1_status, ep2_status;
        int line_seg_type, pt_idx, pb_idx;
        double v_x, v_y, y_small_max, y_large_min, y_intersect;
        double min_y = corridor[0].second;
        double max_y = min_y;

        for (int i = 0; i < corridor_length; i++)
        {
            all_segments[corridor[i]] = { corridor[positive_mod(i - 1, corridor_length)], corridor[(i + 1) % corridor_length]};
            all_vertices.insert(corridor[i]);
            min_y = std::min(min_y, corridor[i].second);
            max_y = std::max(max_y, corridor[i].second);
		}

        sweep_bottom.second = min_y;
        sweep_top.second = max_y;

        for (int i = 0; i < obstacles.size(); i++)
        {
			std::vector<point> obstacle = obstacles[i];
			int obstacle_length = static_cast<int>(obstacle.size());
            for (int j = 0; j < obstacle_length; j++)
            {
				all_segments[obstacle[j]] = { obstacle[positive_mod(j - 1, obstacle_length)], obstacle[(j + 1) % obstacle_length] };
                all_vertices.insert(obstacle[j]);
				obstacle_vertex_to_id[obstacle[j]] = i;
            }
        }

        double min_x = all_vertices.begin()->first;
        double max_x = prev(all_vertices.end())->first;
        
        for (const point v : all_vertices)
        {
            v_x = v.first;
            v_y = v.second;
            line_seg_endpoints = all_segments[v];
			point_set_iterator = line_seg_endpoints.begin();
            ep1 = *point_set_iterator;
            point_set_iterator++;
            ep2 = *point_set_iterator;

            if (obstacle_vertex_to_id.count(v)) line_seg_type = obstacle_line_seg_type(v, obstacles[obstacle_vertex_to_id[v]], ep1, ep2, &ep1_status, &ep2_status);
			else line_seg_type = corridor_line_seg_type(v, corridor, ep1, ep2, &ep1_status, &ep2_status);

            seg1 = { ep1, v };
            seg2 = { ep2, v };

            if (ep1.first < v_x && ep2.first < v_x)
            {
                // both endpoints on left of v, vertex case 3
                if (ep1.second > ep2.second)
                {
                    v_l_t = ep1;
					v_l_b = ep2;
                }
                else
                {
					v_l_t = ep2;
					v_l_b = ep1;
                }
            }
            else
            {
                v_l = ep1;
            }

            sweep_bottom.first = v_x;
            sweep_top.first = v_x;

			// determine line segment type and get intersection points with sweep line accordingly
            // also update past x-segments for midpoint calculations
            pt_idx = -1;
            pb_idx = -1;
            switch (line_seg_type)
            {
            case 0:
            {
                past_x_segments[v_x] = { v_y };
                break;
            }
            case 1:
            {
                y_small_max = MIN_COORD;
                for (int i = 0; i < current_edges.size(); i++)
                {
					point_set_iterator = current_edges[i].begin();
                    ls1 = *point_set_iterator;
                    point_set_iterator++;
                    ls2 = *point_set_iterator;
                    if (line_seg_intersect(sweep_bottom, sweep_top, ls1, ls2, &y_intersect))
                    {
						if (is_zero(y_intersect - v_y)) continue; // prevent floating point errors
                        else if (y_intersect < v_y && y_intersect > y_small_max)
                        {
                            y_small_max = y_intersect;
                            pb_l = ls1;
                            pb_idx = i;
                        }
                    }
                }

                pb = { v_x, y_small_max };
                past_x_segments[v_x] = { y_small_max, v_y };
                midpoints[v_x] = { cal_vertical_midpoint(y_small_max, v_y) };
                break;
            }
            case 2:
            {
                y_large_min = MAX_COORD;
                for (int i = 0; i < current_edges.size(); i++)
                {
                    point_set_iterator = current_edges[i].begin();
                    ls1 = *point_set_iterator;
                    point_set_iterator++;
                    ls2 = *point_set_iterator;
                    if (line_seg_intersect(sweep_bottom, sweep_top, ls1, ls2, &y_intersect))
                    {
                        if (is_zero(y_intersect - v_y)) continue; // prevent floating point errors
                        else if (y_intersect > v_y && y_intersect < y_large_min)
                        {
                            y_large_min = y_intersect;
                            pt_l = ls1;
                            pt_idx = i;
                        }
                    }
                }

                pt = { v_x, y_large_min };
                past_x_segments[v_x] = { v_y, y_large_min };
                midpoints[v_x] = { cal_vertical_midpoint(v_y, y_large_min) };
                break;
            }
            case 3:
            {
                y_small_max = MIN_COORD;
                y_large_min = MAX_COORD;
                for (int i = 0; i < current_edges.size(); i++)
                {
                    point_set_iterator = current_edges[i].begin();
                    ls1 = *point_set_iterator;
                    point_set_iterator++;
                    ls2 = *point_set_iterator;
                    if (line_seg_intersect(sweep_bottom, sweep_top, ls1, ls2, &y_intersect))
                    {
                        if (is_zero(y_intersect - v_y)) continue; // prevent floating point errors
                        else if (y_intersect < v_y && y_intersect > y_small_max)
                        {
                            y_small_max = y_intersect;
                            pb_l = ls1;
                            pb_idx = i;
                        }
                        else if (y_intersect > v_y && y_intersect < y_large_min)
                        {
                            y_large_min = y_intersect;
                            pt_l = ls1;
                            pt_idx = i;
                        }
                    }
                }

                pb = { v_x, y_small_max };
                pt = { v_x, y_large_min };
                past_x_segments[v_x] = { y_small_max, v_y, y_large_min };
                midpoints[v_x] = { cal_vertical_midpoint(y_small_max, v_y), cal_vertical_midpoint(y_large_min, v_y) };
                break;
            }
            }

            // output final trapezoid, always a triangle as all x-vertices different
            if (is_zero(max_x - v.first))
            {
                trapezoid1.vertices = { v, v_l_t, v_l_b };
                cal_centroid(trapezoid1.vertices, trapezoid1.centroid);
                cal_trapezoid_midpoints(trapezoid1, midpoints, start_nodes, end_node);
                connect_nodes(start_nodes, end_node, trapezoid1, vcd_paths);
                cells.insert(cells.end(), trapezoid1);
                break;
            }

			// determine vertex type and get trapezoid(s) accordingly
            switch (cal_vertex_type(line_seg_type, ep1_status, ep2_status))
            {
            case 1:
            {
				if (pt_idx == -1 || pb_idx == -1) return; // should not happen, do not return any vcd paths

                if (points_equal(pt_l, pb_l)) trapezoid1.vertices = { pb_l, pb, pt };
				else trapezoid1.vertices = { pt_l, pb_l, pb, pt };

                cal_centroid(trapezoid1.vertices, trapezoid1.centroid);
				cal_trapezoid_midpoints(trapezoid1, midpoints, start_nodes, end_node);
                connect_nodes(start_nodes, end_node, trapezoid1, vcd_paths);
                cells.insert(cells.end(), trapezoid1);

                reset_seg_point(pt, pt_idx, current_edges, all_segments);
                reset_seg_point(pb, pb_idx, current_edges, all_segments);
				break;
            }
            case 3:
            {
				if (pt_idx == -1 || pb_idx == -1) return; // should not happen, do not return any vcd paths

                v_l_t = { pt_l.first, std::max(ep1.second, ep2.second) };
                v_l_b = { pb_l.first, std::min(ep1.second, ep2.second) };

                if (points_equal(v_l_t, pt_l)) trapezoid1.vertices = { pt_l, v, pt };
                else trapezoid1.vertices = { pt_l, v_l_t, v, pt };

                if (points_equal(v_l_b, pb_l)) trapezoid2.vertices = { pb_l, v, pb };
                else trapezoid2.vertices = { pb_l, v_l_b, v, pb };

                cal_centroid(trapezoid1.vertices, trapezoid1.centroid);
				cal_centroid(trapezoid2.vertices, trapezoid2.centroid);

                cal_trapezoid_midpoints(trapezoid1, midpoints, start_nodes, end_node);
				cal_trapezoid_midpoints(trapezoid2, midpoints, start_nodes, end_node);

				connect_nodes(start_nodes, end_node, trapezoid1, vcd_paths);
                connect_nodes(start_nodes, end_node, trapezoid2, vcd_paths);

                cells.insert(cells.end(), trapezoid1);
                cells.insert(cells.end(), trapezoid2);

                reset_seg_point(pt, pt_idx, current_edges, all_segments);
                reset_seg_point(pb, pb_idx, current_edges, all_segments);
				break;
            }
            case 4:
            {
				trapezoid1.vertices = { v, v_l_t, v_l_b }; // pb = pt = v, pt_l, pb_l, v
				cal_centroid(trapezoid1.vertices, trapezoid1.centroid);
                cal_trapezoid_midpoints(trapezoid1, midpoints, start_nodes, end_node);
                connect_nodes(start_nodes, end_node, trapezoid1, vcd_paths);
                cells.insert(cells.end(), trapezoid1);
				break;
            }
            case 5:
            {
				if (pb_idx == -1) return; // should not happen, do not return any vcd paths

                v_l = { pb_l.first, ep1.second };

                if (points_equal(v_l, pb_l)) trapezoid1.vertices = { pb_l, pb, v };
				else trapezoid1.vertices = { v_l, pb_l, pb, v };

				cal_centroid(trapezoid1.vertices, trapezoid1.centroid);
                cal_trapezoid_midpoints(trapezoid1, midpoints, start_nodes, end_node);
                connect_nodes(start_nodes, end_node, trapezoid1, vcd_paths);
                cells.insert(cells.end(), trapezoid1);
				reset_seg_point(pb, pb_idx, current_edges, all_segments);
                break;
            }
            case 6:
            {
                if (pt_idx == -1) return; // should not happen, do not return any vcd paths

                v_l = { pt_l.first, ep1.second };

                if (points_equal(v_l, pt_l)) trapezoid1.vertices = { pt_l, pt, v };
                else trapezoid1.vertices = { v_l, pt_l, pt, v };

                cal_centroid(trapezoid1.vertices, trapezoid1.centroid);
                cal_trapezoid_midpoints(trapezoid1, midpoints, start_nodes, end_node);
                connect_nodes(start_nodes, end_node, trapezoid1, vcd_paths);
                cells.insert(cells.end(), trapezoid1);
                reset_seg_point(pt, pt_idx, current_edges, all_segments);
                break;
            }
            }

            if (ep1_status) current_edges.insert(current_edges.end(), seg1);
            else remove_segment(current_edges, seg1);
            if (ep2_status) current_edges.insert(current_edges.end(), seg2);
			else remove_segment(current_edges, seg2);
        }
	}
}

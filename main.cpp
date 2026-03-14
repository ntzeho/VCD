#include <iostream>
#include <chrono>

#include "vcd.h"

int main()
{
    std::cout << "Hello World!\n";

    std::vector<vcd::point> corridor = { {0, 0}, {9, 0}, {10, 10}, {1, 10} };
    std::vector<std::vector<vcd::point>> obstacles = { {{2, 2}, {4, 2}, {3.9, 4}, {2.1, 4}}, {{6.1, 6}, {8.1, 6}, {8, 8}, {6, 8}} };
    std::set<vcd::point> start_nodes = { {1, 1}, {9, 1} };
    vcd::point end_node = {5, 5};
    std::vector<std::vector<vcd::point>> vcd_paths;
    auto start_time = std::chrono::high_resolution_clock::now();
    vcd::vertical_cell_decomposition(corridor, obstacles, start_nodes, end_node, vcd_paths);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    std::cout << "VCD paths calculated in " << duration / 1e6 << " s\n";
    return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file

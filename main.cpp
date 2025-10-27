#include "Procedure.h"
#include <iostream>

int main(int arg_count, char* arg_vars[]) {
    try {
        auto [file_name, from, to] = parse_args(arg_count, arg_vars);
        std::cout << file_name << from << to << std::endl;
        matrix_t m = load_matrix(file_name);
        std::cout << m;
        graph_t<bool> g = create_graph<bool>(m);
        std::cout << "Дэйкстра:\n";
        auto d = dijkstra_algorithm(m, 0, 1);
        std::cout << d.first << '\n';
        for (auto el : d.second) {
            std::cout << el << ' ';
        }

    }
    catch (std::exception& e) {
        std::cout << e.what();
    }
    return 0;
}

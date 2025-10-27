#include "TypesAndDevs.h"

std::tuple<const char*, node_name_t, node_name_t> parse_args(int arg_count, char* arg_vars[]);
// Возвращает граф и положительны ли веса рёбер графа:
matrix_t load_matrix(const char* filename);
template<typename Node>
graph_t<Node> create_graph(const matrix_t& matr) noexcept;
route_t topol(graph_t<bool>& g) noexcept;
components_t compute_components(const matrix_t& matr) noexcept;
std::pair<weight_t, route_t> dijkstra_algorithm(const matrix_t& matr, node_name_t key_from, node_name_t key_to) noexcept;
#include "Procedure.hpp"
#include "kosoradju-sharira.hpp"
#include "deikstra.hpp"



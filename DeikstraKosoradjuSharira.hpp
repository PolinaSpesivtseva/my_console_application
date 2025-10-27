//Deikstra
// ���� ������ ��� ������ � �����
struct NodeDijkstra {
    NodeDijkstra& operator=(const weight_t& m) { current_mark = m; return *this; }
    weight_t current_mark = INF;
    bool visited = false;
    node_name_t came_from;
};

// ��������� ��� �������� ������, ��������� ���������
class NodesToBeVisited {
public:
    NodesToBeVisited(graph_t<NodeDijkstra>& graph) { for (auto it = graph.begin(); it != graph.end(); ++it) vizited_nodes.push_back(it); }
    graph_t<NodeDijkstra>::iterator pop_min_weight() noexcept; //������� �������
    bool empty() const {
        return vizited_nodes.empty();
    }
private:
    std::vector<graph_t<NodeDijkstra>::iterator> vizited_nodes;
};

graph_t<NodeDijkstra>::iterator NodesToBeVisited::pop_min_weight() noexcept {
    auto it = std::min_element(vizited_nodes.begin(), vizited_nodes.end(),
        [](const auto a, const auto b)
        {return a->second.value().current_mark < b->second.value().current_mark; });
    auto it_min = *it;
    vizited_nodes.erase(it);
    it_min->second.value().visited = true;
    return it_min;
}

void dijkstra_step(graph_t<NodeDijkstra>& graph, NodesToBeVisited& nodes_to_be_visited) {
    auto it = nodes_to_be_visited.pop_min_weight();
    for (auto& edge : it->second) {
        auto& node = graph[edge.first];
        weight_t new_current_mark = it->second.value().current_mark + edge.second;

        if (new_current_mark < node.current_mark) {
            node.current_mark = new_current_mark;
            node.came_from = it->first;
        }
    }
}

std::pair<weight_t, route_t> find_route(const graph_t<NodeDijkstra>& graph, node_name_t key_from, node_name_t key_to) {
    auto it_from = graph.find(key_from);
    auto it_to = graph.find(key_to);
    weight_t route_length = it_to->second.value().current_mark;
    route_t route;
    if (route_length == INF) {
        route.push_back(key_from);
        route.push_back(key_to);
        return { route_length, route };
    }
    auto it = it_to;
    while (it != it_from) {
        route.push_back(it->first);
        it = graph.find(it->second.value().came_from);
    }
    route.push_back(it_from->first);
    std::reverse(route.begin(), route.end());
    return { route_length, route };
}


std::pair<weight_t, route_t> dijkstra_algorithm(const matrix_t& matr, node_name_t key_from, node_name_t key_to) noexcept {
    graph_t<NodeDijkstra> graph = create_graph<NodeDijkstra>(matr);
    // �������� ������������� ���������:
    graph.at(key_from) = 0.0;
    NodesToBeVisited nodes_to_be_visited{ graph };
    // ��������� �������� ���� ��������� (���������� � ������������� ���������):
    while (!nodes_to_be_visited.empty())
        dijkstra_step(graph, nodes_to_be_visited);
    return find_route(graph, key_from, key_to); // ����� ������ � ��� ���.
}

//KosoradjuSharira

route_t DFS(const matrix_t& matr, size_t start_node, std::vector<bool>& visited, std::vector<size_t>& path) {
    visited[start_node] = true; // ������� ������� ��� ����������
    path.push_back(start_node); // ��������� ������� ������� � ����
    for (size_t i = 0; i < matr.rows(); ++i) { // �������� �� ���� ��������
        if (matr(start_node, i) && !visited[i]) { // ���� ���� ����� �� ������� ������� � ������� i � ������� i �� ���� ��������
            DFS(matr, i, visited, path); // ���������� ��������� DFS ��� ������� i
        }
    }
    return path; // ���������� ����
}

route_t topolog_sort(const matrix_t& matr) {
    size_t num_nodes = matr.rows(); //���������� ������ � �����
    std::vector<bool> visited(num_nodes, false);//������ ����������
    std::vector<size_t> path; // ������ ����
    for (size_t i = 0; i < num_nodes; ++i) {
        if (!visited[i]) { // ���� ������� �� ��������
            path = DFS(matr, i, visited, path); // ��������� DFS ����� � ������� i � ���������� ����
        }
    }
    return path; // ���������� ������ ����, ���� ���� ������
}

template <typename Node>
graph_t<Node> create_invert_graph(const matrix_t& matr) noexcept {
    graph_t<Node> graph = create_graph<Node>(transpose(matr));
    return graph;
}

void find_comps(graph_t<bool>& g, const node_name_t& key_from, std::set<node_name_t>& strong_comp) {
    if (!g.degree_out(key_from)) {
        strong_comp.emplace(key_from);
        return;
    }
    for (auto& [key, nodes] : g) {
        if (key != key_from) continue;
        for (auto& [key_to, edges] : nodes) {
            if (g[key_to]) continue;
            g[key_to] = true;
            find_comps(g, key_to, strong_comp);
        }
    }
    strong_comp.emplace(key_from);
}

components_t rezult_comps(graph_t<bool>& g, const route_t& nodes) {
    components_t comp;
    for (auto& key : nodes) {
        std::set<node_name_t> strong_comp;
        if (!g[key]) {
            g[key] = true;
            find_comps(g, key, strong_comp);
        }
        if (strong_comp.size() != 0)
            comp.emplace(strong_comp);
    }
    return comp;
}

components_t compute_components(const matrix_t& matr) noexcept {
    graph_t<bool> graph_initial = create_graph<bool>(matr);
    route_t sorted_nodes = topolog_sort(transpose(matr));
    std::reverse(sorted_nodes.begin(), sorted_nodes.end());
    return rezult_comps(graph_initial, sorted_nodes);
}


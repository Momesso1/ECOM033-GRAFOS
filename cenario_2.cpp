#include <unordered_map>
#include <vector>
#include <iostream>
#include <fstream>
#include <limits>

/*

    VAMOS COMPARAR O CÓDIGO FEITO COM O PSEUDOCÓDIGO COLOCANDO
    A PARTE DO PSEUDOCÓDIGO CORRESPONDENTE ACIMA DA PARTE DO CÓDIGO FEITO.

*/

/*

PSEUDOCÓDIGO DO BELLMAN-FORD

Bellman_Ford(G,w,s):
    Initialize_Single_Source(G,s)
    for i = 1 to |V|-1:
        for each edge (u,v) in G.E:
            Relax(u,v,w)
    for each edge (u,v) in G.E:
        if v.d > u.d + w(u,v)
            return false  // ciclo negativo
    return true

*/

struct Node {
    int parent = -1;                    // predecessor
    float g_score = std::numeric_limits<float>::infinity(); // dist
};

// Relax(u,v,w)
void Relax(Node& u, Node& v, int w, int u_id) {
    if (v.g_score > u.g_score + w) {
        v.g_score = u.g_score + w;
        v.parent = u_id;
    }
}

// Bellman-Ford
bool bellman_ford(const std::unordered_map<int, std::unordered_map<int,int>>& adjacency_list,
                  int vertice_inicial,
                  std::unordered_map<int, Node>& nodes)
{
    // Initialize_Single_Source
    for (auto& [id, node] : nodes) {
        node.g_score = std::numeric_limits<float>::infinity();
        node.parent = -1;
    }
    nodes[vertice_inicial].g_score = 0;

    int V = nodes.size();

    // for i = 1 to |V|-1
    for (int i = 0; i < V-1; i++) {
        // for each edge (u,v) in G.E
        for (const auto& [u, neighbors] : adjacency_list) {
            for (const auto& [v, w] : neighbors) {
                Relax(nodes[u], nodes[v], w, u); // Relax(u,v,w)
            }
        }
    }

    // Verificação de ciclo negativo
    for (const auto& [u, neighbors] : adjacency_list) {
        for (const auto& [v, w] : neighbors) {
            if (nodes[v].g_score > nodes[u].g_score + w) {
                return false; // ciclo negativo
            }
        }
    }

    return true;
}

std::pair<std::vector<int>, int> get_path_and_cost(const std::unordered_map<int, Node>& nodes, int vertice_final)
{
    std::vector<int> path;
    int atual = vertice_final;
    if (nodes.at(atual).g_score != std::numeric_limits<float>::infinity()) {
        while (atual != -1) {
            path.insert(path.begin(), atual);
            atual = nodes.at(atual).parent;
        }
    }

    int custo_total = (nodes.at(vertice_final).g_score == std::numeric_limits<float>::infinity())
                        ? -1
                        : static_cast<int>(nodes.at(vertice_final).g_score);

    return {path, custo_total};
}

int main()
{
    std::ifstream infile("graph2.txt");
    if (!infile.is_open()) {
        std::cerr << "Erro ao abrir arquivo\n";
        return 1;
    }

    int numero_de_vertices, numero_de_arestas;
    infile >> numero_de_vertices >> numero_de_arestas;

    std::unordered_map<int, std::unordered_map<int,int>> adjacency_list;
    std::unordered_map<int, Node> nodes;

    int u, v, w;
    for (int i = 0; i < numero_de_arestas; i++) {
        infile >> u >> v >> w;
        adjacency_list[u][v] = w;
        nodes[u]; nodes[v]; 
    }

    infile.close();

    bool sucesso = bellman_ford(adjacency_list, 0, nodes);

    if (!sucesso) {
        std::cerr << "Grafo possui ciclo negativo\n";
    }

    auto resultado = get_path_and_cost(nodes, 6);

    std::cout << "1. Caminho: ";
    if (!resultado.first.empty()) {
        for (size_t i = 0; i < resultado.first.size()-1; i++)
            std::cout << resultado.first[i] << " -> ";
        std::cout << resultado.first.back() << "\n";
    } else {
        std::cout << "Nenhum caminho encontrado\n";
    }

    std::cout << "2. Custo total: " << resultado.second << std::endl;

    return 0;
}

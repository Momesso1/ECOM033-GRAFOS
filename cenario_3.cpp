#include <unordered_map>
#include <queue>
#include <limits>
#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>

/*
 Pseudocódigo do Algoritmo de Dijkstra (para encontrar um caminho)

 1  função Dijkstra(Grafo, vértice_inicial, vértice_final):
 2      para cada vértice v no Grafo:
 3          distancias[v] := infinito
 4          anteriores[v] := indefinido
 5
 6      distancias[vértice_inicial] := 0
 7
 8      ConjuntoAberto := uma fila de prioridade com o vértice_inicial
 9
10      enquanto ConjuntoAberto não está vazio:
11          u := vértice em ConjuntoAberto com a menor distância
12          remover u de ConjuntoAberto
13
14          se u é o vértice_final:
15              // Reconstruir o caminho e retornar
16              Caminho := []
17              temp := u
18              enquanto temp não é indefinido:
19                  adicionar temp no início de Caminho
20                  temp := anteriores[temp]
21              retornar Caminho e distancias[u]
22
23          para cada vizinho v de u:
24              dist_alternativa := distancias[u] + peso da aresta (u, v)
25              se dist_alternativa < distancias[v]:
26                  distancias[v] := dist_alternativa
27                  anteriores[v] := u
28                  adicionar v ao ConjuntoAberto
29
30      retornar falha (caminho não encontrado)
*/
std::pair<std::vector<int>, int> dijkstra(const std::unordered_map<int, std::unordered_map<int, int>>& adjacency_list, char vertice_inicial, char vertice_final)
{
    // Estrutura para armazenar os dados de cada nó (vértice).
    // Corresponde às estruturas de 'distancias' e 'anteriores' do pseudocódigo.
    struct Node {
        int parent = -1;                                        // Linha 4: anteriores[v]
        float g_score = std::numeric_limits<float>::infinity(); // Linha 3: distancias[v]
        bool closed = false;                                    // Usado para marcar nós já visitados
    };

    // Mapa que armazena os dados de todos os nós. A inicialização com infinito (g_score) é implícita.
    // Corresponde à linha 2 do pseudocódigo.
    std::unordered_map<int, Node> nodes;

    // Define a distância do vértice inicial como 0.
    // Corresponde à linha 6 do pseudocódigo.
    nodes[vertice_inicial].g_score = 0;

    // Estrutura para a fila de prioridade, que garantirá que o vértice
    // com menor custo seja sempre processado primeiro.
    struct PairCompare {
        bool operator()(const std::pair<float, int>& a,
                        const std::pair<float, int>& b) const {
            return a.first > b.first;
        }
    };

    // Declaração da fila de prioridade (ConjuntoAberto).
    // Corresponde à linha 8 do pseudocódigo.
    std::priority_queue<
        std::pair<float, int>,
        std::vector<std::pair<float, int>>,
        PairCompare
    > open_set;

    // Adiciona o vértice inicial à fila de prioridade.
    open_set.emplace(0, vertice_inicial);

    // Vetor para armazenar o caminho reconstruído.
    // Corresponde à variável 'Caminho' na linha 16.
    std::vector<int> path;
    
    // Loop principal do algoritmo.
    // Corresponde à linha 10 do pseudocódigo.
    while (!open_set.empty()) 
    {
        // Pega o vértice com a menor distância na fila de prioridade.
        // Corresponde à linha 11 do pseudocódigo.
        auto [current_cost, current] = open_set.top();
        
        // Remove o vértice da fila.
        // Corresponde à linha 12 do pseudocódigo.
        open_set.pop();

        // Otimizações: ignora se o nó já foi visitado ou se já existe um caminho mais curto.
        if (nodes[current].closed) continue;
        if (current_cost > nodes[current].g_score) continue;

        nodes[current].closed = true;

        // Itera sobre todos os vizinhos do vértice atual, se existirem.
        // Corresponde à linha 23 do pseudocódigo.
        if(adjacency_list.find(current) != adjacency_list.end())
        {
            for (const auto& [neighbor, weight] : adjacency_list.at(current)) 
            {
                if (nodes[neighbor].closed) continue;

                // Calcula o custo para chegar ao vizinho através do nó atual.
                // Corresponde à linha 24 do pseudocódigo.
                float tentative_g_score = nodes[current].g_score + weight;

                // Verifica se o novo caminho é mais curto que o anterior.
                // Corresponde à linha 25 do pseudocódigo.
                if (tentative_g_score < nodes[neighbor].g_score) 
                {
                    // Atualiza o nó anterior do vizinho.
                    // Corresponde à linha 27 do pseudocódigo.
                    nodes[neighbor].parent = current;

                    // Atualiza a distância (custo) para o vizinho.
                    // Corresponde à linha 26 do pseudocódigo.
                    nodes[neighbor].g_score = tentative_g_score;

                    // Adiciona o vizinho atualizado à fila de prioridade.
                    // Corresponde à linha 28 do pseudocódigo.
                    open_set.emplace(nodes[neighbor].g_score, neighbor);
                }
            }
        }

        // Verifica se o vértice atual é o destino.
        // Corresponde à linha 14 do pseudocódigo.
        if (current == vertice_final) 
        {
            auto current_vertex = current;
            
            // Inicia a reconstrução do caminho, do fim para o início.
            // Corresponde às linhas 16-20 do pseudocódigo.
            path.insert(path.begin(), current_vertex);
            
            while (nodes.find(current_vertex) != nodes.end() && current_vertex != vertice_inicial) 
            {
                current_vertex = nodes[current_vertex].parent;
                path.insert(path.begin(), current_vertex);
            }
            
            break; // Sai do loop, pois o caminho foi encontrado.
        }
    }

    // --- CORREÇÃO ---
    // O custo total do caminho é a distância acumulada no nó final,
    // não a soma das distâncias de todos os nós explorados.
    // Corresponde ao retorno de 'distancias[u]' na linha 21.
    int distance = 0;
    if (!path.empty()) // Garante que um caminho foi encontrado
    {
        distance = nodes[vertice_final].g_score;
    }

    std::pair<std::vector<int>, int> resultado = std::make_pair(path, distance);

    return resultado;
}


int main() 
{
     std::ifstream infile("grid_example.txt");
    if (!infile.is_open()) {
        std::cerr << "Erro ao abrir arquivo\n";
        return 1;
    }

    int numero_de_linhas, numero_de_colunas;
    infile >> numero_de_linhas >> numero_de_colunas;
    infile.ignore(); 

    std::vector<std::string> grid(numero_de_linhas);
    for (int i = 0; i < numero_de_linhas; i++) 
    {
        std::getline(infile, grid[i]);
        if (grid[i].size() > static_cast<size_t>(numero_de_colunas)) grid[i] = grid[i].substr(0, numero_de_colunas);
    }

    infile.close();

    std::unordered_map<int, std::unordered_map<int, int>> adjacency_list;

    auto in_bounds = [&](int x, int y) 
    {
        return x >= 0 && x < numero_de_linhas && y >= 0 && y < numero_de_colunas;
    };

    auto cell_to_vertex = [&](int x, int y) 
    {
        return x * numero_de_colunas + y; 
    };

    const int dx[4] = {-1, 1, 0, 0};
    const int dy[4] = {0, 0, -1, 1};

    int vertice_inicial = 0, vertice_final = 0;

    for (int x = 0; x < numero_de_linhas; x++) 
    {
        for (int y = 0; y < numero_de_colunas; y++) 
        {
            char cell = grid[x][y];
            if (cell == '#') continue;

            int u = cell_to_vertex(x, y);

            if (cell == 'S') vertice_inicial = u;
            if (cell == 'G') vertice_final = u;

            for (int dir = 0; dir < 4; dir++) 
            {
                int nx = x + dx[dir];
                int ny = y + dy[dir];

                if (!in_bounds(nx, ny)) continue;
                char neighbor_cell = grid[nx][ny];
                if (neighbor_cell == '#') continue;

                int v = cell_to_vertex(nx, ny);

                int cost = 1; 
                if (neighbor_cell == '~') cost = 3;
                adjacency_list[u][v] = cost;
            }
        }
    }


 
    std::pair<std::vector<int>, int> resultado = dijkstra(adjacency_list, vertice_inicial, vertice_final);

    std::cout << "1. Caminho: " << std::endl;

    for(size_t i = 0; i < resultado.first.size() - 1; i++)
    {
        std::cout << resultado.first[i] << " -> ";
    }

    std::cout << resultado.first[resultado.first.size() - 1] << std::endl;

    
    std::cout << "2. Custo total: " << resultado.second << std::endl;
 
    return 0;
}
#include <unordered_map>
#include <queue>
#include <limits>
#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>

struct Resultados
{
    int soma_total = 0;
    std::unordered_map<int, int> custo_v_inicial_para_final;
    std::pair<int, int> vertice_mais_distance_e_custo;
};

/*
 Pseudocódigo do Algoritmo de Dijkstra

 1  função Dijkstra(Grafo, vértice_inicial):
 2      para cada vértice v no Grafo:
 3          distancias[v] := infinito
 4          anteriores[v] := indefinido
 5
 6      distancias[vértice_inicial] := 0
 7
 8      ConjuntoAberto := uma fila de prioridade com todos os vértices do Grafo
 9
10      enquanto ConjuntoAberto não está vazio:
11          u := vértice em ConjuntoAberto com a menor distância
12          remover u de ConjuntoAberto
13
14          para cada vizinho v de u:
15              dist_alternativa := distancias[u] + peso da aresta (u, v)
16              se dist_alternativa < distancias[v]:
17                  distancias[v] := dist_alternativa
18                  anteriores[v] := u
19
20      retornar distancias, anteriores
*/
Resultados dijkstra(const std::unordered_map<int, std::unordered_map<int, int>>& adjacency_list, int vertice_inicial)
{
    // Estrutura para armazenar os dados de cada nó (vértice).
    // Corresponde às estruturas de 'distancias' e 'anteriores' do pseudocódigo.
    struct Node {
        int parent = -1;                                        // Linha 4: anteriores[v]
        float g_score = std::numeric_limits<float>::infinity(); // Linha 3: distancias[v]
        bool closed = false;                                    // Usado para marcar nós já visitados
    };

    // Mapa que armazena os dados de todos os nós. A inicialização com infinito (g_score) é implícita.
    // Corresponde às linhas 2-4 do pseudocódigo.
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

    int distance = 0; // Variável auxiliar para calcular a soma total das distâncias.

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

        // Otimizações: ignora se o nó já foi visitado (fechado) ou se encontramos
        // um caminho mais curto para ele em uma iteração anterior.
        if (nodes[current].closed) continue;
        if (current_cost > nodes[current].g_score) continue;

        // Marca o nó atual como visitado/fechado.
        nodes[current].closed = true;

        // Itera sobre todos os vizinhos do vértice atual.
        // Corresponde à linha 14 do pseudocódigo.
        for (const auto& [neighbor, weight] : adjacency_list.at(current))
        {
            // Ignora vizinhos que já foram finalizados.
            if (nodes[neighbor].closed) continue;

            // Calcula o custo para chegar ao vizinho através do nó atual.
            // Corresponde à linha 15 do pseudocódigo.
            float tentative_g_score = nodes[current].g_score + weight;

            // Verifica se o novo caminho é mais curto que o anterior.
            // Corresponde à linha 16 do pseudocódigo.
            if (tentative_g_score < nodes[neighbor].g_score)
            {
                // Atualiza o nó anterior do vizinho.
                // Corresponde à linha 18 do pseudocódigo.
                nodes[neighbor].parent = current;

                // Atualiza a distância (custo) para o vizinho.
                // Corresponde à linha 17 do pseudocódigo.
                nodes[neighbor].g_score = tentative_g_score;

                // Adiciona o vizinho atualizado à fila de prioridade para ser processado.
                open_set.emplace(nodes[neighbor].g_score, neighbor);
            }
        }
    }

    // A partir daqui, o código processa e formata os resultados obtidos.
    // Corresponde à linha 20 do pseudocódigo (retornar distancias).
    std::unordered_map<int, int> custo_v_inicial_para_final_local;
    int vertice, custo = 0;

    for(const auto& [chave, node] : nodes)
    {
        custo_v_inicial_para_final_local[chave] = node.g_score;

        if(node.g_score > custo)
        {
            custo = node.g_score;
            vertice = chave;
        }

        distance = distance + node.g_score;
    }

    Resultados toma;
    toma.custo_v_inicial_para_final = custo_v_inicial_para_final_local;
    toma.vertice_mais_distance_e_custo = std::make_pair(vertice, custo);
    toma.soma_total = distance;

    return toma;
}


int main()
{
    std::ifstream infile("graph1.txt");
    if (!infile.is_open())
    {
        std::cerr << "Erro ao abrir arquivo\n";
        return 1;
    }

    int numero_de_vertices, numero_de_arestas;
    infile >> numero_de_vertices >> numero_de_arestas;

    std::unordered_map<int, std::unordered_map<int, int>> adjacency_list;

    int u, v, w;
    for (int i = 0; i < numero_de_arestas; i++) {
        infile >> u >> v >> w;
        adjacency_list[u][v] = w;
        adjacency_list[v][u] = w;
    }

    infile.close();
    std::unordered_map<int, Resultados> resultado;

    int distancia = static_cast<int>(std::numeric_limits<float>::infinity());
    int melhor_vertice = 0;

    for(int i = 1; i <= numero_de_vertices; i++)
    {
        resultado[i] = dijkstra(adjacency_list, i);

        if(resultado[i].soma_total < distancia)
        {
            melhor_vertice = i;
            distancia = resultado[i].soma_total;
        }
    }

    std::cout << "1. Vértice central: " << melhor_vertice << std::endl;
    std::cout << "2. Distâncias: \n" << std::endl;

    std::vector<std::pair<int, int>> distancias(resultado[melhor_vertice].custo_v_inicial_para_final.begin(),resultado[melhor_vertice].custo_v_inicial_para_final.end());

    std::sort(distancias.begin(), distancias.end(),
            [](const auto& a, const auto& b) {
                return a.second < b.second;
            });

    for (const auto& [chave, dados] : distancias)
    {
        if(chave != melhor_vertice)
        {
            std::cout << melhor_vertice << " -> " << chave << " | " << dados << std::endl;
        }
    }

    std::cout << std::endl;
    std::cout << "3. Vértice mais distante: " << distancias[distancias.size() - 1].first << " | " << distancias[distancias.size() - 1].second << std::endl;

    std::cout << "\n4. Matriz de distâncias (linhas = candidatos, colunas = vértices):\n" << std::endl;
    for(int i = 1; i <= numero_de_vertices; i++)
    {
        std::cout << "Vértice " << i << ": ";
        for(int j = 1; j <= numero_de_vertices; j++)
        {
            std::cout << resultado[i].custo_v_inicial_para_final[j] << " ";
        }
        std::cout << std::endl;
    }
    return 0;
}
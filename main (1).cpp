#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>

using namespace std;

// Graph Representation
using Graph = unordered_map<int, vector<pair<int, double>>>;

// BFS to find all vertices reachable from a source
void BFS(const Graph& graph, int start, vector<int>& visited) {
    queue<int> q;
    q.push(start);
    visited[start] = 1;

    while (!q.empty()) {
        int node = q.front();
        q.pop();

        for (const auto& neighbor : graph.at(node)) {
            if (!visited[neighbor.first]) {
                visited[neighbor.first] = 1;
                q.push(neighbor.first);
            }
        }
    }
}

// Dijkstra's Algorithm for shortest path
vector<double> dijkstra(const Graph& graph, int start, int numVertices) {
    vector<double> dist(numVertices, numeric_limits<double>::infinity());
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;

    dist[start] = 0;
    pq.push({0, start});

    while (!pq.empty()) {
        double d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (d > dist[u]) continue;

        for (const auto& neighbor : graph.at(u)) {
            int v = neighbor.first;
            double weight = neighbor.second;
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                pq.push({dist[v], v});
            }
        }
    }

    return dist;
}

// Visualization with SFML
void visualizeGraph(const Graph& graph) {
    sf::RenderWindow window(sf::VideoMode(800, 600), "Credit Risk Graph Visualization");
    sf::CircleShape vertexShape(10);
    vertexShape.setFillColor(sf::Color::Green);

    // Draw vertices and edges
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::White);

        // Draw vertices (mock positions)
        for (const auto& [vertex, edges] : graph) {
            vertexShape.setPosition(rand() % 800, rand() % 600); // Random position for demonstration
            window.draw(vertexShape);
        }

        // Draw edges (mock lines)
        for (const auto& [vertex, edges] : graph) {
            for (const auto& [neighbor, weight] : edges) {
                sf::Vertex line[] = {
                    sf::Vertex(sf::Vector2f(rand() % 800, rand() % 600)),
                    sf::Vertex(sf::Vector2f(rand() % 800, rand() % 600))
                };
                window.draw(line, 2, sf::Lines);
            }
        }

        window.display();
    }
}

int main() {
    int numVertices, numEdges;

    cout << "Enter the number of vertices: ";
    cin >> numVertices;

    Graph graph;

    cout << "Enter the number of edges: ";
    cin >> numEdges;

    cout << "Enter the edges (source destination weight):" << endl;
    for (int i = 0; i < numEdges; ++i) {
        int u, v;
        double weight;
        cin >> u >> v >> weight;
        graph[u].emplace_back(v, weight);
    }

    // BFS Example
    vector<int> visited(numVertices + 1, 0);
    int startVertex;
    cout << "Enter the start vertex for BFS: ";
    cin >> startVertex;
    BFS(graph, startVertex, visited);
    cout << "BFS Visit Order:" << endl;
    for (int i = 1; i <= numVertices; ++i) {
        if (visited[i]) cout << "Vertex " << i << " is reachable." << endl;
    }

    // Dijkstra's Example
    auto distances = dijkstra(graph, startVertex, numVertices + 1);
    cout << "Dijkstra's Shortest Paths from Vertex " << startVertex << ":" << endl;
    for (int i = 1; i <= numVertices; ++i) {
        cout << "Distance to Vertex " << i << ": " << distances[i] << endl;
    }

    // Visualize the graph
    visualizeGraph(graph);

    return 0;
}

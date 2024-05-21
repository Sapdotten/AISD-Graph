#ifndef INCLUDE_GRAPH_H
#define INCLUDE_GRAPH_H
#include <iostream>
#include <list>
#include <memory>
#include <stack>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <queue>
namespace graph_space {
template <typename Vertex, typename Distance = double>
class Graph {
 public:
  struct Edge {
    Vertex from, to;
    Distance dist;
    Edge(const Vertex& from_, const Vertex& to_, const Distance d)
        : from(from_), to(to_), dist(d) {}
    Edge(const Edge& another) {
      from = another.from;
      to = another.to;
      dist = another.dist;
    }
    bool operator==(const Edge& other) const {
      return (from == other.from && to == other.to && dist == other.dist);
    }
    Edge operator=(const Edge& another) {
      from = another.from;
      to = another.to;
      dist = another.dist;
      return (*this);
    }
  };
  using edge_set = std::list<Edge>;
  std::unordered_map<Vertex, std::list<Edge>> graph;

  Graph() = default;
  ~Graph() = default;

  bool has_vertex(const Vertex& v) const {
    return this->graph.find(v) != graph.end();
  }

  void add_vertex(const Vertex& v) {
    if (!(this->has_vertex(v))) {
      graph[v] = std::list<Edge>();
    }
  }

  bool remove_vertex(const Vertex& v) {
    this->graph.erase(this->graph.find(v));
    bool flag = false;
    for (auto& [vertex, edges] : graph) {
      for (auto it = edges.begin(); it != edges.end();) {
        if (it->to == v) {
          edges.erase(it);
          flag = true;
        } else
          ++it;
      }
    }
    return flag;
  }
  //проверка-добавление-удаление ребер

  bool has_edge(const Vertex& from, const Vertex& to) const {
    auto it = this->graph.find(from);
    if (it != this->graph.end()) {
      const auto& vertex = it->second;
      for (const auto& edge : vertex) {
        if (edge.to == to) return true;
      }
    }
    return false;
  }

  void add_edge(const Vertex& from, const Vertex& to, const Distance& d) {
    if (this->has_edge(from, to))
      throw std::runtime_error("This edge already exists...");
    if (!(this->has_vertex(from))) {
      graph[from] = std::list<Edge>();
    }
    this->graph[from].push_back(Edge(from, to, d));
    if (!(this->has_vertex(to))) {
      graph[to] = std::list<Edge>();
    }
  }
  bool remove_edge(const Vertex& from, const Vertex& to) {
    if (!(this->has_edge(from, to))) return false;
    auto& [vertex, edges] = *(graph.find(from));
    for (auto it = edges.begin(); it != edges.end();) {
      if (it->to == to) {
        it = edges.erase(it);
        return true;
      } else
        ++it;
    }
  }
  bool has_edge(const Edge& e) const {
    if (this->has_vertex(e.from)) {
      auto& [vertex, edges] = *(graph.find(e.from));
      for (auto& edge : edges) {
        if (edge == e) return true;
      }
    }
    return false;
  }
  bool remove_edge(const Edge& e) {
    if (!(this->has_edge(e))) return false;
    auto& [vertex, edges] = *(graph.find(e.from));
    for (auto it = edges.begin(); it != edges.end();) {
      if (it->to == e.to && it->dist == e.dist) {
        it = edges.erase(it);
        return true;
      } else
        ++it;
    }
  }

  size_t order() const { return graph.size(); }

  size_t degree(const Vertex& v) const {
    auto it = graph.find(v);
    if (it == graph.end())
      throw std::runtime_error("There is no that vertex...");
    auto& [_, edges_v] = *it;
    size_t v_out = edges_v.size();
    size_t v_in = 0;
    for (auto& [_, edges] : graph) {
      for (auto& edge : edges) {
        if (edge.to == v) v_in++;
      }
    }
    return v_out + v_in;
  }

  //получение всех ребер, выходящих из вершины
  std::vector<Edge> edges(const Vertex& vertex) {
    std::vector<Edge> edges_from_v;
    auto it = graph.find(vertex);
    if (it == graph.end())
      throw std::runtime_error("There is no this vertex...");
    auto& [_, edges] = *it;
    for (auto& edge : edges) {
      edges_from_v.push_back(edge);
    }
    return edges_from_v;
  }

  std::vector<Vertex> vertices() const {
    std::vector<Vertex> vs;
    vs.reserve(graph.size());
    for (auto& [vertex, _] : graph) {
      vs.push_back(vertex);
    }
    return vs;
  }

  //обход
  std::vector<Vertex> walk(const Vertex& start_vertex) const {
    auto it = graph.find(start_vertex);
    if (it == graph.end())
      throw std::runtime_error("There is no this vertex...");
    std::unordered_set<Vertex> visited;
    std::stack<Vertex> to_visit;
    std::vector<Vertex> vertexes;
    auto& [vertex, edges] = *it;
    visited.insert(vertex);
    vertexes.push_back(vertex);
    for (auto& e : edges) {
      if (!visited.contains(e.to)) to_visit.push(e.to);
    }
    while (!to_visit.empty()) {
      auto v = to_visit.top();
      to_visit.pop();
      if (!visited.contains(v)) {
        vertexes.push_back(v);
        visited.insert(v);
      }
      auto it = graph.find(start_vertex);
      auto& [vertex, edges] = *it;
      for (auto& e : edges) {
        if (!visited.contains(e.to)) to_visit.push(e.to);
      }
    }
    return vertexes;
  }


   std::vector<Edge> shortest_path(const Vertex& start, const Vertex& end){
    std::unordered_map<Vertex, Distance> dist;
    std::unordered_map<Vertex, Vertex> prev;
    for (const auto& pair : graph) {
      dist[pair.first] = std::numeric_limits<Distance>::max();
    }
    dist[start] = 0;

    size_t n = graph.size();
    for (int i = 0; i < n - 1; ++i) {
      bool updated = false;
      for (const auto& pair : graph) {
        const Vertex& u = pair.first;
        for (const Edge& edge : pair.second) {
          const Vertex& v = edge.to;
          const Distance& weight = edge.dist;
          if (dist[u] != std::numeric_limits<Distance>::max() &&
              dist[v] > dist[u] + weight) {
            dist[v] = dist[u] + weight;
            prev[v] = u;
            updated = true;
          }
        }
      }
      if (!updated) {
        break;
      }
    }

    bool negative_cycle_exists = false;
    for (const auto& pair : graph) {
      const Vertex& u = pair.first;
      for (const Edge& edge : pair.second) {
        const Vertex& v = edge.to;
        const Distance& weight = edge.dist;
        if (dist[u] != std::numeric_limits<Distance>::max() &&
            dist[v] > dist[u] + weight) {
          negative_cycle_exists = true;
          break;
        }
      }
      if (negative_cycle_exists) {
        break;
      }
    }

    if (negative_cycle_exists) {
      throw std::runtime_error(
          "Negative cycle detected. Cannot find shortest path.");
      return {};
    }

    if (dist[end] == std::numeric_limits<Distance>::max()) {
      throw std::runtime_error("There is no path between vertexees");
      return {};
    }

    std::vector<Edge> path;
    Vertex current = end;
    while (current != start) {
      Vertex prev_vertex = prev[current];
      for (const Edge& edge : graph[prev_vertex]) {
        if (edge.to == current) {
          path.push_back(edge);
          break;
        }
      }
      current = prev_vertex;
    }
    std::reverse(path.begin(), path.end());
    return path;
  }
};
}  // namespace graph_space
#endif
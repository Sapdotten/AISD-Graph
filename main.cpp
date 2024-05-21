#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "graph.h"
using namespace std;
using namespace graph_space;

template <typename Vertex, typename Distance>
Distance calculate_shortest_path(
    vector<typename Graph<Vertex, Distance>::Edge> short_path) {
  Distance result(0);
  for (auto& e : short_path) {
    result += e.dist;
  }
  return result;
}

template <typename Vertex, typename Distance>
Vertex find_optimal_ware(Graph<Vertex, Distance>& g,
                         const Vertex& first_vertex) {
  if (g.order() < 1) throw runtime_error("Graph is empty");
  vector<Vertex> vertexes = g.walk(first_vertex);
  Vertex optimal;
  long double dist = numeric_limits<double>::max();
  for (auto& v : vertexes) {
    Distance sum_dist(0);
    int count_dist = 0;
    for (auto& to_v : vertexes) {
      if (to_v != v && g.has_edge(v, to_v)) {
        sum_dist += calculate_shortest_path<Vertex, Distance>(g.shortest_path(v, to_v));
        count_dist += 1;
      }
    }
    long double avg_dist = double(sum_dist) / count_dist;
    if (avg_dist < dist) {
      dist = avg_dist;
      optimal = v;
    }
  }
  return optimal;
}

int main() {
  Graph<int, int> g;
  g.add_edge(1, 4, 2);
  g.add_edge(1, 3, 4);
  g.add_edge(1, 2, 3);

  g.add_edge(2, 6, 3);

  g.add_edge(3, 6, 6);

  g.add_edge(4, 5, 5);
  g.add_edge(4, 6, 2);

  g.add_edge(5, 7, 6);
  g.add_edge(5, 9, 12);

  g.add_edge(6, 5, 1);
  g.add_edge(6, 7, 8);
  g.add_edge(6, 8, 7);

  g.add_edge(7, 10, 4);

  g.add_edge(8, 10, 3);

  g.add_edge(9, 8, 6);
  g.add_edge(9, 10, 11);

  cout << "Optimal place for ware: " << find_optimal_ware<int, int>(g, 1) << endl;
  vector<Graph<int, int>::Edge> arr = g.shortest_path(1, 10);
  for (auto& elem : arr) {
    cout << elem.from << "->" << elem.to << endl;
  }
  return 0;
}

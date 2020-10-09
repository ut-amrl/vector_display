//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    graph_domain.h
\brief   Domain definition for A* planner to use graphs loaded from files.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

// C headers.
#include <inttypes.h>

// C++ headers.
#include <algorithm>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <fstream>

// Library headers.
#include "eigen3/Eigen/Dense"
#include "glog/logging.h"
#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/Marker.h"

// Project headers.
#include "math/geometry.h"
#include "math/line2d.h"
#include "math/math_util.h"
#include "ros/ros_helpers.h"
#include "util/helpers.h"
#include "vector_map/vector_map.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#ifndef GRAPH_DOMAIN_H
#define GRAPH_DOMAIN_H

namespace navigation {

struct GraphDomain {
  struct State {
    uint64_t id;
    Eigen::Vector2f loc;
    State(uint64_t id, float x, float y) : id(id), loc(x, y) {}
    State(uint64_t id, const Eigen::Vector2f loc) : id(id), loc(loc) {}
    State() {}

    json toJSON() const {
      json data;
      data["id"] = id;
      data["loc"]["x"] = loc.x();
      data["loc"]["y"] = loc.y();
      return data;
    }

    static State fromJSON(const json& j) {
      State s;
      s.id = j["id"].get<uint64_t>();
      s.loc = Eigen::Vector2f(j["loc"]["x"].get<float>(), j["loc"]["y"].get<float>());
      return s;
    }
  };

  // V2 Map Edge structure.
  struct NavigationEdge {
    geometry::Line2f edge;
    uint64_t s0_id;
    uint64_t s1_id;
    float max_speed;
    float max_clearance;
    bool has_door;
    bool has_stairs;

    json toJSON() const {
      json data;
      data["s0_id"] = s0_id;
      data["s1_id"] = s1_id;
      data["max_speed"] = max_speed;
      data["max_clearance"] = max_clearance;
      data["has_door"] = has_door;
      data["has_stairs"] = has_stairs;
      // no need to save the edge, it's redundant data that can be recovered at load time
      return data;
    }

    // Constructing a navigation edge from JSON assumes you already know the states that exist in the world
    static NavigationEdge fromJSON(const json& j, const std::vector<State>& states) {
      NavigationEdge e;
      e.s0_id = j["s0_id"].get<uint64_t>();
      e.s1_id = j["s1_id"].get<uint64_t>();
      e.max_speed = j["max_speed"].get<float>();
      e.max_clearance = j["max_clearance"].get<float>();

      e.has_door = j["has_door"].get<bool>();
      e.has_stairs = j["has_stairs"].get<bool>();

      e.edge.p0 = states[e.s0_id].loc;
      e.edge.p1 = states[e.s1_id].loc;
      return e;
    }
  };


  GraphDomain() {}

  explicit GraphDomain(const std::string& map_file) {
    // Load graph from file.
    LoadV2(map_file);
  }

  State KeyToState(uint64_t key) const {
    DCHECK_LT(key, states.size());
    return states[key];
  }

  uint64_t StateToKey(const State& s) const {
    return s.id;
  }

  // Return the edge cost, assuming the two states are indeed connectable.
  float EdgeCost(const State& s0, const State& s1) const {
    return (s0.loc - s1.loc).norm();
  }

  float EdgeCost(const uint64_t k_s1, const uint64_t k_s2) const {
    DCHECK_LT(k_s1, states.size());
    DCHECK_LT(k_s2, states.size());
    return EdgeCost(states[k_s1], states[k_s2]);
  }

  float Heuristic(const State& s0, const State& s1) const {
    return (s0.loc - s1.loc).norm();
  }

  float Heuristic(const uint64_t k_s1, const uint64_t k_s2) const {
    DCHECK_LT(k_s1, states.size());
    DCHECK_LT(k_s2, states.size());
    return Heuristic(states[k_s1], states[k_s2]);
  }

  void GetNeighbors(const State& s,
                    std::vector<State>* neighbors) const {
    printf("%s unimplemented\n", __PRETTY_FUNCTION__);
  }

  // Get neighbors to a state.
  void GetNeighborsKeys(uint64_t s_key,
                        std::vector<uint64_t>* state_neighbors) const {
    CHECK_LT(s_key, states.size());
    CHECK_LT(s_key, neighbors.size());
    state_neighbors->clear();
    state_neighbors->insert(state_neighbors->end(),
                            neighbors[s_key].begin(),
                            neighbors[s_key].end());
  }

  void GrowIfNeeded(uint64_t id) {
    if (states.size() <= id) {
      states.resize(id + 1, State(0, 0, 0));
      neighbors.resize(id + 1);
    }
  }

  void ResetDynamicStates() {
    states = static_states;
    neighbors = static_neighbors;
  }

  uint64_t GetClosestState(const Eigen::Vector2f& v) {
    uint64_t closest_state = 0;
    float closest_sq_dist = FLT_MAX;
    for (size_t i = 0; i < states.size(); ++i) {
      const State& s = states[i];
      const float sq_dist = (s.loc - v).squaredNorm();
      if (sq_dist < closest_sq_dist) {
        closest_state = i;
        closest_sq_dist = sq_dist;
      }
    }
    return closest_state;
  }

  float GetClosestEdge(const Eigen::Vector2f& v,
                       uint64_t* p0_id_ptr,
                       uint64_t* p1_id_ptr) {
    uint64_t& p0 = *p0_id_ptr;
    uint64_t& p1 = *p1_id_ptr;
    float closest_dist = FLT_MAX;
    for (const State& s : states) {
      for (const uint64_t n : neighbors[s.id]) {
        const Eigen::Vector2f& a = s.loc;
        const Eigen::Vector2f& b = states[n].loc;
        const float dist = geometry::DistanceFromLineSegment(v, a, b);
        if (dist < closest_dist) {
          p0 = s.id;
          p1 = n;
          closest_dist = dist;
        }
      }
    }
    return closest_dist;
  }

  uint64_t AddState(const Eigen::Vector2f& v) {
    State s(states.size(), v);
    GrowIfNeeded(s.id);
    states[s.id] = s;
    return s.id;
  }

  void DeleteState(const uint64_t s_id) {
    // Delete all edges that touch this state.
    for (size_t i = 0; i < static_edges.size(); ++i) {
      if (static_edges[i].s0_id == s_id || static_edges[i].s1_id == s_id) {
        static_edges.erase(static_edges.begin() + i);
        --i;
      } else {
        if (static_edges[i].s0_id > s_id) {
          // Renumber static_edges after this state.
          --static_edges[i].s0_id;
        }
        if (static_edges[i].s1_id > s_id) {
          // Renumber static_edges after this state.
          --static_edges[i].s1_id;
        }
      }
    }
    // delete all entries for this state in other adjacency rows
    for (std::vector<uint64_t>& n : neighbors) {
      for (size_t i = 0; i < n.size(); ++i) {
        if (n[i] == s_id) {
          n.erase(n.begin() + i);
          --i;
        } else if (n[i] > s_id) {
          // Renumber states after this state.
          --n[i];
        }
      }
    }
    
    // Remove the adjacency row for this state.
    neighbors.erase(neighbors.begin() + s_id);
    // Remove this state.
    for (size_t i = 0; i < states.size(); ++i) {
      if (states[i].id == s_id) {
        states.erase(states.begin() + i);
        --i;
      } else if (states[i].id > s_id) {
        // Renumber states after this state.
        --states[i].id;
      }
    }
  }

  void DeleteUndirectedEdge(const uint64_t s0, const uint64_t s1) {
    for (uint64_t s : {s0, s1}) {
      std::vector<uint64_t>& n = neighbors[s];
      for (size_t j = 0; j < n.size(); ++j) {
        if ((s == s0 && n[j] == s1) || (s == s1 && n[j] == s0)) {
          n.erase(n.begin() + j);
        }
      }
    }
    // This edge must exist, hence the list can't be empty.
    CHECK_GT(static_edges.size(), 0);
    for (int i = static_edges.size() - 1; i >= 0; --i) {
      if ((static_edges[i].s0_id == s0 && 
          static_edges[i].s1_id == s1) || (static_edges[i].s1_id == s0 && 
          static_edges[i].s0_id == s1)) {
            printf("DELETING\n");
        static_edges.erase(static_edges.begin() + i);
      }
    }
  }

  bool NeighborExists(const std::vector<uint64_t>& neighbors,
                      const uint64_t s) {
    return (std::find(neighbors.begin(), neighbors.end(), s)
        != neighbors.end());
  }

  void AddDirectedEdge(const uint64_t s0, const uint64_t s1) {
    GrowIfNeeded(s0);
    GrowIfNeeded(s1);
    if (!NeighborExists(neighbors[s0], s1)) {
      neighbors[s0].push_back(s1);
    }
  }

  void AddUndirectedEdge(const uint64_t s0, 
                         const uint64_t s1,
                         const float max_speed,
                         const float max_clearance,
                         const bool has_door,
                         const bool has_stairs) {
    CHECK_LT(s0, states.size());
    CHECK_LT(s1, states.size());
    CHECK_LT(s0, neighbors.size());
    CHECK_LT(s1, neighbors.size());
    if (!NeighborExists(neighbors[s0], s1)) {
      neighbors[s0].push_back(s1);
    }
    if (!NeighborExists(neighbors[s1], s0)) {
      neighbors[s1].push_back(s0);
    }
    NavigationEdge e;
    e.s0_id = s0;
    e.s1_id = s1;
    e.max_speed = max_speed;
    e.max_clearance = max_clearance;
    e.edge.p0 = states[s0].loc;
    e.edge.p1 = states[s1].loc;
    e.has_door = has_door;
    e.has_stairs = has_stairs;
    static_edges.push_back(e);
  }

  void AddUndirectedEdge(const json& edge) {
    uint64_t s0 = edge["s0_id"].get<uint64_t>();
    uint64_t s1 = edge["s1_id"].get<uint64_t>();
    CHECK_LT(s0, states.size());
    CHECK_LT(s1, states.size());
    CHECK_LT(s0, neighbors.size());
    CHECK_LT(s1, neighbors.size());
    if (!NeighborExists(neighbors[s0], s1)) {
      neighbors[s0].push_back(s1);
    }
    if (!NeighborExists(neighbors[s1], s0)) {
      neighbors[s1].push_back(s0);
    }
    static_edges.push_back(NavigationEdge::fromJSON(edge, states));
  }

  void AddUndirectedEdge(const uint64_t s0, const uint64_t s1) {
    GrowIfNeeded(s0);
    GrowIfNeeded(s1);
    if (!NeighborExists(neighbors[s0], s1)) {
      neighbors[s0].push_back(s1);
    }
    if (!NeighborExists(neighbors[s1], s0)) {
      neighbors[s1].push_back(s0);
    }
  }

  uint64_t AddDynamicState(const Eigen::Vector2f& v) {
    static const bool kDebug = true;
    CHECK(!states.empty());
    // Find the closest Edge.
    uint64_t p0_id = 0, p1_id = 1;
    GetClosestEdge(v, &p0_id, &p1_id);
    const Eigen::Vector2f& p0 = states[p0_id].loc;
    const Eigen::Vector2f& p1 = states[p1_id].loc;
    if (kDebug) {
      printf("Adding %f,%f\n", v.x(), v.y());
      printf("p0: %lu (%f,%f)\n", p0_id, p0.x(), p0.y());
      printf("p1: %lu (%f,%f)\n", p1_id, p1.x(), p1.y());
    }
    // Find the projection of v on to the line segment p0 : p1;
    const Eigen::Vector2f pmid =
        geometry::ProjectPointOntoLineSegment(v, p0, p1);

    // Add p0 : pmid
    const uint64_t pmid_id = AddState(pmid);
    AddUndirectedEdge(p0_id, pmid_id);

    // Add p1 : pmid
    AddUndirectedEdge(p1_id, pmid_id);

    // Delete p0 : p1, since there is a p0 : pmid : p1 pathway now.
    DeleteUndirectedEdge(p0_id, p1_id);

    // Add pmid : v
    const uint64_t v_id = AddState(v);
    AddUndirectedEdge(pmid_id, v_id);

    printf("Adding dynamic state %f,%f (%lu) %lu %lu %lu\n",
        v.x(), v.y(), v_id, p0_id, p1_id, pmid_id);
    return v_id;
  }

  bool Save(const std::string& file) {
    ScopedFile fid(file, "w", true);
    for(uint32_t i = 0; i < states.size(); i++) {
      std::stringstream line;
      line << states[i].id << ", " << states[i].loc.x() << ", " << states[i].loc.y();
      // handle neighbors
      line << ", " << neighbors[i].size();
      for(uint32_t j = 0; j < neighbors[i].size(); j++) {
        line << ", " << neighbors[i][j];
      }
      line << std::endl;
      fputs(line.str().c_str(), fid());
    }
    return true;
  }

  // Save a V2 Map from V2 map structures.
  bool SaveV2(const std::string& file) {
    json j;
    std::vector<json> state_jsons;
    for (const State& s: states) {
      state_jsons.push_back(s.toJSON());
    }
    j["nodes"] = state_jsons;


    std::vector<json> edge_jsons;
    for (const NavigationEdge& e: static_edges) {
      edge_jsons.push_back(e.toJSON());
    }
    j["edges"] = edge_jsons;

    std::ofstream o(file);
    o << std::setw(4) << j << std::endl;
    o.close();
    return true;
  }

  void DrawMap() {
    printf("Map:\n======\n");
    for (const State& s : static_states) {
      printf("%lu: %8.3f,%8.3f", s.id, s.loc.x(), s.loc.y());
      CHECK_GT(static_neighbors.size(), s.id);
      for (const uint64_t n : static_neighbors[s.id]) {
        printf(" %4lu", n);
      }
      printf("\n");
    }
    printf("======\n");
  };

  // Load from a V2 map file.
  bool LoadV2(const std::string& file) {
    static const bool kDebug = true;
    std::ifstream i(file);
    json j;
    i >> j;
    i.close();
    
    CHECK(j["nodes"].is_array());
    auto const states_json = j["nodes"];

    states.clear();
    states.resize(states_json.size());
    neighbors.clear();
    neighbors.resize(states_json.size());
    static_edges.clear();
    
    for(const json& j : states_json) {
      State s = State::fromJSON(j);
      states[s.id] = s;    
    }

    CHECK(j["edges"].is_array());
    auto const edges_json = j["edges"];

    for(const json& j : edges_json) {
      AddUndirectedEdge(j);
    }
    
    printf("Loaded %s with %lu states, %lu edges\n",
            file.c_str(),
            states.size(),
            static_edges.size());

    static_states = states;
    static_neighbors = neighbors;
    if (kDebug) DrawMap();
    return true;
  }

  // Save a V2 Map from V1 map structures.
  bool SaveV2FromV1(const std::string& file) {
    // V1 Maps had no defined max speed, set it to some crazy high number.
    static const float kMaxSpeed = 50.0;
    // V1 Maps had no defined max clearance, set it to some crazy high number.
    static const float kMaxClearance = 10.0;

    ScopedFile fid(file, "w", true);

    std::vector<std::pair<uint64_t, uint64_t>> unique_edges;
    for(uint64_t i = 0; i < neighbors.size(); i++) {
      for (uint64_t j : neighbors[i]) {
        const auto e1 = std::make_pair(i, j);
        const auto e2 = std::make_pair(j, i);
        if (std::find(unique_edges.begin(), unique_edges.end(), e1) ==
            unique_edges.end() && 
            std::find(unique_edges.begin(), unique_edges.end(), e2) ==
            unique_edges.end()) {
          unique_edges.push_back(e1);
        }
      }
    }

    fprintf(fid(), "%lu\n", states.size());
    fprintf(fid(), "%lu\n", unique_edges.size());
    for (const State& s : states) {
      fprintf(fid(), "%lu, %f, %f\n", s.id, s.loc.x(), s.loc.y());
    }
    for (const std::pair<uint64_t, uint64_t> e : unique_edges) {
      fprintf(fid(), "%lu, %lu, %f, %f\n", 
          e.first, e.second, kMaxSpeed, kMaxClearance);
    }
    return true;
  }

  uint64_t GetClosestVertex(const Eigen::Vector2f& p) {
    float best_sq_dist = FLT_MAX;
    uint64_t best = 0;
    for (const State& s : states) {
      if ((s.loc - p).squaredNorm() <  best_sq_dist) {
        best_sq_dist = (s.loc - p).squaredNorm();
        best = s.id;
      }
    }
    return best;
  }

  void Load(const std::string& file) {
    static const bool kDebug = true;
    ScopedFile fid(file, "r", true);
    CHECK_NOTNULL(fid());
    bool valid = true;
    uint64_t id = 0;
    float x = 0, y = 0;
    int num_neighbors = 0;
    int num_edges = 0;
    states.clear();
    neighbors.clear();
    while (valid &&
          !feof(fid()) &&
          fscanf(fid(), "%lu, %f, %f, %d", &id, &x, &y, &num_neighbors) == 4) {
      GrowIfNeeded(id);
      states[id] = State(id, x, y);
      if (kDebug) {
        printf("Node %lu (%f,%f), with %d neighbors:\n",
               id,
               x,
               y,
               num_neighbors);
      }
      for (int i = 0; i < num_neighbors; ++i) {
        uint64_t n = 0;
        if (fscanf(fid(), ", %lu", &n) == 1) {
          if (kDebug) printf("%lu -> %lu\n", id, n);
          AddUndirectedEdge(id, n);
          ++num_edges;
        } else {
          if (kDebug) printf("\n");
          valid = false;
          break;
        }
      }
      // drawmap();
    }
    printf("Loaded %s with %d states, %d edges\n",
            file.c_str(),
            static_cast<int>(states.size()),
            num_edges);
    static_states = states;
    static_neighbors = neighbors;
    if (kDebug) DrawMap();
  }

  void GetClearanceAndSpeedFromLoc(const Eigen::Vector2f& p, 
                                   float* clearance, 
                                   float* speed) const {
    if (static_edges.empty()) return;
    float min_dist = FLT_MAX;
    NavigationEdge closest_edge = static_edges[0];
    for (const NavigationEdge& e : static_edges) {
      const float dist = e.edge.Distance(p);
      if (dist < min_dist) {
        closest_edge = e;
        min_dist = dist;
      }
    }
    printf("Closest (%.3f,%.3f): %lu (%.3f,%.3f) : %lu (%.3f,%.3f) %f %f \n",
           p.x(),
           p.y(),
           closest_edge.s0_id,
           closest_edge.edge.p0.x(),
           closest_edge.edge.p0.y(),
           closest_edge.s1_id,
           closest_edge.edge.p1.x(),
           closest_edge.edge.p1.y(),
           closest_edge.max_clearance,
           closest_edge.max_speed);

    if (clearance) *clearance = closest_edge.max_clearance;
    if (speed) *speed = closest_edge.max_speed;
  }

  std::vector<State> states;
  std::vector<std::vector<uint64_t> > neighbors;
  std::vector<State> static_states;
  std::vector<std::vector<uint64_t> > static_neighbors;
  // V2 Map edges.
  std::vector<NavigationEdge> static_edges;
};


}  // namespace navigation
#endif  // GRAPH_DOMAIN_H

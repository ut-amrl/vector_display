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
    Load(map_file);
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
    printf("%s unimplemented\n", __PRETTY_FUNCTION__);          
  }

  void GrowIfNeeded(uint64_t id) {
    if (states.size() <= id) {
      states.resize(id + 1, State(0, 0, 0));
    }
  }

  void ResetDynamicStates() {
    printf("%s unimplemented\n", __PRETTY_FUNCTION__);
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

  float GetClosestEdge(const Eigen::Vector2f& v, NavigationEdge& closest_edge) const {
    float closest_dist = FLT_MAX;
    closest_edge.s0_id = -1;
    closest_edge.s1_id = -1;
    if (edges.empty()) return closest_dist;
    for (const NavigationEdge& e : edges) {
      const float dist = e.edge.Distance(v);
      if (dist < closest_dist) {
        closest_dist = dist;
        closest_edge = e;
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
    for (size_t i = 0; i < edges.size(); ++i) {
      if (edges[i].s0_id == s_id || edges[i].s1_id == s_id) {
        edges.erase(edges.begin() + i);
        --i;
      } else {
        if (edges[i].s0_id > s_id) {
          // Renumber edges after this state.
          --edges[i].s0_id;
        }
        if (edges[i].s1_id > s_id) {
          // Renumber edges after this state.
          --edges[i].s1_id;
        }
      }
    }
    
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
    // This edge must exist, hence the list can't be empty.
    CHECK_GT(edges.size(), 0);
    for (int i = edges.size() - 1; i >= 0; --i) {
      if ((edges[i].s0_id == s0 && 
          edges[i].s1_id == s1) || (edges[i].s0_id == s1 && 
          edges[i].s1_id == s0)) {
        edges.erase(edges.begin() + i);
      }
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
    NavigationEdge e;
    e.s0_id = s0;
    e.s1_id = s1;
    e.max_speed = max_speed;
    e.max_clearance = max_clearance;
    e.edge.p0 = states[s0].loc;
    e.edge.p1 = states[s1].loc;
    e.has_door = has_door;
    e.has_stairs = has_stairs;
    edges.push_back(e);
  }

  void AddUndirectedEdge(const json& edge) {
    uint64_t s0 = edge["s0_id"].get<uint64_t>();
    uint64_t s1 = edge["s1_id"].get<uint64_t>();
    CHECK_LT(s0, states.size());
    CHECK_LT(s1, states.size());
    edges.push_back(NavigationEdge::fromJSON(edge, states));
  }

  // This should only be called while upgrading v1 maps
  void AddUndirectedEdge(const uint64_t s0, const uint64_t s1) {
    GrowIfNeeded(s0);
    GrowIfNeeded(s1);
    NavigationEdge e;
    e.s0_id = s0;
    e.s1_id = s1;
    e.max_speed = 50.0;
    e.max_clearance = 10.0;
    e.edge.p0 = states[s0].loc;
    e.edge.p1 = states[s1].loc;
    e.has_door = false;
    e.has_stairs = false;
    edges.push_back(e);
  }

  uint64_t AddDynamicState(const Eigen::Vector2f& v) {
    printf("%s unimplemented\n", __PRETTY_FUNCTION__);
    return 0;
  }

  // Save a V2 Map from V2 map structures.
  bool Save(const std::string& file) {
    json j;
    std::vector<json> state_jsons;
    for (const State& s: states) {
      state_jsons.push_back(s.toJSON());
    }
    j["nodes"] = state_jsons;


    std::vector<json> edge_jsons;
    for (const NavigationEdge& e: edges) {
      edge_jsons.push_back(e.toJSON());
    }
    j["edges"] = edge_jsons;

    std::ofstream o(file);
    o << std::setw(4) << j << std::endl;
    o.close();
    return true;
  }

  void DrawMap() {
    printf("Map:\n===Nodes===\n");
    for (const State& s : states) {
      printf("%lu: %8.3f,%8.3f\n", s.id, s.loc.x(), s.loc.y());
    }
    printf("===Edges===\n");
    for(const NavigationEdge& e : edges) {
      printf("%lu <-> %lu\n", e.s0_id, e.s1_id);
    }
  };

  // Load from a V2 map file.
  bool Load(const std::string& file) {
    static const bool kDebug = true;
    std::ifstream i(file);
    json j;
    i >> j;
    i.close();
    
    CHECK(j["nodes"].is_array());
    auto const states_json = j["nodes"];

    states.clear();
    states.resize(states_json.size());
    edges.clear();
    
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
            edges.size());

    if (kDebug) DrawMap();
    return true;
  }

  // Save a V2 Map from V1 map structures.
  bool SaveV2FromV1(const std::string& in_file, const std::string& out_file) {
    ScopedFile fid(in_file, "r", true);
    CHECK_NOTNULL(fid());
    bool valid = true;
    uint64_t id = 0;
    float x = 0, y = 0;
    int num_edges = 0;
    int num_neighbors = 0;
    states.clear();
    while (valid &&
          !feof(fid()) &&
          fscanf(fid(), "%lu, %f, %f, %d", &id, &x, &y, &num_neighbors) == 4) {
      GrowIfNeeded(id);
      states[id] = State(id, x, y);
      for (int i = 0; i < num_neighbors; ++i) {
        uint64_t n = 0;
        if (fscanf(fid(), ", %lu", &n) == 1) {
          AddUndirectedEdge(id, n);
          ++num_edges;
        } else {
          valid = false;
          break;
        }
      }
    }

    printf("Loaded %s with %d states, %d edges\n",
            in_file.c_str(),
            static_cast<int>(states.size()),
            num_edges);

    DrawMap();
    
    Save(out_file);
    
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

  void GetClearanceAndSpeedFromLoc(const Eigen::Vector2f& p, 
                                   float* clearance, 
                                   float* speed) const {
    if (edges.empty()) return;
    float min_dist = FLT_MAX;
    NavigationEdge closest_edge = edges[0];
    for (const NavigationEdge& e : edges) {
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
  // V2 Map edges.
  std::vector<NavigationEdge> edges;
};


}  // namespace navigation
#endif  // GRAPH_DOMAIN_H

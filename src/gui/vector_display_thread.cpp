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
\file    vector_display_thread.cpp
\brief   Thread to run the GUI for Vector Localization; C++ Interface: VectorDisplayThread
\author  Joydeep Biswas, (C) 2010
 */
//========================================================================

#include <eigen3/Eigen/Dense>
#include <stdio.h>
#include <string>
#include <vector>
#include <QtWidgets/QInputDialog>
#include <QtWidgets/QMessageBox>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "gflags/gflags.h"

#include "amrl_msgs/Localization2DMsg.h"
#include "vector_display.h"
#include "vector_display_thread.h"

#include "shared/ros/ros_helpers.h"
#include "shared/util/helpers.h"
#include "shared/util/timer.h"

using namespace geometry;
using namespace math_util;
using Eigen::Vector2f;
using Eigen::Rotation2Df;
using amrl_msgs::Localization2DMsg;
using amrl_msgs::VisualizationMsg;
using geometry_msgs::PoseWithCovarianceStamped;
using std::min;
using std::max;
using std::size_t;
using std::string;
using std::vector;

#define V2COMP(v) v.x, v.y

DEFINE_string(maps_dir, "maps", "Directory to load maps from");
DEFINE_bool(edit_localization, false, "Edit localization map");
DEFINE_bool(edit_navigation, false, "Edit navigation map");
DEFINE_bool(edit_semantic, false, "Edit semantic map");
DEFINE_bool(view_navmap, false, "View navigation map");
DEFINE_bool(autoswitch_map, false,
    "Automatically switch maps based on localization");
DEFINE_bool(live, false, "Live view");
DEFINE_bool(test, false, "Test mode");
DEFINE_string(map, "UT_Campus", "Initial localization map to load");
DEFINE_double(max_fps, 60.0, "Maximum graphics refresh rate");

DEFINE_string(localization_topic, "localization", "Topic to listen for localization messages on");
DEFINE_string(laser_topic, "scan", "Topic to listen for laser messages on");
DEFINE_string(visualization_topic, "visualization", "Topic to listen for visualization messages on");

string MapnameToLocalizationFilename(const string& map) {
  return StringPrintf("%s/%s/%s.vectormap.txt",
                      FLAGS_maps_dir.c_str(),
                      map.c_str(),
                      map.c_str());
}

string MapnameToNavigationFilenameJson(const string& map) {
  return StringPrintf("%s/%s/%s.navigation.json",
                      FLAGS_maps_dir.c_str(),
                      map.c_str(),
                      map.c_str());
}

string MapnameToNavigationFilenameTxt(const string& map) {
  return StringPrintf("%s/%s/%s.navigation.txt",
                      FLAGS_maps_dir.c_str(),
                      map.c_str(),
                      map.c_str());
}


bool VectorDisplayThread::GetSemanticType(
    const vector<string>& types,
    string* selected_type) {
  QStringList types_list;
  int old_type = 0;
  for (size_t i = 0; i < types.size(); ++i) {
    types_list << tr(types[i].c_str());
    if (types[i] == (*selected_type)) {
      old_type = i;
    }
  }
  bool ok = false;
  *selected_type = QInputDialog::getItem(
      display,
      tr("Semantic Type"),
      tr("Type:"),
      types_list,
      old_type,
      false,
      &ok).toStdString();
  return ok;
}

bool VectorDisplayThread::GetSemanticTypeAndLabel(
    const vector<string>& types,
    string* selected_type,
    string* label) {
  if (!GetSemanticType(types, selected_type)) {
    return false;
  }
  bool ok = false;
  *label = QInputDialog::getText(
      display,
      tr("Semantic Label"),
      tr("Label:"),
      QLineEdit::Normal,
      tr((*label).c_str()),
      &ok).toStdString();
  return ok;
}

bool VectorDisplayThread::GetNavEdgeParams(
    float* width,
    float* max_speed,
    bool* has_door,
    bool* has_stairs, 
    bool* has_elevator, 
    bool* has_automated_door) {
  bool ok = false;
  *width = QInputDialog::getDouble(
      display,
      tr("Set Edge Width"),
      tr("Edge Width:"),
      *width,
      0,
      100,
      2,
      &ok);
  if (!ok) return false;
  *max_speed = QInputDialog::getDouble(
      display,
      tr("Set Max Speed"),
      tr("Max Speed:"),
      *max_speed,
      0,
      2.0,
      2,
      &ok);
  if (!ok) return false;
  QStringList bool_types;
  bool_types << tr("True");
  bool_types << tr("False");
  const string has_door_str = QInputDialog::getItem(
      display,
      tr("Has Door"),
      tr("Has Door:"),
      bool_types,
      ((*has_door) ? 0 : 1),
      false,
      &ok).toStdString();
  *has_door = (has_door_str == "True");

  const string has_stairs_str = QInputDialog::getItem(
      display,
      tr("Has Stairs"),
      tr("Has Stairs:"),
      bool_types,
      ((*has_stairs) ? 0 : 1),
      false,
      &ok).toStdString();
  *has_stairs = (has_stairs_str == "True");

  const string has_elevator_str = QInputDialog::getItem(
      display,
      tr("Has Elevator"),
      tr("Has Elevator:"),
      bool_types,
      ((*has_elevator) ? 0 : 1),
      false,
      &ok).toStdString();
  *has_elevator = (has_elevator_str == "True");


  const string has_automated_door_str = QInputDialog::getItem(
      display,
      tr("Has Automated Door"),
      tr("Has Automated Door:"),
      bool_types,
      ((*has_automated_door) ? 0 : 1),
      false,
      &ok).toStdString();
  *has_automated_door = (has_automated_door_str == "True");
  return ok;
}

void VectorDisplayThread::ChangeMap() {
  QStringList maps;
  {
    ScopedFile fid (FLAGS_maps_dir + "/atlas.txt", "r");
    if (fid() == NULL) {
      fprintf(stderr, "Error: Unable to load atlas!\n");
      return;
    }
    vector<char> map_entry(4096, 0);
    int map_index = 0;
    while (fscanf(fid, "%d %s", &map_index, map_entry.data()) == 2) {
      maps << tr(map_entry.data());
      map_entry = vector<char>(4096, 0);
    }
  }
  bool ok = false;
  QString map_name = QInputDialog::getItem(
      display, tr("Load Map"), tr("Map:"), maps, 0, false, &ok);
  if (ok && !map_name.isEmpty()) {
    const string localization_map_file =
        MapnameToLocalizationFilename(map_name.toStdString());
    const string navigation_map_file =
        MapnameToNavigationFilenameJson(map_name.toStdString());
    if (FLAGS_autoswitch_map && localization_map_file != vectorMap.file_name) {
      QMessageBox confirmBox;
      confirmBox.setWindowTitle("Confirm");
      confirmBox.setText("Localization auto-update is on.");
      confirmBox.setInformativeText("Do you want to turn off auto-update and change maps?");
      confirmBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
      confirmBox.setDefaultButton(QMessageBox::Ok);
      int ret = confirmBox.exec();
      if (ret == QMessageBox::Ok) {
        FLAGS_autoswitch_map = false;
      } else {
        return;
      }
    }
    if (FLAGS_edit_localization) {
      if (vectorMap.Save(MapnameToLocalizationFilename(map_name_))) {
        printf("Saved vector map %s\n", map_name_.c_str());
      } else {
        printf("Error saving vector map %s\n", map_name_.c_str());
        return;
      }
    }
    if (FLAGS_edit_navigation) {
      if (navMap.Save(MapnameToNavigationFilenameJson(map_name_))) {
        printf("Saved navigation map %s\n", map_name_.c_str());
      } else {
        printf("Error saving navigation map %s\n", map_name_.c_str());
        return;
      }
      navMap.Load(navigation_map_file);
    }
    if (FLAGS_view_navmap) {
      navMap.Load(navigation_map_file);
    }
    printf("Change map to %s\n", map_name.toStdString().c_str());
    vectorMap.Load(localization_map_file);
    map_name_ = map_name.toStdString();
  }
}

void VectorDisplayThread::KeyboardEventCallback(
    uint32_t key_code,uint32_t modifiers) {
  switch (key_code) {
    case Qt::Key_R : {
      compileDisplay();
      break;
    }

    case Qt::Key_C : {
      clearDisplay = true;
    } break;

    case Qt::Key_M : {
      ChangeMap();
    } break;

    case Qt::Key_U : {
      FLAGS_autoswitch_map = !FLAGS_autoswitch_map;
      printf("FLAGS_autoswitch_map: %d\n", FLAGS_autoswitch_map);
    } break;

    case Qt::Key_N : {
      printf("Number of lines in map %s: %i\n", map_name_.c_str(),
             static_cast<int>(vectorMap.lines.size()));
    } break;
  }
}

void VectorDisplayThread::editMap(
    const Vector2f& mouse_down, const Vector2f& mouse_up, float orientation,
    float viewScale,
    uint32_t modifiers) {
  const Vector2f p0 = mouse_down;
  const Vector2f p1 = mouse_up;

  std::cout << "MODIFIERS: " << modifiers << std::endl;

  switch (modifiers) {
    case 0x04: {
      // Add Line
      vectorMap.AddLine(p0, p1);
      compileDisplay();
    } break;

    case 0x02: {
      // Delete Line
      static const float kMaxError = 4.0f * viewScale;
      int best_match = -1;
      float best_match_error = FLT_MAX;
      for (size_t i = 0; i < vectorMap.lines.size(); ++i) {
        const Line2f& l = vectorMap.lines[i];
        const float match_error = l.Distance(p0);
        if (match_error < best_match_error && match_error < kMaxError) {
          best_match = i;
          best_match_error = match_error;
        }
      }
      if (best_match > -1) {
        vectorMap.lines.erase(vectorMap.lines.begin() + best_match);
        compileDisplay();
      }
    } break;
  }
}

void VectorDisplayThread::editGraph(
    const Vector2f& mouse_down, const Vector2f& mouse_up,
    float orientation, float viewScale, uint32_t modifiers) {

  const Vector2f p0 = mouse_down;
  const Vector2f p1 = mouse_up;
  static const float kMaxError = 4.0f * viewScale;

  // Check if an edge was selected.
  navigation::GraphDomain::NavigationEdge closest_edge;
  float nearest_edge_dist = FLT_MAX;
  bool found_edge = navMap.GetClosestEdge(p0, &closest_edge, &nearest_edge_dist);
  const bool down_on_edge = found_edge && nearest_edge_dist < kMaxError;

  // Check if the mouse down location was near a vertex.
  uint64_t nearest_vertex_down = 0;
  bool found_down_state = navMap.GetClosestState(p0, &nearest_vertex_down);

  float down_vertex_dist;
  bool down_on_vertex;
  if (nearest_vertex_down < navMap.states.size()) {
    down_vertex_dist = (navMap.KeyToState(nearest_vertex_down).loc - p0).norm();
    down_on_vertex = found_down_state && down_vertex_dist < kMaxError;
  } else {
    down_on_vertex = false;
    down_vertex_dist = FLT_MAX;
  }

  // Check if the mouse up location was near a vertex.
  uint64_t nearest_vertex_up = 0;
  bool found_up_state = navMap.GetClosestState(p1, &nearest_vertex_up);
  float up_vertex_dist;
  bool up_on_vertex;
  if (nearest_vertex_up < navMap.states.size()) {
    up_vertex_dist =
      (navMap.KeyToState(nearest_vertex_up).loc - p1).norm();
    up_on_vertex = found_up_state && up_vertex_dist < kMaxError;
  } else {
    up_on_vertex = false;
    up_vertex_dist = FLT_MAX;
  }
  
  const bool click = ((p0 - p1).norm() < kMaxError);

  vector<string> semantic_vertex_types;
  semantic_vertex_types.push_back("Office");
  semantic_vertex_types.push_back("Other");
  semantic_vertex_types.push_back("Stair");
  semantic_vertex_types.push_back("Bathroom");
  semantic_vertex_types.push_back("Elevator");
  semantic_vertex_types.push_back("Kitchen");
  semantic_vertex_types.push_back("Printer");
  semantic_vertex_types.push_back("MapExit");

  vector<string> semantic_edge_types;
  semantic_edge_types.push_back("Hallway");
  semantic_edge_types.push_back("Vertical");
  semantic_edge_types.push_back("MapExit");
  const bool dragged_between_vertices =
      down_on_vertex && up_on_vertex &&
      nearest_vertex_down != nearest_vertex_up;

  switch(modifiers) {
    case 0x04: { // Shift
      // Add Edge or Vertex
      if (!dragged_between_vertices && !down_on_vertex) {
        if (FLAGS_edit_navigation) {
          navMap.AddState(p0);
        } else if (FLAGS_edit_semantic) {
          string roomType;
          string roomLabel;
          if (GetSemanticTypeAndLabel(semantic_vertex_types, &roomType, &roomLabel)) {
            printf("TODO: Semantic map editing\n");
          }
        }
      } else if (dragged_between_vertices) {
        // add edge if drag from one vertex to another
        // using default values for width, max_speed, and has_door
        if (FLAGS_edit_navigation) {
          float width = 1;
          float max_speed = 10;
          bool has_door = false;
          bool has_stairs = false;
	  bool has_elevator = false;
	  bool has_automated_door = false;
          if (GetNavEdgeParams(&width, &max_speed, &has_door, &has_stairs, &has_elevator, &has_automated_door)) {
            navMap.AddUndirectedEdge(
                nearest_vertex_down, nearest_vertex_up, max_speed, width, has_door, has_stairs, has_elevator, has_automated_door);
          }
        } else if (FLAGS_edit_semantic) {
          string edgeType;
          if (GetSemanticType(semantic_edge_types, &edgeType)) {
            printf("TODO: Semantic map editing\n");
          }
        }
      } else {
        return;
      }
      compileDisplay();
    } break;

    case 0x02: { // Control
      // Delete Edge or Vertex
      if (click && down_on_vertex) {
        navMap.DeleteState(nearest_vertex_down);
      } else if (click && down_on_edge) {
        navMap.DeleteUndirectedEdge(closest_edge.s0_id, closest_edge.s1_id);
      } else {
        return;
      }
      compileDisplay();
    } break;

    case 0x01: { // Alt
      // Move edge or vertex
      if (down_on_vertex &&
          (!down_on_edge || down_vertex_dist < nearest_edge_dist)) {
        navMap.KeyToState(nearest_vertex_down).loc = p1;
      } else if (down_on_edge) {
        Vector2f shift = p1 - p0;
        navMap.states[closest_edge.s0_id].loc += shift;
        navMap.states[closest_edge.s1_id].loc += shift;
      } else {
        return;
      }
      compileDisplay();
    } break;

    case 0x03: { // Ctrl-Alt
      // Edit parameters of edge or vertex
      if (down_on_vertex && FLAGS_edit_semantic) {
        string roomType;
        string roomLabel;
        if (GetSemanticTypeAndLabel(semantic_vertex_types, &roomType, &roomLabel)) {
          printf("TODO: Semantic map editing\n");
        }
      } else if (!down_on_vertex && down_on_edge) {
        if (FLAGS_edit_navigation) {
          float width = 0;
          float max_speed = 0;
          bool has_door = false;
          bool has_stairs = false;
	  bool has_elevator = false;
	  bool has_automated_door = false;
          if (GetNavEdgeParams(&width, &max_speed, &has_door, &has_stairs, &has_elevator, &has_automated_door)) {
            printf("TODO: Semantic map editing\n");
          }
        } else if (FLAGS_edit_semantic) {
          string edgeType;
          if (GetSemanticType(semantic_edge_types, &edgeType)) {
            printf("TODO: Semantic map editing\n");
          }
        }
      }
      compileDisplay();
    } break;
  }
}

void VectorDisplayThread::MouseEventCallback(
    const Vector2f& mouse_down,
    const Vector2f& mouse_up, float orientation,
    float viewScale,
    uint32_t modifiers) {
  static const bool debug = true;

  if (FLAGS_edit_localization) {
    editMap(mouse_down, mouse_up, orientation, viewScale, modifiers);
    if (modifiers == 0x01) {
      printf("Length: %f\n", (mouse_down - mouse_up).norm());
    }
    return;
  }

  if (FLAGS_edit_navigation || FLAGS_edit_semantic) {
    editGraph(mouse_down, mouse_up, orientation, viewScale, modifiers);
    return;
  }
  {
    switch (modifiers) {
      case 0x04: {
        // Set Position
        set_pose_msg.header.stamp = ros::Time::now();
        set_pose_msg.map = map_name_;
        set_pose_msg.pose.x = mouse_down.x();
        set_pose_msg.pose.y = mouse_down.y();
        set_pose_msg.pose.theta = orientation;
        if (debug) {
          printf("SetPosition: %7.3f, %7.3f %6.1f\u00b0\n",
              mouse_down.x(), mouse_down.y(), RadToDeg(orientation));
        }
        initialPosePublisher.publish(set_pose_msg);
      } break;
      case 0x02: {
        // Set Target
        nav_target_msg_.header.stamp = ros::Time::now();
        nav_target_msg_.pose.position.x = mouse_down.x();
        nav_target_msg_.pose.position.y = mouse_down.y();
        nav_target_msg_.pose.orientation.w = cos(0.5f * orientation);
        nav_target_msg_.pose.orientation.z = sin(0.5f * orientation);
        if (debug) {
          printf("SetTarget: %7.3f, %7.3f %6.1f\u00b0\n",
              mouse_down.x(), mouse_down.y(), RadToDeg(orientation));
        }
        nav_goal_pub_.publish(nav_target_msg_);
      } break;
      case 0x01: {
        // Information
        printf("Mouse down: %7.3f, %7.3f\n",
              mouse_down.x(), mouse_down.y());
        printf("Mouse up: %7.3f, %7.3f\n",
              mouse_up.x(), mouse_up.y());
        printf("Length: %7.3f Angle: %7.4f (%7.3f\u00b0)\n",
              (mouse_down - mouse_up).norm(), orientation,
              RadToDeg(orientation));
      } break;
    }
  }
}

void VectorDisplayThread::Zoom(float zoom) {
  if (display) display->Zoom(zoom);
}

void VectorDisplayThread::drawMap(
    vector<VectorDisplay::Line>* lines,
    vector<VectorDisplay::Color>* lineColors) {
  for (const Line2f& l : vectorMap.lines) {
    lines->push_back(VectorDisplay::Line(l.p0, l.p1));
    lineColors->push_back(VectorDisplay::Color(0.32, 0.49, 0.91, 1.0));
  }
}

void VectorDisplayThread::LocalizationCallback(const Localization2DMsg& msg) {
  robotLoc = Vector2f(msg.pose.x, msg.pose.y);
  robotAngle = msg.pose.theta;
  const string localization_map_file =
      MapnameToLocalizationFilename(msg.map);
  const string navigation_map_file =
      MapnameToNavigationFilenameJson(msg.map);
  if (FLAGS_autoswitch_map && map_name_ != msg.map) {
    if (FLAGS_edit_localization) {
      if (vectorMap.Save(localization_map_file)) {
        printf("Saved map %s\n", map_name_.c_str());
      } else {
        printf("Error saving map %s\n", map_name_.c_str());
        return;
      }
    }
    if (FLAGS_edit_navigation) {
      if (navMap.Save(navigation_map_file)) {
        printf("Saved navigation map %s\n", map_name_.c_str());
      } else {
        printf("Error saving navigation map %s\n", map_name_.c_str());
        return;
      }
      navMap.Load(navigation_map_file);
    }
    if (FLAGS_edit_semantic) {
      // if (navMap.SaveSemanticMap(map_name_)) {
      //   printf("Saved semantic map %s\n", map_name_.c_str());
      // } else {
      //   printf("Error saving semantic map %s\n", map_name_.c_str());
      //   return;
      // }
      // navMap.LoadSemanticMap(msg.map.c_str());
      printf("TODO: Semantic map editing\n");
    }
    if (FLAGS_view_navmap) {
      navMap.Load(navigation_map_file);
    }

    vectorMap.Load(localization_map_file);
    map_name_ = msg.map;
  }
  compileDisplay();
}

void VectorDisplayThread::kinectScanCallback(
    const sensor_msgs::LaserScan& msg) {
  kinectScanMsg = msg;
  compileDisplay();
}

void VectorDisplayThread::laserCallback(const sensor_msgs::LaserScan& msg) {
  laserScanMsg = msg;
  tLaser = GetWallTime();
  compileDisplay();
}

void VectorDisplayThread::filteredPointCloudCallback(
    const sensor_msgs::PointCloud& msg) {
  pointCloudMsg = msg;
  tPointCloud = GetWallTime();
  compileDisplay();
}

void VectorDisplayThread::visualizationCallback(
      const ros::MessageEvent<const amrl_msgs::VisualizationMsg>& msgEvent) {
  static const bool debug = false;
  const amrl_msgs::VisualizationMsgConstPtr &msg = msgEvent.getConstMessage();
  bool duplicate = false;
  unsigned int i = 0;
  const string source_name =
      msgEvent.getPublisherName() + ":" + msg->header.frame_id;
  if (debug) {
    printf("Received message from %s\n", source_name.c_str());
  }

  for (; i < displayProviders.size() && !duplicate; i++) {
    if (displayProviders[i].compare(source_name)== 0)
      duplicate = true;
  }
  if (debug) printf("Duplicate:%d, i:%d\n", duplicate, i);
  if (duplicate) {
    i--;
    visualizationMsgs[i] = *msg;
  } else {
    visualizationMsgs.push_back(*msg);
    displayProviders.push_back(source_name);
  }
  compileDisplay();
}

void VectorDisplayThread::clearDisplayMessages() {
  displayProviders.clear();
  laserScanMsg.ranges.clear();
  kinectScanMsg.ranges.clear();
}

void VectorDisplayThread::DrawNavigationMap() {
  VectorDisplay::Color roomLabel(0.0, 0.0, 0.0, 1.0);
  for(unsigned int i=0; i<navMap.states.size(); ++i) {
    Vector2f v = navMap.states[i].loc;
    points.push_back(v);
    pointColors.push_back(VectorDisplay::Color(0xFF008800));
  }
  for(unsigned int i=0; i < navMap.edges.size(); ++i) {
    lines.push_back(VectorDisplay::Line(navMap.edges[i].edge.p0, navMap.edges[i].edge.p1));
    lineColors.push_back(VectorDisplay::Color(0xFFFF00FF));
  }
  if (FLAGS_edit_semantic) {
    // TODO: Draw Semantic map
  }
}

void VectorDisplayThread::compileDisplay() {
  static double tLast = 0.0;
  static const double MessageTimeout = 1.0;
  static const VectorDisplay::Color LidarPointColor(0xFFF0761F);
  static const VectorDisplay::Color KinectScanColor(0xFFFF0505);
  static const VectorDisplay::Color PointCloudColor(0xFFDE2352);
  static const bool debug = false;

  if (debug) printf("GUI updated!\n");
  if (GetMonotonicTime()-tLast< 1.0 / FLAGS_max_fps) return;
  tLast = GetMonotonicTime();

  lines.clear();
  points.clear();
  circles.clear();
  quads.clear();
  arcs.clear();
  circleColors.clear();
  lineColors.clear();
  pointColors.clear();
  quadColors.clear();
  textColors.clear();
  textStrings.clear();
  textLocs.clear();
  textHeights.clear();
  textInWindowCoords.clear();

  drawMap(&lines, &lineColors);

  if (FLAGS_edit_navigation || FLAGS_edit_semantic || FLAGS_view_navmap) {
    DrawNavigationMap();
  }

  const Eigen::Affine2f robot_to_map_tf =
      Eigen::Translation2f(robotLoc) * Eigen::Rotation2Df(robotAngle);
  for (const VisualizationMsg& msg : visualizationMsgs) {
    const Eigen::Affine2f tf = ((msg.header.frame_id == "base_link") ?
        robot_to_map_tf : Eigen::Affine2f::Identity());
    const float angle_tf = ((msg.header.frame_id == "base_link") ?
        robotAngle : 0);
    for (const amrl_msgs::ColoredLine2D& l : msg.lines) {
      const Vector2f p0 = tf * Vector2f(l.p0.x, l.p0.y);
      const Vector2f p1 = tf * Vector2f(l.p1.x, l.p1.y);
      lines.push_back(VectorDisplay::Line(p0, p1));
      lineColors.push_back(VectorDisplay::Color(l.color | 0xFF000000ull));
    }
    for (const amrl_msgs::ColoredPoint2D& p : msg.points) {
      const Vector2f p0 = tf * Vector2f(p.point.x, p.point.y);
      points.push_back(p0);
      pointColors.push_back(VectorDisplay::Color(p.color | 0xFF000000ull));
    }
    for (const amrl_msgs::ColoredArc2D& a : msg.arcs) {
      arcs.push_back(VectorDisplay::ColoredArc(
          tf * Vector2f(a.center.x, a.center.y),
          a.radius,
          a.start_angle + angle_tf,
          a.end_angle + angle_tf,
          VectorDisplay::Color(a.color | 0xFF000000ull)));
    }
  }

  if (debug) {
    printf("lines: %d points: %d circles: %d\n",
        static_cast<int>(lines.size()), static_cast<int>(points.size()),
        static_cast<int>(circles.size()));
  }

  if ((GetWallTime()-tPointCloud < MessageTimeout || persistentDisplay)
      && FLAGS_live) {
    const Rotation2Df robot_angle(robotAngle);
    for (size_t i = 0; i < pointCloudMsg.points.size(); ++i) {
      static const float kMaxNormalAngle = RadToDeg(30.0);
      static const float kMaxNormalCosine = cos(kMaxNormalAngle);
      static const size_t kZNormalIndex = 2;
      if (fabs(pointCloudMsg.channels[kZNormalIndex].values[i]) >
          kMaxNormalCosine) {
        continue;
      }
      const Vector2f point =
          robot_angle * Vector2f(V2COMP(pointCloudMsg.points[i])) + robotLoc;
      points.push_back(point);
      pointColors.push_back(PointCloudColor);
    }
  }
  if (FLAGS_live) {
    size_t i = 0;
    float a = 0.0;
    Vector2f p(0.0, 0.0);
    for (i = 0, a = robotAngle + kinectScanMsg.angle_min;
        i < kinectScanMsg.ranges.size();
        i++, a += kinectScanMsg.angle_increment) {
      if (kinectScanMsg.ranges[i]<= kinectScanMsg.range_min ||
          kinectScanMsg.ranges[i]>= kinectScanMsg.range_max) {
        continue;
      }
      p = Rotation2Df(a) * Vector2f(1.0, 0.0) * kinectScanMsg.ranges[i];
      points.push_back(robotLoc + p);
      pointColors.push_back(KinectScanColor);
    }
  }
  if ((GetWallTime()-tLaser < MessageTimeout || persistentDisplay)
      && FLAGS_live) {
    unsigned int i = 0;
    float a = 0.0;
    // TODO(joydeepb): Load this from a config file.
    const Vector2f laserLoc =
        Rotation2Df(robotAngle) * Vector2f(0.05, 0.0) + robotLoc;
    for (i = 0, a = robotAngle + laserScanMsg.angle_min;
        i < laserScanMsg.ranges.size();
        i++, a+= laserScanMsg.angle_increment) {
      if (laserScanMsg.ranges[i]<= laserScanMsg.range_min ||
          laserScanMsg.ranges[i]>= laserScanMsg.range_max) {
        continue;
      }
      const Vector2f p =
          Rotation2Df(a) * Vector2f(1.0, 0.0) * laserScanMsg.ranges[i] +
          laserLoc;
      points.push_back(p);
      pointColors.push_back(LidarPointColor);
    }
  }
  display->updateDisplay(
      robotLoc, robotAngle, 100.0, lines, points, circles, quads,
      lineColors, pointColors, circleColors, quadColors, arcs,
      textLocs, textStrings, textHeights, textColors, textInWindowCoords);
}

void VectorDisplayThread::TestMode() {
  static const bool kTextOnly = false;
  static const bool debug = false;
  vector<bool> text_in_window_coords;
  int char_offset = 0;
  string str(
    "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Donec au "
    "metus risus, in scelerisque lorem pharetra eget. Aliquam facilisi\n"
    "luctus mauris condimentum ornare. Morbi lacinia enim ac dictum tpo "
    "\t\tSuspendisse potenti. Maecs aliquet urna dolor, non commodo aue\n"
    "\tconsequat sed. Vestibulum mlis mollis sem, id convallis leo ulic "
    "in. Sed vel odio placerat urna dictum rutrum.");
  while (runApp && ros::ok()) {
    textLocs.clear();
    textStrings.clear();
    textHeights.clear();
    textColors.clear();
    static const int kNumTextParagraphs = 20;
    for (int i = 0; i < kNumTextParagraphs; ++i) {
      textLocs.push_back(Vector2f(10.0, 4.0 * static_cast<float>(i)));
      for (size_t j = 0; j < str.length(); ++j) {
        if (str[j] >= 'a' && str[j] <= 'z') {
          str[j] = 'a' + ((str[j] - 'a' + char_offset) % 26);
        }
      }
      textStrings.push_back(str);
      textHeights.push_back(1.0);
      textColors.push_back(VectorDisplay::Color(0xFF000000));
      char_offset = (char_offset + 1) % 26;
    }
    textLocs.push_back(Vector2f(0.0, 10.0));
    textStrings.push_back(StringPrintf("%.3f", GetWallTime()));
    textHeights.push_back(1.0);
    textColors.push_back(VectorDisplay::Color(0xFF0000F0));
    display->updateText(textLocs,
                        textStrings,
                        textHeights,
                        textColors,
                        text_in_window_coords);
    if (!kTextOnly) {
      static const float scale = 0.005;
      static const int numLines = 10;
      static const int numPoints = 800;
      vector<VectorDisplay::Line> lines;
      vector<Vector2f> points;
      double dTheta = 2.0*M_PI/static_cast<double>(numLines);
      Vector2f offset(1000.0, 0.0);
      double theta = 0.0;
      double omega = RadToDeg(30.0);
      const float angle = AngleMod<double>(omega*GetWallTime());
      for (int i = 0; i < numLines; i++) {
        const Vector2f p(1000.0, 0.0);
        const Vector2f p1 =
            scale * (Rotation2Df(theta + angle) * p);
        const Vector2f p2 =
            scale * (Rotation2Df(theta + dTheta + angle) * p);
        lines.push_back(VectorDisplay::Line(p1, p2));
        theta += dTheta;
      }
      dTheta = 2.0*M_PI/static_cast<double>(numPoints);
      theta = 0.0;
      for (int i = 0; i < numPoints; i++) {
        Vector2f p(3500.0, 0.0);
        p = p*max(0.0, 1.1+sin(sin(2.0*theta)*M_PI))/2.0;
        p = Rotation2Df(theta + angle) * p + offset;
        p *= scale;
        points.push_back(p);
        theta += dTheta;
      }
      display->updateLines(lines, vector<VectorDisplay::Color>());
      display->updatePoints(points, vector<VectorDisplay::Color>());
    }
    Sleep(0.016);
  }
  if (debug) {
    printf("Terminating testMode thread. runApp:%d ok():%d\n",
        runApp?1:0, ros::ok()?1:0);
  }
}

void VectorDisplayThread::run() {
  if (FLAGS_test) {
    TestMode();
  } else {
    map_name_ = FLAGS_map;
    vectorMap.Load(MapnameToLocalizationFilename(map_name_));
    if (FLAGS_edit_navigation || FLAGS_view_navmap) {
      std::string nav_map_file = MapnameToNavigationFilenameJson(map_name_);
      std::string old_nav_map_file = MapnameToNavigationFilenameTxt(map_name_);
      if (!FileExists(nav_map_file) && FileExists(old_nav_map_file)) {
        printf("Could not find navigation file at %s. An V1 nav-map was found at %s. Please run map_upgrade to upgrade this map.\n", nav_map_file.c_str(), old_nav_map_file.c_str());
        exit(1);
      } else if (!FileExists(nav_map_file)) {
        printf("Could not find navigation file at %s.\n", nav_map_file.c_str());
        exit(1);
      } else {
        navMap.Load(nav_map_file);
      }
    }

    ros::Subscriber laserSub;
    ros::Subscriber kinectScanSub;
    ros::Subscriber localizationSub;
    ros::Subscriber visualizationSub;

    laserSub = node_handle_->subscribe(
        FLAGS_laser_topic, 1, &VectorDisplayThread::laserCallback, this);
    localizationSub = node_handle_->subscribe(
        FLAGS_localization_topic, 1,
        &VectorDisplayThread::LocalizationCallback, this);
    kinectScanSub = node_handle_->subscribe(
        "Cobot/Kinect/Scan", 1,
        &VectorDisplayThread::kinectScanCallback, this);
    visualizationSub = node_handle_->subscribe(
        "visualization", 1,
        &VectorDisplayThread::visualizationCallback, this);
    compileDisplay();

    while (runApp && ros::ok()) {
      ros::spinOnce();
      Sleep(0.01);
      if (clearDisplay) {
        clearDisplayMessages();
        clearDisplay = false;
        compileDisplay();
      }
    }
    printf("Terminating vector display thread. runApp:%d ok():%d\n",
        runApp?1:0, ros::ok()?1:0);
  }
  if (app != 0) app->quit();
}

VectorDisplayThread::VectorDisplayThread(
  VectorDisplay* disp,
    ros::NodeHandle* node_handle, QApplication* qapp, QObject* parent) :
    node_handle_(node_handle), app(qapp), display(disp) {
  runApp = true;
  clearDisplay = false;
  tPointCloud = 0.0;
  persistentDisplay = false;
  tLaser = 0.0;
  ros_helpers::InitRosHeader("map", &set_pose_msg.header);
  ros_helpers::InitRosHeader("map", &nav_target_msg_.header);
  localizationInitMsg.header.seq = 0;
  initialPosePublisher = node_handle_->advertise<amrl_msgs::Localization2DMsg>(
      "initialpose", 5, false);
  nav_goal_pub_ = node_handle_->advertise<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 10);
}

VectorDisplayThread::~VectorDisplayThread() {
  const string localization_map_file =
      MapnameToLocalizationFilename(map_name_);
  const string navigation_map_file =
      MapnameToNavigationFilenameJson(map_name_);
  if (FLAGS_edit_localization && vectorMap.lines.size() > 0) {
    if (vectorMap.Save(localization_map_file)) {
      printf("Saved map %s\n", map_name_.c_str());
    } else {
      printf("Error saving map %s\n", map_name_.c_str());
    }
  }
  if (FLAGS_edit_navigation && navMap.states.size() > 0) {
    if (navMap.Save(navigation_map_file)) {
      printf("Saved navigation map %s\n", map_name_.c_str());
    } else {
      printf("Error saving navigation map %s\n", map_name_.c_str());
    }
  }
  if (FLAGS_edit_semantic) {
    printf("TODO: Save semantic map\n");
  }
  fflush(stdout);
}

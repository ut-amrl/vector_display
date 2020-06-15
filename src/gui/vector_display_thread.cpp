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
#include "vector_display/LidarDisplayMsg.h"
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
using vector_display::LidarDisplayMsg;
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
DEFINE_string(map, "GDC3", "Initial localization map to load");
DEFINE_double(max_fps, 60.0, "Maximum graphics refresh rate");

string MapnameToLocalizationFilename(const string& map) {
  return StringPrintf("%s/%s/%s.vectormap.txt",
                      FLAGS_maps_dir.c_str(),
                      map.c_str(),
                      map.c_str());
}

string MapnameToNavigationFilename(const string& map) {
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
    bool* has_door) {
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
        MapnameToNavigationFilename(map_name.toStdString());
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
  compileDisplay();
}

void VectorDisplayThread::editMap(
    const Vector2f& mouse_down, const Vector2f& mouse_up, float orientation,
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
      static const float kMaxError = 1.0;
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
    float orientation, uint32_t modifiers) {
  const Vector2f p0 = mouse_down;
  const Vector2f p1 = mouse_up;
  static const float kMaxError = 0.1;

  // Check if an edge was selected.
  uint64_t edge_p0(0), edge_p1(0);
  const float nearest_edge_dist =
      navMap.GetClosestEdge(p0, &edge_p0, &edge_p1);
  const bool down_on_edge =
      edge_p0 < navMap.states.size() &&
      edge_p1 < navMap.states.size() &&
      nearest_edge_dist < kMaxError;

  // Check if the mouse down location was near a vertex.
  const uint64_t nearest_vertex_down = navMap.GetClosestVertex(p0);
  const float down_vertex_dist =
      (navMap.states[nearest_vertex_down].loc - p0).norm();
  const bool down_on_vertex =
      nearest_vertex_down < navMap.states.size() &&
      down_vertex_dist < kMaxError;

  // Check if the mouse up location was near a vertex.
  const uint64_t nearest_vertex_up = navMap.GetClosestVertex(p1);
  const float up_vertex_dist =
      (navMap.states[nearest_vertex_up].loc - p0).norm();
  const bool up_on_vertex =
      nearest_vertex_up < navMap.states.size() &&
      up_vertex_dist < kMaxError;

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
            // navMap.AddVertex(navMap.GetNextVertexIndex(), p0.x, p0.y, angle,
            //                  roomType, roomLabel);
            printf("TODO: Semantic map editing\n");
          }
        }
      } else if (dragged_between_vertices) {
        // add edge if drag from one vertex to another
        // using default values for width, max_speed, and has_door
        if (FLAGS_edit_navigation) {
          float width = 1;
          float max_speed = 1;
          bool has_door = false;
          if (GetNavEdgeParams(&width, &max_speed, &has_door)) {
            printf("TODO: Save edge params width:%f speed:%f door:%d\n",
                  width,
                  max_speed,
                  has_door);
            navMap.AddUndirectedEdge(nearest_vertex_down, nearest_vertex_up);
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
        navMap.DeleteUndirectedEdge(edge_p0, edge_p1);
      } else {
        return;
      }
      compileDisplay();
    } break;

    case 0x01: { // Alt
      // Move edge or vertex
      if (down_on_vertex &&
          (!down_on_edge || down_vertex_dist < nearest_edge_dist)) {
        navMap.states[nearest_vertex_down].loc = p1;
      } else if (down_on_edge) {
        Vector2f shift = p1 - p0;
        navMap.states[edge_p0].loc += shift;
        navMap.states[edge_p1].loc += shift;
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
          if (GetNavEdgeParams(&width, &max_speed, &has_door)) {
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
    uint32_t modifiers) {
  static const bool debug = false;
  if (FLAGS_edit_localization) {
    editMap(mouse_down, mouse_up, orientation, modifiers);
    if (modifiers == 0x01) {
      printf("Length: %f\n", (mouse_down - mouse_up).norm());
    }
    return;
  }
  if (FLAGS_edit_navigation || FLAGS_edit_semantic) {
    editGraph(mouse_down, mouse_up, orientation, modifiers);
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
      MapnameToNavigationFilename(msg.map);
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

void VectorDisplayThread::displayMsgCallback(
    const ros::MessageEvent<LidarDisplayMsg const>& msgEvent) {
  static const bool debug = false;
  const vector_display::LidarDisplayMsgConstPtr &msg = msgEvent.getConstMessage();
  bool duplicate = false;
  unsigned int i = 0;
  if (debug) {
    printf("Received message from %s\n",
           msgEvent.getPublisherName().c_str());
  }
  for (; i < displayProviders.size() && !duplicate; i++) {
    if (displayProviders[i].compare(msgEvent.getPublisherName())== 0)
      duplicate = true;
  }
  if (debug) printf("Duplicate:%d, i:%d\n", duplicate, i);
  if (duplicate) {
    i--;
    displayMsgs[i] = *msg;
  } else {
    displayMsgs.push_back(*msg);
    displayProviders.push_back(msgEvent.getPublisherName());
  }
  compileDisplay();
}

void VectorDisplayThread::clearDisplayMessages() {
  displayMsgs.clear();
  displayProviders.clear();
  laserScanMsg.ranges.clear();
  kinectScanMsg.ranges.clear();
}

void VectorDisplayThread::DrawNavigationMap() {
  printf("TODO: Draw navigation map\n");

  // VectorDisplay::Color roomLabel(0.0, 0.0, 0.0, 1.0);
  // const vector<Vertex>& vertices = navMap.Vertices;
  // for(unsigned int i=0; i<navMap.numVertices; ++i) {
  //   Vector2f v = navMap.Vertices[i].loc;
  //   points.push_back(Vector2f(V2COMP(v)));
  //   pointColors.push_back(VectorDisplay::Color(0xFF008800));
  //   if (FLAGS_edit_semantic) {
  //     textStrings.push_back(vertices[i].name);
  //     textLocs.push_back(Vector2f(V2COMP(v)));
  //     textColors.push_back(roomLabel);
  //     textHeights.push_back(0.5);
  //     const Vector2f p0(V2COMP(vertices[i].loc));
  //     Vector2f heading;
  //     heading.heading(vertices[i].theta);
  //     const Vector2f p1(V2COMP(vertices[i].loc + 0.5*heading));
  //     lines.push_back(VectorDisplay::Line(p0, p1));
  //     lineColors.push_back(VectorDisplay::Color(0x7F404040));
  //   }
  // }
  // for(unsigned int i=0; i<navMap.numEdges; i++) {
  //   int v1 = navMap.Edges[i].v1;
  //   int v2 = navMap.Edges[i].v2;

  //   lines.push_back(VectorDisplay::Line(Vector2f(V2COMP(vertices[v1].loc)),
  //                                       Vector2f(V2COMP(vertices[v2].loc))));
  //   lineColors.push_back(VectorDisplay::Color(0xFFFF00FF));
  // }
}

void VectorDisplayThread::compileDisplay() {
  static double tLast = 0.0;
  static const double MessageTimeout = 1.0;
  static const VectorDisplay::Color LidarPointColor(0xFFF0761F);
  static const VectorDisplay::Color KinectScanColor(0xFFFF0505);
  static const VectorDisplay::Color PointCloudColor(0xFFDE2352);
  static const bool debug = true;

  if (debug) printf("GUI updated!\n");
  if (GetMonotonicTime()-tLast< 1.0 / FLAGS_max_fps) return;
  tLast = GetMonotonicTime();

  lines.clear();
  points.clear();
  circles.clear();
  quads.clear();
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

  for (unsigned int j = 0; j < displayMsgs.size(); j++) {
    const LidarDisplayMsg& displayMsg = displayMsgs[j];
    unsigned int numLines =
        min(min(displayMsg.lines_p1x.size(),
            displayMsg.lines_p1y.size()),
            min(displayMsg.lines_p2x.size(),
                displayMsg.lines_p2y.size()));
    for (unsigned int i = 0; i < numLines; i++) {
      lines.push_back(VectorDisplay::Line(
          Vector2f(displayMsg.lines_p1x[i], displayMsg.lines_p1y[i]),
          Vector2f(displayMsg.lines_p2x[i], displayMsg.lines_p2y[i])));
      if (i < displayMsg.lines_col.size()) {
        lineColors.push_back(VectorDisplay::Color(displayMsg.lines_col[i]));
      }
    }
    unsigned int numPoints = min(displayMsg.points_x.size(),
        displayMsg.points_y.size());
    for (unsigned int i = 0; i < numPoints; i++) {
      points.push_back(
          Vector2f(displayMsg.points_x[i], displayMsg.points_y[i]));
      if (i < displayMsg.points_col.size())
        pointColors.push_back(
            VectorDisplay::Color(displayMsg.points_col[i]));
    }
    unsigned int numCircles = min(displayMsg.circles_x.size(),
        displayMsg.circles_y.size());
    for (unsigned int i = 0; i < numCircles; i++) {
      circles.push_back(
          Vector2f(displayMsg.circles_x[i], displayMsg.circles_y[i]));
      if (i < displayMsg.circles_col.size())
        circleColors.push_back(
            VectorDisplay::Color(displayMsg.circles_col[i]));
    }
    for (size_t i = 0; i < displayMsg.text.size(); ++i) {
      textLocs.push_back(Vector2f(displayMsg.text_x[i], displayMsg.text_y[i]));
    }
    for (size_t i = 0; i < displayMsg.text_col.size(); ++i) {
      textColors.push_back(VectorDisplay::Color(displayMsg.text_col[i]));
    }
    textStrings.insert(textStrings.end(), displayMsg.text.begin(),
                       displayMsg.text.end());
    textHeights.insert(textHeights.end(), displayMsg.text_height.begin(),
                       displayMsg.text_height.end());
    textInWindowCoords.insert(textInWindowCoords.end(),
                              displayMsg.text_in_window_coords.begin(),
                              displayMsg.text_in_window_coords.end());
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
    const Vector2f laserLoc =
        Rotation2Df(robotAngle) * Vector2f(0.145, 0.0) + robotLoc;
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
      lineColors, pointColors, circleColors, quadColors, textLocs, textStrings,
      textHeights, textColors, textInWindowCoords);
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

    ros::Subscriber guiSub;
    ros::Subscriber laserSub;
    ros::Subscriber kinectScanSub;
    ros::Subscriber localizationSub;

    laserSub = node_handle_->subscribe(
        "Cobot/Laser", 1, &VectorDisplayThread::laserCallback, this);
    localizationSub = node_handle_->subscribe(
        "Cobot/Localization", 1,
        &VectorDisplayThread::LocalizationCallback, this);
    guiSub = node_handle_->subscribe(
        "Cobot/VectorLocalization/Gui", 1,
        &VectorDisplayThread::displayMsgCallback, this);
    kinectScanSub = node_handle_->subscribe(
        "Cobot/Kinect/Scan", 1,
        &VectorDisplayThread::kinectScanCallback, this);
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
  FLAGS_autoswitch_map = true;
  runApp = true;
  FLAGS_edit_localization = false;
  FLAGS_edit_navigation = false;
  FLAGS_edit_semantic = false;
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
      MapnameToNavigationFilename(map_name_);
  if (FLAGS_edit_localization) {
    if (vectorMap.Save(localization_map_file)) {
      printf("Saved map %s\n", map_name_.c_str());
    } else {
      printf("Error saving map %s\n", map_name_.c_str());
    }
  }
  if (FLAGS_edit_navigation) {
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

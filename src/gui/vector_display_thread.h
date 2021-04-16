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
\file    vector_display_thread.h
\brief   Thread to run the GUI for Vector Localization; C++ Implementation: VectorDisplayThread
\author  Joydeep Biswas, (C) 2010
 */
//========================================================================

#ifndef VECTOR_DISPLAY_THREAD_H_
#define VECTOR_DISPLAY_THREAD_H_

#include <stdio.h>

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtCore/QObject>
#include <QtCore/QThread>
#include <string>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"

#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "gui/vector_display.h"
#include "vector_map/vector_map.h"
#include "navigation_map/navigation_map.h"

class ScopedFile;

class VectorDisplayThread : public QThread {
  // Q_OBJECT

private:

  ros::NodeHandle *node_handle_;

  bool runApp;
  bool persistentDisplay;
  bool clearDisplay;
  bool autoUpdateMap;

  geometry_msgs::PoseWithCovarianceStamped localizationInitMsg;
  ros::Publisher initialPosePublisher;
  ros::Publisher nav_goal_pub_;

  amrl_msgs::Localization2DMsg set_pose_msg;
  geometry_msgs::PoseStamped nav_target_msg_;

  QApplication* app;
  VectorDisplay* display;
  std::string map_name_;

  navigation::GraphDomain navMap;

  vector_map::VectorMap vectorMap;
  Eigen::Vector2f robotLoc;
  float robotAngle;
  sensor_msgs::LaserScan laserScanMsg;
  sensor_msgs::LaserScan kinectScanMsg;
  sensor_msgs::PointCloud pointCloudMsg;
  std::vector<amrl_msgs::VisualizationMsg> visualizationMsgs;
  std::vector<std::string> displayProviders;
  double tLaser, tPointCloud;

  std::vector<VectorDisplay::Line> lines;
  std::vector<Eigen::Vector2f> points;
  std::vector<Eigen::Vector2f> circles;
  std::vector<VectorDisplay::Quad> quads;
  std::vector<VectorDisplay::ColoredArc> arcs;
  std::vector<VectorDisplay::Color> circleColors;
  std::vector<VectorDisplay::Color> lineColors;
  std::vector<VectorDisplay::Color> pointColors;
  std::vector<VectorDisplay::Color> quadColors;

  std::vector<std::string> textStrings;
  std::vector<Eigen::Vector2f> textLocs;
  std::vector<float> textHeights;
  std::vector<VectorDisplay::Color> textColors;
  std::vector<bool> textInWindowCoords;

public:

  void Zoom(float zoom);

  std::string GetMapName() { return map_name_; }

  bool GetNavEdgeParams(
      float* width,
      float* max_speed,
      bool* has_door,
      bool* has_stairs, 
      bool* has_elevator, 
      bool* has_automated_door);

  bool GetSemanticType(
      const std::vector<std::string>& types,
      std::string* selected_type);

  bool GetSemanticTypeAndLabel(
      const std::vector<std::string>& types,
      std::string* selected_type,
      std::string* label);

  void ChangeMap();

  void AutoLocalize();

  void KeyboardEventCallback(uint32_t key_code, uint32_t modifiers);

  void MouseEventCallback(
      const Eigen::Vector2f& mouse_down,
      const Eigen::Vector2f& mouse_up, float orientation,
      float viewScale,
      uint32_t modifiers);

  void drawMap(std::vector<VectorDisplay::Line>* lines,
               std::vector<VectorDisplay::Color>* lineColors);

  void LocalizationCallback(const amrl_msgs::Localization2DMsg& msg);

  void kinectScanCallback(const sensor_msgs::LaserScan& msg);

  void laserCallback(const sensor_msgs::LaserScan& msg);

  void visualizationCallback(
      const ros::MessageEvent<const amrl_msgs::VisualizationMsg>& msgEvent);

  void filteredPointCloudCallback(const sensor_msgs::PointCloud& msg);

  void clearDisplayMessages();

  void compileDisplay();

  void DrawNavigationMap();

  void TestMode();

  void setRunApp(bool newRunApp) {
    runApp = newRunApp;
  }

protected:
  void run();
  // Edit the localization map
  void editMap(const Eigen::Vector2f& mouse_down,
               const Eigen::Vector2f& mouse_up, float orientation,
               float viewScale,
               uint32_t modifiers);
  // Edit the navigation map or semantic map (both are really graphs)
  void editGraph(const Eigen::Vector2f& mouse_down,
                 const Eigen::Vector2f& mouse_up, float orientation,
                 float viewScale,
                 uint32_t modifiers);

public:
  VectorDisplayThread(VectorDisplay* disp,
      ros::NodeHandle* node_handle, QApplication* qapp = 0,
      QObject* parent = 0);
  ~VectorDisplayThread();
};

#endif /* VECTOR_DISPLAY_THREAD_H_ */

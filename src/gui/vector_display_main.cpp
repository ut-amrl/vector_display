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
\file    vector_display_main.cpp
\brief   A graphical vector localization vizualizer
\author  Joydeep Biswas, (C) 2010
 */
//========================================================================

#include <stdio.h>

#include <eigen3/Eigen/Dense>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>

#include "gflags/gflags.h"
#include "glog/logging.h"

#include "vector_display/GuiKeyboardEvent.h"
#include "vector_display/GuiMouseClickEvent.h"
#include "vector_display/GuiMouseMoveEvent.h"
#include "vector_display/LocalizationGuiCaptureSrv.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "shared/util/helpers.h"
#include "shared/util/timer.h"
#include "gui/vector_display.h"
#include "gui/vector_display_thread.h"
#include "vector_map/vector_map.h"
#include "config_reader/config_reader.h"

using Eigen::Vector2f;
using ros::Publisher;
using ros::Subscriber;
using std::max;
using std::size_t;
using std::string;
using std::vector;

ros::Publisher mouse_click_publisher_;
ros::Publisher mouse_move_publisher_;
ros::Publisher keyboard_events_publisher_;
ros::ServiceServer capture_service_;

vector_display::GuiMouseMoveEvent mouse_move_msg_;
vector_display::GuiMouseClickEvent mouse_click_msg_;
vector_display::GuiKeyboardEvent keyboard_events_msg_;

VectorDisplayThread* thread_ = NULL;
VectorDisplay* display_ = NULL;

void KeyboardEventCallback(uint32_t key_code, uint32_t modifiers) {
  if (thread_ != NULL)
    thread_->KeyboardEventCallback(key_code, modifiers);
  keyboard_events_msg_.header.seq++;
  keyboard_events_msg_.header.stamp = ros::Time::now();
  keyboard_events_msg_.keycode = key_code;
  keyboard_events_msg_.modifiers = modifiers;
  keyboard_events_publisher_.publish(keyboard_events_msg_);
}

void MouseMoveCallback(
    const Vector2f& location, uint32_t buttons, uint32_t modifiers) {
  mouse_move_msg_.location.x = location.x();
  mouse_move_msg_.location.y = location.y();
  mouse_move_msg_.buttons = buttons;
  mouse_move_msg_.modifiers = modifiers;
  mouse_move_publisher_.publish(mouse_move_msg_);
}

void MouseClickCallback(
    const Vector2f& mouse_down, const Vector2f& mouse_up, float orientation, float viewScale,
    uint32_t modifiers) {
  if (thread_ != NULL) {
    thread_->MouseEventCallback(mouse_down, mouse_up, orientation, viewScale, modifiers);
  }
  mouse_click_msg_.header.seq++;
  mouse_click_msg_.header.stamp = ros::Time::now();
  mouse_click_msg_.modifiers = modifiers;
  mouse_click_msg_.mouse_down.x = mouse_down.x();
  mouse_click_msg_.mouse_down.y = mouse_down.y();
  mouse_click_msg_.mouse_up.x = mouse_up.x();
  mouse_click_msg_.mouse_up.y = mouse_up.y();
  mouse_click_publisher_.publish(mouse_click_msg_);
}

bool CaptureCallback(vector_display::LocalizationGuiCaptureSrv::Request& req,
                     vector_display::LocalizationGuiCaptureSrv::Response& res) {
  printf("Saving image to %s\n", req.filename.c_str());
  if (display_ != NULL) {
    display_->Capture(req.filename);
  }
  return true;
}

void InitRosMessages() {
  mouse_click_msg_.header.frame_id = "map";
  mouse_click_msg_.header.seq = 0;
  mouse_click_msg_.header.stamp.fromSec(0.0);
  mouse_click_msg_.mouse_down.z = 0.0;
  mouse_click_msg_.mouse_up.z = 0.0;

  mouse_move_msg_.header.frame_id = "map";
  mouse_move_msg_.header.seq = 0;
  mouse_move_msg_.header.stamp.fromSec(0.0);
  mouse_move_msg_.location.z = 0.0;

  keyboard_events_msg_.header.frame_id = "map";
  keyboard_events_msg_.header.seq = 0;
  keyboard_events_msg_.header.stamp.fromSec(0.0);
}

DEFINE_double(display_width, 100, "Initial display width in meters");

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  QApplication* app = NULL;
  static const bool debug = false;
  printf("Vector Localization GUI\n");

  InitRosMessages();
  const string node_name = StringPrintf(
      "vector_display_%lu",
      static_cast<uint64_t>(GetWallTime() * 1000.0));
  ros::init(argc, argv, node_name);

  ros::NodeHandle node_handle;
  mouse_move_publisher_ =
      node_handle.advertise<vector_display::GuiMouseMoveEvent>(
      "Cobot/VectorLocalization/GuiMouseMoveEvents", 1, false);
  mouse_click_publisher_ =
      node_handle.advertise<vector_display::GuiMouseClickEvent>(
      "Cobot/VectorLocalization/GuiMouseClickEvents", 1, false);
  keyboard_events_publisher_ =
      node_handle.advertise<vector_display::GuiKeyboardEvent>(
      "Cobot/VectorLocalization/GuiKeyboardEvents", 1, false);
  capture_service_ = node_handle.advertiseService(
      "VectorLocalization/Capture", CaptureCallback);

  app = new QApplication(argc, argv);
  display_ = new VectorDisplay();

  //InitHandleStop(&runApp);
  display_->show();
  thread_ = new VectorDisplayThread(display_, &node_handle, app);

  display_->setMouseClickCallback(&MouseClickCallback);
  display_->setKeyboardCallback(&KeyboardEventCallback);
  display_->setMouseMoveCallback(&MouseMoveCallback);

  thread_->start();
  display_->setView(0, 0, FLAGS_display_width);

  int retVal = app->exec();
  thread_->setRunApp(false);
  if (debug) printf("Waiting for thread termination... ");
  thread_->wait();
  if (debug) printf("Done, exiting.\n");
  delete thread_;
  delete display_;
  return retVal;
}

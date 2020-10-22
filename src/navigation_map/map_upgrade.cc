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
\file    map_upgrade.cc
\brief   Utility to convert V1 Nav maps to V2 format.
\author  Joydeep Biswas, (C) 2020
*/
//========================================================================

#include <stdio.h>

#include "gflags/gflags.h"
#include "glog/logging.h"

#include "navigation_map/navigation_map.h"

using navigation::GraphDomain;

DEFINE_string(in, "", "Input V1 navigation map file");
DEFINE_string(out, "", "Output V2 navigation map file");

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  CHECK(!FLAGS_in.empty());
  CHECK(!FLAGS_out.empty());

  GraphDomain map;
  map.SaveV2FromV1(FLAGS_in, FLAGS_out);

  printf("Verifying...\n");
  map.Load(FLAGS_out);
  return 0;
}
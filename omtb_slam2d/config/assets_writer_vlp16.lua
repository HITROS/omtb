-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

VOXEL_SIZE = 1

options = {
  tracking_frame = "robot1/imu_link",
  published_frame = "robot1/odom",
  odom_frame = "robot1/odom",
  pipeline = {
    {
      action = "min_max_range_filter",
      min_range = 1.,
      max_range = 60.,
    },
    {
      action = "dump_num_points",
    },
    {
      action = "write_ply",
      filename = "points.ply"
    },
    {
      action = "write_pcd",
      filename = "points.pcd"
    },
  }
}

return options

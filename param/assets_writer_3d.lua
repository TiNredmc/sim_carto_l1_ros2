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

VOXEL_SIZE = 5e-2

include "transform.lua"

options = {
  tracking_frame = "unilidar_frame",
  pipeline = {
    {
      action = "min_max_range_filter",
      min_range = 0.05,
      max_range = 30.,
    },
    {
      action = "dump_num_points",
    },

    {
      action = "fixed_ratio_sampler",
      sampling_ratio = 0.90, 
    },

    {
      action = "write_pcd",
      filename = "l1_cloud.pcd"
    },
  }
}

return options

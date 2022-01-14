// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the gpu_voxels_ros package.
// © Copyright 2022, Mark Finean 
// 
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the followingdisclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation 
// and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software 
// without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS 
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// -- END LICENSE BLOCK ------------------------------------------------
//----------------------------------------------------------------------
/*!\file
 *
 * \author  Mark Finean
 * \date    2020-09-24 
 */
//----------------------------------------------------------------------

#ifndef GPU_VOXELS_ROS_RECOVERYPLANNER_H
#define GPU_VOXELS_ROS_RECOVERYPLANNER_H

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <chrono>

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct MapDims
{
  MapDims() : x(), y(), z() {} // problematic in device shared memory arrays

  explicit MapDims(uint32_t _x) : x(_x), y(_x), z(_x) {}

  MapDims(uint32_t _x, uint32_t _y, uint32_t _z) : x(_x), y(_y), z(_z) {}
  
  uint32_t x;
  uint32_t y;
  uint32_t z;

};

class RecoveryPlanner
{
    private:
        MapDims map_dims;
        float cell_size;
        std::vector<bool> occ_grid_2d_;
    public:
        RecoveryPlanner() {};
        RecoveryPlanner(MapDims map_dims, float cell_size, std::vector<bool> occ_grid_2d): map_dims(map_dims), cell_size(cell_size), occ_grid_2d_(occ_grid_2d) {};
        RecoveryPlanner(MapDims map_dims, float cell_size): map_dims(map_dims), cell_size(cell_size) {};
        virtual ~RecoveryPlanner(){};

    void plan(const float start_x, const float start_y, const float goal_x, const float goal_y, const uint interp_num);
    void updateOccupancyGrid(std::vector<bool> occ_grid_2d);

};



class ValidityChecker : public ob::StateValidityChecker
{
    private:
    std::vector<bool> occ_grid_2d;
    MapDims map_dims;
    float cell_size;

    public:
    ValidityChecker(const ob::SpaceInformationPtr& si, std::vector<bool> &occ_grid_2d, MapDims map_dims, float cell_size) :
        ob::StateValidityChecker(si), occ_grid_2d(occ_grid_2d), map_dims(map_dims), cell_size(cell_size) {}

    bool isValid(const ob::State* state) const;
    uint32_t getVoxelIndexUnsigned(const double x, const double y) const;

};

  


#endif
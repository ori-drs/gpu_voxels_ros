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
#include <gpu_voxels_ros/RecoveryPlanner.h>

void RecoveryPlanner::plan(const float start_x,const float start_y, const float goal_x, const float goal_y, const uint interp_num)
{
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));

    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(map_dims.x);

    // space->setBounds(bounds);
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    og::SimpleSetup ss(space);

    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    ss.setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si, occ_grid_2d_, map_dims, cell_size)));
    ss.getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
    
    ob::ScopedState<ob::RealVectorStateSpace> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_x;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_y;

    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_x;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_y;

    ss.setStartAndGoalStates(start, goal);

    ss.setup();
    ss.print();

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
        
        og::PathGeometric &p = ss.getSolutionPath();
        p.interpolate(interp_num);
        // p.printAsMatrix(std::cout);

        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        p.print(std::cout);
    }
    else{
        std::cout << "No solution found" << std::endl;
    }
}

void RecoveryPlanner::updateOccupancyGrid(std::vector<bool> occ_grid_2d){
  occ_grid_2d_ = occ_grid_2d;
}


bool ValidityChecker::isValid(const ob::State* state) const
{
    const ob::RealVectorStateSpace::StateType* state2D = state->as<ob::RealVectorStateSpace::StateType>();

    // Extract the robot's (x,y) position from its state
    double x = state2D->values[0];
    double y = state2D->values[1];

    uint32_t lin_ind = getVoxelIndexUnsigned(x, y);

    return !occ_grid_2d[lin_ind];
}

//! Maps 3D voxel coordinates to linear voxel index
uint32_t ValidityChecker::getVoxelIndexUnsigned(const double x, const double y) const
{
    return floor(y) * map_dims.x + floor(x);
}

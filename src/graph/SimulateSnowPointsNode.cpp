// Copyright 2023 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <graph/NodesCore.hpp>
#include <gpu/snowflakesKernels.hpp>
#include <gpu/helpersKernels.hpp>


void SimulateSnowPointsNode::setParameters(float snowfallRate, float snowflakeTerminalVelicity, int numberOfChannels, float maxRange, float beamDivergence)
{
    this->snowfallRate = snowfallRate;
    this->terminalVelocity = snowflakeTerminalVelicity;
    this->numberOfChannels = numberOfChannels;
    this->maxRange = maxRange;
    this->beamDivergence = beamDivergence;

    // TODO: calculate number of flakes per channel.
    this ->numberOfFlakesPerChannel = 100;

    for(int i = 0; i < numberOfChannels; i++)
    {
        snowflakesDisks.emplace_back(DeviceAsyncArray<Vec2f>::create(arrayMgr));

        snowflakesDisks[i]->resize(numberOfFlakesPerChannel, false, false);
        randomizationStates->resize(numberOfFlakesPerChannel, false, false);
        gpuSetupRandomNumberGenerator(arrayMgr.getStream()->getHandle(), numberOfFlakesPerChannel, randomDevice(), randomizationStates->getWritePtr());

        auto* randPtr = randomizationStates->getWritePtr();
        auto currentDiskPtr = snowflakesDisks[i]->getWritePtr();

        gpuSnowflakesSimulationCalculateDisk(arrayMgr.getStream()->getHandle(), numberOfFlakesPerChannel, maxRange, currentDiskPtr, randPtr);

    }
    CHECK_CUDA(cudaStreamSynchronize(arrayMgr.getStream()->getHandle()));

}
void SimulateSnowPointsNode::validateImpl()
{
    IPointsNodeSingleInput::validateImpl();

    for (int i = 0; i < numberOfChannels; ++i) {
       if(snowflakesDisks[i]->getCount() != numberOfFlakesPerChannel)
           throw std::runtime_error("Snowflakes disks are not calculated!");

    }
}
void SimulateSnowPointsNode::enqueueExecImpl()
{
    // TODO raytrace cloud again with snowflakes
    auto pointCount = input->getPointCount();
  //  outXyz->resize(pointCount, false, false);
}
IAnyArray::ConstPtr SimulateSnowPointsNode::getFieldData(rgl_field_t field)
{
    if (field == XYZ_F32) {
        return outXyz;
    }
    return input->getFieldData(field);
}

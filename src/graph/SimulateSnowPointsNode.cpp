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


void SimulateSnowPointsNode::setParameters(float maxRange,
    float rainRate, float meanSnowflakeDiameter, float terminalVelocity,
    float density, int32_t numberOfChannels, float beamDivergence)
{
    this->numberOfChannels = numberOfChannels;
    this->maxRange = maxRange;
    this->beamDivergence = beamDivergence;

    // Calculate derived snow parameters
    // Only God and Mr Hahner knows WTF is going on there. DEBUG it with someone slowly.
    // Duck it for now.
    float helper = rainRate / terminalVelocity;
    float snowfallHelper = rainRate / (487.0f * density * meanSnowflakeDiameter * terminalVelocity);
    float snowfallRate = sqrt(snowfallHelper * snowfallHelper * snowfallHelper);
    float distributionRate = 2.55f * pow(snowfallRate, 0.48f);

    float areaOfDisk =  3.14 * maxRange * maxRange;
    float snowedArea = (rainRate / (density * terminalVelocity)) * areaOfDisk;
    // Clamp snowed area.
    if(snowedArea>areaOfDisk)
    {
        snowedArea = areaOfDisk;
    }

    float meanAreaOfFlake = 3.14 * meanSnowflakeDiameter * meanSnowflakeDiameter;
    this ->numberOfFlakesPerChannel = snowedArea / meanAreaOfFlake;

    for(int i = 0; i < numberOfChannels; i++)
    {
        snowflakesDisks.emplace_back(DeviceAsyncArray<Vec3f>::create(arrayMgr));

        snowflakesDisks[i]->resize(numberOfFlakesPerChannel, false, false);
        randomizationStates->resize(numberOfFlakesPerChannel, false, false);
        gpuSetupRandomNumberGenerator(arrayMgr.getStream()->getHandle(), numberOfFlakesPerChannel, randomDevice(), randomizationStates->getWritePtr());

        auto* randPtr = randomizationStates->getWritePtr();
        auto currentDiskPtr = snowflakesDisks[i]->getWritePtr();

        gpuSnowflakesSimulationCalculateDisk(arrayMgr.getStream()->getHandle(), numberOfFlakesPerChannel, maxRange, distributionRate, currentDiskPtr, randPtr);

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

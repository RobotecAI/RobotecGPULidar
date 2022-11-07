// Copyright 2022 Robotec.AI
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

#include <graph/Nodes.hpp>

#include <pcl/io/pcd_io.h>


void WritePCDFilePointsNode::validate()
{
	input = getValidInput<IPointsNode>();
}

void WritePCDFilePointsNode::schedule(cudaStream_t stream)
{
	if (input->getWidth() == 0) {
		return;
	}

	// Get formatted input data
	FormatPointsNode::formatAsync(inputFmtData, input, requiredFields, stream);

	// Convert to PCL cloud
	const PCLPointType * data = reinterpret_cast<const PCLPointType*>(inputFmtData->getReadPtr(MemLoc::Host));
	pcl::PointCloud<PCLPointType> cloud;
	cloud.resize(input->getWidth(), input->getHeight());
	cloud.assign(data, data + cloud.size(), input->getWidth());
	cloud.is_dense = input->isDense();
	cachedPCLs += cloud;
}

WritePCDFilePointsNode::~WritePCDFilePointsNode()
{
	if (cachedPCLs.empty()) {
		RGL_WARN("{}: skipped saving PCD file {} - empty point cloud", getName(), filePath.string());
		return;
	}
	pcl::io::savePCDFileASCII(filePath.string(), cachedPCLs);
}

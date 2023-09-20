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

#pragma once

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/time.h>
#include <vtkVersion.h>

class PCLVisualizerFix : public pcl::visualization::PCLVisualizer
{
public:
	using Ptr = std::shared_ptr<PCLVisualizerFix>;
	using ConstPtr = std::shared_ptr<const PCLVisualizerFix>;

	PCLVisualizerFix() : pcl::visualization::PCLVisualizer() { }

	// Based on https://github.com/PointCloudLibrary/pcl/issues/5556
	void close()
	{
#if __unix__
		// VTX fixed the segmentation fault on close() in version 9.2.3, but we cannot simply check the version because
		// VTK_BUILD_VERSION is modified in some way (vcpkg?). Manual verification needed.
		if (VTK_MAJOR_VERSION != 9 && VTK_MINOR_VERSION != 2 && VTK_BUILD_VERSION != 20220823) {
			RGL_WARN("VTK version has changed. Existence of PCLVisualizerFix may not be necessary.");
		}
		interactor_->SetDone(true);
#else
		pcl::visualization::PCLVisualizer::close();
#endif
	}

	void closeFinalViewer()
	{
#if __unix__
		pcl::visualization::PCLVisualizer::close();
#endif
	}

	virtual ~PCLVisualizerFix() { }
};

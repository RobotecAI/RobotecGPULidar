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

class PCLVisualizerFix : public pcl::visualization::PCLVisualizer {
public:
	using Ptr = std::shared_ptr<PCLVisualizerFix>;
	using ConstPtr = std::shared_ptr<const PCLVisualizerFix>;

	PCLVisualizerFix() : pcl::visualization::PCLVisualizer() {}

	// Based on https://github.com/PointCloudLibrary/pcl/pull/5252
	void spinOnce(int time, bool force_redraw = true)
	{
		if (!interactor_->IsA("vtkXRenderWindowInteractor")) {
			pcl::visualization::PCLVisualizer::spinOnce(time, force_redraw);
			return;
		}

		resetStoppedFlag();

		if (!interactor_) {
			return;
		}

		if (time <= 0) {
			time = 1;
		}

		if (force_redraw) {
			interactor_->Render();
		}

		DO_EVERY(1.0 / interactor_->GetDesiredUpdateRate(), interactor_->ProcessEvents();
		         std::this_thread::sleep_for(std::chrono::milliseconds(time)););
	}

	virtual ~PCLVisualizerFix() {}
};

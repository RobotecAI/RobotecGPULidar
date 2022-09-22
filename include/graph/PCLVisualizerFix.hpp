#pragma once

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/time.h>

class PCLVisualizerFix : public pcl::visualization::PCLVisualizer
{
public:
	using Ptr = std::shared_ptr<PCLVisualizerFix>;
	using ConstPtr = std::shared_ptr<const PCLVisualizerFix>;

	PCLVisualizerFix() : pcl::visualization::PCLVisualizer() { }

	// Based on https://github.com/PointCloudLibrary/pcl/pull/5252
	void spinOnce (int time, bool force_redraw = true)
	{
		if(!interactor_->IsA("vtkXRenderWindowInteractor")) {
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

		DO_EVERY(1.0 / interactor_->GetDesiredUpdateRate(),
			interactor_->ProcessEvents();
			std::this_thread::sleep_for(std::chrono::milliseconds(time));
		);
	}

	virtual ~PCLVisualizerFix() { }
};

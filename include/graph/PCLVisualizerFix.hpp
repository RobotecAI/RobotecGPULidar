#pragma once

#include <pcl/visualization/pcl_visualizer.h>

class PCLVisualizerFix : public pcl::visualization::PCLVisualizer
{
public:
	using Ptr = std::shared_ptr<PCLVisualizerFix>;
	using ConstPtr = std::shared_ptr<const PCLVisualizerFix>;

	static const int FRAME_RATE = 60;

	PCLVisualizerFix() :
		pcl::visualization::PCLVisualizer()
	{
		fixedTimerId = interactor_->CreateRepeatingTimer(1000 / FRAME_RATE);
	};

	virtual ~PCLVisualizerFix()
	{
		if (interactor_) {
			interactor_->DestroyTimer(fixedTimerId);
		}
	}

private:
	int fixedTimerId;
};

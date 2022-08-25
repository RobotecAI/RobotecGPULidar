#include <pipeline/Nodes.hpp>
#include <gpu/nodeKernels.hpp>

#include <pcl/filters/voxel_grid.h>

void DownSampleNode::validate()
{

}

void DownSampleNode::schedule(cudaStream_t stream)
{
	// a) Add passthrough to FormatNode
	// b) Make FormatNodes internal to DownSampleNode and WritePCDFileNode

	// Internal, vs API-added??
}

size_t DownSampleNode::getWidth() const
{
	return 0;
}

VArray::ConstPtr DownSampleNode::getFieldData(rgl_field_t field, cudaStream_t stream) const
{
	return VArray::ConstPtr();
}

#include <rgl/api/experimental.h>
#include <graph/Interfaces.hpp>
#include <gpu/GPUFieldDesc.hpp>
#include <RGLFields.hpp>
#include <VArrayProxy.hpp>

std::size_t getPointSize(const std::vector<rgl_field_t>& fields);
VArrayProxy<GPUFieldDesc>::Ptr getGPUFields(const std::vector<rgl_field_t>& fields,
                                            IPointCloudNode::Ptr input, cudaStream_t stream);

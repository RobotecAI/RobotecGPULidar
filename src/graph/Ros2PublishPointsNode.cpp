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

#include <graph/NodesRos2.hpp>

RglRos2Node::RglRos2Node(std::string topicName)
: Node(std::string("rgl_") + topicName)
{
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topicName, 10);
}

void RglRos2Node::constructRos2Msg(std::vector<rgl_field_t> fields)
{
    msg_ = sensor_msgs::msg::PointCloud2();
    int offset = 0;
    for (const auto& field : fields) {
        switch (field) {
            case XYZ_F32:
                msg_.fields.push_back(createPointFieldMsg("x", offset, sensor_msgs::msg::PointField::FLOAT32, 1));
                msg_.fields.push_back(createPointFieldMsg("y", offset + 4, sensor_msgs::msg::PointField::FLOAT32, 1));
                msg_.fields.push_back(createPointFieldMsg("z", offset + 8, sensor_msgs::msg::PointField::FLOAT32, 1));
                offset += 12;
                break;
            case IS_HIT_I32:
                msg_.fields.push_back(createPointFieldMsg("is_hit", offset, sensor_msgs::msg::PointField::INT32, 1));
                offset += 4;
                break;
            case RAY_IDX_U32:
                msg_.fields.push_back(createPointFieldMsg("ray_idx", offset, sensor_msgs::msg::PointField::UINT32, 1));
                offset += 4;
                break;
            case INTENSITY_F32:
                msg_.fields.push_back(createPointFieldMsg("intensity", offset, sensor_msgs::msg::PointField::FLOAT32, 1));
                offset += 4;
                break;
            case RING_ID_U16:
                msg_.fields.push_back(createPointFieldMsg("ring", offset, sensor_msgs::msg::PointField::UINT16, 1));
                offset += 2;
                break;
            case AZIMUTH_F32:
                msg_.fields.push_back(createPointFieldMsg("azimuth", offset, sensor_msgs::msg::PointField::FLOAT32, 1));
                offset += 4;
                break;
            case DISTANCE_F32:
                msg_.fields.push_back(createPointFieldMsg("distance", offset, sensor_msgs::msg::PointField::FLOAT32, 1));
                offset += 4;
                break;
            case RETURN_TYPE_U8:
                msg_.fields.push_back(createPointFieldMsg("return_type", offset, sensor_msgs::msg::PointField::UINT8, 1));
                offset += 1;
                break;
            case TIME_STAMP_F64:
                msg_.fields.push_back(createPointFieldMsg("time_stamp", offset, sensor_msgs::msg::PointField::FLOAT64, 1));
                offset += 8;
                break;
            case PADDING_8:
                offset += 1;
                break;
            case PADDING_16:
                offset += 2;
                break;
            case PADDING_32:
                offset += 4;
                break;
            default:
                throw std::invalid_argument(fmt::format("unknown RGL field {} for ROS2 msg", field));
        }
    }
    msg_.height = 1,
    msg_.point_step = offset,
    msg_.is_dense = true,
    msg_.is_bigendian = false,
    msg_.header.frame_id = "test";
}

sensor_msgs::msg::PointField RglRos2Node::createPointFieldMsg(std::string name, int offset, int datatype, int count)
{
    auto pointField = sensor_msgs::msg::PointField();
    pointField.name = name;
    pointField.offset = offset;
    pointField.datatype = datatype;
    pointField.count = count;
    return pointField;
}

void RglRos2Node::publish(const void* rawData, int count)
{
    unsigned char *charBuf = (unsigned char*)rawData;
    std::vector<unsigned char> v(charBuf, charBuf + msg_.point_step * count);
    msg_.data = v;
    msg_.width = count;
    msg_.row_step = msg_.point_step * msg_.width;
    publisher_->publish(msg_);
}

void Ros2PublishPointsNode::setParameters(const char* topicName, const char* frameId)
{
    if (ros2Node.get() != nullptr) {
		RGL_WARN("Could not update parameters for Ros2PublishPointsNode.");
		return;
	}

	this->topicName = topicName;
	this->frameId = frameId;

    rclcpp::init(0, nullptr);
    ros2Node = std::make_shared<RglRos2Node>(this->topicName);
}

void Ros2PublishPointsNode::validate()
{
	input = getValidInput<FormatPointsNode>();
    ros2Node->constructRos2Msg(input->getRequiredFieldList());
}

void Ros2PublishPointsNode::schedule(cudaStream_t stream)
{
    //input->getFieldData(RGL_FIELD_DYNAMIC_FORMAT, stream);
    auto fielddata = input->getFieldData(RGL_FIELD_DYNAMIC_FORMAT, stream);
	ros2Node->publish(fielddata->getReadPtr(MemLoc::Host), input->getPointCount());
}

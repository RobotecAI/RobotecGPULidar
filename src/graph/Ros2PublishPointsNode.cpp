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

void Ros2PublishPointsNode::setParameters(
	const char* topicName, const char* frameId,
	rgl_qos_policy_reliability_t qosReliability,
	rgl_qos_policy_durability_t qosDurability,
	rgl_qos_policy_history_t qosHistory, int32_t qosDepth)
{
	if (ros2Node.get() == nullptr) {
		static char *args[] = {
			(char*)"--ros-args",
			(char*)"--disable-external-lib-logs",
			nullptr
		};
		rclcpp::init(2, args);

		ros2Node = std::make_shared<rclcpp::Node>(ros2NodeName);
		ros2Executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
		ros2Executor->add_node(ros2Node);
	}

	if (ros2TopicNames.contains(topicName) && this->topicName != topicName) {
		throw InvalidAPIArgument("ROS2 publisher with the same topic name already exist!");
	}

	if (ros2Publisher.get() != nullptr) {
		ros2TopicNames.erase(this->topicName);
		ros2Publisher.reset();
	}

	this->topicName = topicName;
	this->frameId = frameId;
	ros2TopicNames.insert(topicName);

	rclcpp::QoS qos = rclcpp::QoS(qosDepth);
	qos.reliability(static_cast<rmw_qos_reliability_policy_t>(qosReliability));
	qos.durability(static_cast<rmw_qos_durability_policy_t>(qosDurability));
	qos.history(static_cast<rmw_qos_history_policy_t>(qosHistory));

	ros2Publisher = ros2Node->create_publisher<sensor_msgs::msg::PointCloud2>(topicName, qos);
}

void Ros2PublishPointsNode::validate()
{
	input = getValidInput<FormatPointsNode>();
	constructRos2Message(input->getRequiredFieldList(), input->isDense());
}

void Ros2PublishPointsNode::schedule(cudaStream_t stream)
{
	auto fielddata = input->getFieldData(RGL_FIELD_DYNAMIC_FORMAT, stream);
	int count = input->getPointCount();
	unsigned char *charBuf = (unsigned char*)fielddata->getReadPtr(MemLoc::Host);
	std::vector<unsigned char> v(charBuf, charBuf + ros2Message.point_step * count);
	ros2Message.data = v;
	ros2Message.width = count;
	ros2Message.row_step = ros2Message.point_step * ros2Message.width;
	ros2Publisher->publish(ros2Message);
	ros2Executor->spin_some();
}

Ros2PublishPointsNode::~Ros2PublishPointsNode()
{
	ros2TopicNames.erase(topicName);
	ros2Publisher.reset();

	if (ros2TopicNames.empty()) {
		rclcpp::shutdown();
		ros2Node.reset();
		ros2Executor.reset();
	}
}

void Ros2PublishPointsNode::constructRos2Message(std::vector<rgl_field_t> fields, bool isDense)
{
	ros2Message = sensor_msgs::msg::PointCloud2();
	int offset = 0;
	for (const auto& field : fields) {
		switch (field) {
			case XYZ_F32:
				ros2Message.fields.push_back(createPointFieldMsg("x", offset, sensor_msgs::msg::PointField::FLOAT32, 1));
				ros2Message.fields.push_back(createPointFieldMsg("y", offset + 4, sensor_msgs::msg::PointField::FLOAT32, 1));
				ros2Message.fields.push_back(createPointFieldMsg("z", offset + 8, sensor_msgs::msg::PointField::FLOAT32, 1));
				offset += 12;
				break;
			case IS_HIT_I32:
				ros2Message.fields.push_back(createPointFieldMsg("is_hit", offset, sensor_msgs::msg::PointField::INT32, 1));
				offset += 4;
				break;
			case RAY_IDX_U32:
				ros2Message.fields.push_back(createPointFieldMsg("ray_idx", offset, sensor_msgs::msg::PointField::UINT32, 1));
				offset += 4;
				break;
			case INTENSITY_F32:
				ros2Message.fields.push_back(createPointFieldMsg("intensity", offset, sensor_msgs::msg::PointField::FLOAT32, 1));
				offset += 4;
				break;
			case RING_ID_U16:
				ros2Message.fields.push_back(createPointFieldMsg("ring", offset, sensor_msgs::msg::PointField::UINT16, 1));
				offset += 2;
				break;
			case AZIMUTH_F32:
				ros2Message.fields.push_back(createPointFieldMsg("azimuth", offset, sensor_msgs::msg::PointField::FLOAT32, 1));
				offset += 4;
				break;
			case DISTANCE_F32:
				ros2Message.fields.push_back(createPointFieldMsg("distance", offset, sensor_msgs::msg::PointField::FLOAT32, 1));
				offset += 4;
				break;
			case RETURN_TYPE_U8:
				ros2Message.fields.push_back(createPointFieldMsg("return_type", offset, sensor_msgs::msg::PointField::UINT8, 1));
				offset += 1;
				break;
			case TIME_STAMP_F64:
				ros2Message.fields.push_back(createPointFieldMsg("time_stamp", offset, sensor_msgs::msg::PointField::FLOAT64, 1));
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
	ros2Message.height = 1,
	ros2Message.point_step = offset,
	ros2Message.is_dense = isDense,
	ros2Message.is_bigendian = false,
	ros2Message.header.frame_id = frameId;
}

sensor_msgs::msg::PointField Ros2PublishPointsNode::createPointFieldMsg(std::string name, int offset, int datatype, int count)
{
	auto pointField = sensor_msgs::msg::PointField();
	pointField.name = name;
	pointField.offset = offset;
	pointField.datatype = datatype;
	pointField.count = count;
	return pointField;
}

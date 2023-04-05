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

#include "UdpClient.hpp"
#include <cerrno>

UdpClient UdpClient::create(const std::string& destAddr, int destPort, bool enableBroadcast)
{
	UdpClient udpClient(destAddr, destPort, enableBroadcast);
	if (!udpClient.initialized) {
		auto msg = fmt::format("Failed to create UDP client to '{}:{}'", destAddr, destPort);
		throw UdpError(msg);
	}
	return udpClient;
}

UdpClient::UdpClient(const std::string& destAddr, int destPort, bool enableBroadcast)
{
	#ifdef _WIN32
	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		return;
	}
	#endif

	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd == INVALID_SOCKET) {
		return;
	}

	if (enableBroadcast) {
		char broadcastEnable = 1;
		setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
	}

	memset(&sockAddr, 0, sizeof(sockAddr));
	sockAddr.sin_family = AF_INET;
	sockAddr.sin_port = htons(destPort);
	if (inet_pton(AF_INET, destAddr.c_str(), &sockAddr.sin_addr) <= 0) {
		return;
	}

	initialized = true;
}

UdpClient::~UdpClient()
{
	#ifdef _WIN32
	if (sockfd != INVALID_SOCKET) {
		closesocket(sockfd);
	}
	WSACleanup();
	#else
	if (sockfd != INVALID_SOCKET) {
		close(sockfd);
	}
	#endif // _WIN32
}

void UdpClient::send(const std::vector<char>& buffer)
{
	if (sendto(sockfd, buffer.data(), buffer.size(), 0, (const sockaddr*)& sockAddr, sizeof(sockAddr)) < 0) {
		auto msg = fmt::format("Failed to send UDP packet: {}", strerror(errno));
		throw UdpError(msg);
	}
}

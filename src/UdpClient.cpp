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

#define BROADCAST_ADDRESS "255.255.255.255"

#ifdef _WIN32
	#define LAST_ERROR std::system_category().message(WSAGetLastError())
	#define SO_BROADCAST_TYPE char
#else
	#define LAST_ERROR strerror(errno)
	#define SO_BROADCAST_TYPE int
#endif // _WIN32

UdpClient UdpClient::createBroadcaster(const std::string& srcAddr, int srcPort)
{
	return create(srcAddr, srcPort, BROADCAST_ADDRESS, srcPort);
}

UdpClient UdpClient::create(const std::string& srcAddr, int srcPort,
                            const std::string& dstAddr, int dstPort)
{
	UdpClient udpClient(srcAddr, srcPort, dstAddr, dstPort);
	if (!udpClient.initialized) {
		auto msg = fmt::format("Failed to create UDP client '{}:{}->{}:{}': {}",
		                       srcAddr, srcPort, dstAddr, dstPort, udpClient.initErrorMsg);
		throw UdpError(msg);
	}
	return udpClient;
}

UdpClient::UdpClient(const std::string& srcAddr, int srcPort,
                     const std::string& dstAddr, int dstPort) :
	initialized(false)
{
	#ifdef _WIN32
	// Initialize Winsock
	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		initErrorMsg = fmt::format("cannot init Winsock - {}", LAST_ERROR);
		return;
	}
	#endif

	// Create socket
	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd == INVALID_SOCKET) {
		initErrorMsg = fmt::format("cannot create socket: {}", LAST_ERROR);
		return;
	}

	// Create socket address
	if (!addressTextToBinary(srcAddr, srcPort, sockAddr)) {
		return;  // initErrorMsg already set
	}

	// Bind constructed address to socket
	if (bind(sockfd, (const struct sockaddr *)&sockAddr, sizeof(sockAddr)) < 0)
	{
		initErrorMsg = fmt::format("cannot bind socket - {}", LAST_ERROR);
		return;
	}

	// Set broadcast permission to socket if destination is BROADCAST_ADDRESS
	if (dstAddr == BROADCAST_ADDRESS) {
		SO_BROADCAST_TYPE broadcastEnable = 1;
		if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) == -1) {
			initErrorMsg = fmt::format("cannot enable broadcast - {}", LAST_ERROR);
			return;
		}
	}

	// Create destination address
	if (!addressTextToBinary(dstAddr, dstPort, sendToAddr)) {
		return;  // initErrorMsg already set
	}

	initialized = true;
}

bool UdpClient::addressTextToBinary(const std::string& textAddr, int port, sockaddr_in& outAddr)
{
	memset(&outAddr, 0, sizeof(outAddr));
	outAddr.sin_family = AF_INET;
	outAddr.sin_port = htons(port);
	int ptonRet = inet_pton(AF_INET, textAddr.c_str(), &outAddr.sin_addr);
	if (ptonRet == 0) {
		initErrorMsg = fmt::format("invalid address format '{}:{}'", textAddr, port);
		return false;
	}
	if (ptonRet < 0) {
		initErrorMsg = fmt::format("invalid address '{}:{}' - {}", textAddr, port, LAST_ERROR);
		return false;
	}
	return true;
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
	if (sendto(sockfd, buffer.data(), buffer.size(), 0, (const sockaddr*)& sendToAddr, sizeof(sendToAddr)) < 0) {
		auto msg = fmt::format("Failed to send UDP packet: {}", LAST_ERROR);
		throw UdpError(msg);
	}
}

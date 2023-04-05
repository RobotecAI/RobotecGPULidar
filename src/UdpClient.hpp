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

#pragma once

#include <string>
#include <vector>

#ifdef _WIN32
	#include <WinSock2.h>
	#include <Ws2tcpip.h>
	#pragma comment(lib, "ws2_32.lib")
#else
	#include <sys/socket.h>
	#include <arpa/inet.h>
	#include <netinet/in.h>
#endif // _WIN32

#include "Logger.hpp"
#include "RGLExceptions.hpp"

#ifdef _WIN32
	#define SOCKET_TYPE SOCKET
#else
	#define SOCKET_TYPE int
	#define INVALID_SOCKET (-1)
#endif // _WIN32

struct UdpClient
{
	// Ctor cannot throw, because we want Dtor call if instance creation failed
	static UdpClient create(const std::string& destAddr, int destPort, bool enableBroadcast=false);

	~UdpClient();

	void send(const std::vector<char>& buffer);

private:
	UdpClient(const std::string& destAddr, int destPort, bool enableBroadcast=false);

	bool initialized = false;
	sockaddr_in sockAddr;
	SOCKET_TYPE sockfd = INVALID_SOCKET;
};

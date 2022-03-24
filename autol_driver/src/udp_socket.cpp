#include "autol_driver/udp_socket.h"

namespace autol_driver
{
    UDPSocket::UDPSocket()
    {
        ROS_INFO("constructor : UDPsocket"); 
    }

    UDPSocket::~UDPSocket()
    {
    }

    int UDPSocket::CreateSocket()
	{
		udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        
		if (udp_socket_ != 0)
			return 0;
		else
			return -1;
	}

    int UDPSocket::Bind(unsigned short port)
	{
		sockaddr_in add;
		//memset(&add, 0, sizeof(add));
		add.sin_family = AF_INET;
		add.sin_addr.s_addr = htonl(INADDR_ANY);
		add.sin_port = htons(port);

		return bind(udp_socket_, (sockaddr *)&add, sizeof(sockaddr));
	}

	int UDPSocket::SetSocketBuffer(int size)
	{
		int rc, rbuffer, wbuffer;
		rbuffer = wbuffer = size;
		rc = setsockopt(udp_socket_, SOL_SOCKET, SO_RCVBUF, (char*)&rbuffer, sizeof(rbuffer));
		
        return 1;
	}

    int UDPSocket::PrintSocketBuffer()
	{
		// int rcv_buf_size;
		// int sockopt_size = sizeof(rcv_buf_size);
		// getsockopt(udp_socket_, SOL_SOCKET, SO_RCVBUF, (char*)&rcv_buf_size, &sockopt_size);
		// //cout << "buf size : " << rcv_buf_size << endl;
		//return rcv_buf_size;
        return 0;
	}

    int UDPSocket::SetTimeout(timeval tv)
	{
		struct timeval optVal = tv;

		int optLen = sizeof(optVal);

//#ifdef _WIN32
//		DWORD dw = (secs * 1000) + ((u_secs + 999) / 1000);
//		setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (char*)&dw, sizeof(dw));
//#else
//		setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(struct timeval));
//#endif

		unsigned long dw = (tv.tv_sec * 1000) + ((tv.tv_usec + 999) / 1000);
		if (setsockopt(udp_socket_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&dw, optLen) < 0) {
			return -1;
			perror("Error");
		}

		return 0;
	}

    // int UDPSocket::RecvFrom(char* buffer, int len, sockaddr_in* from, int flags) // OOB = 0, Non-Blocking = 2
	// {

	// 	int size = sizeof(*from);
	// 	int ret = recvfrom(udp_socket_, buffer, len, flags, reinterpret_cast<SOCKADDR*>(from), &size);
	// 	/*if (ret == SOCKET_ERROR)
	// 		throw std::system_error(WSAGetLastError(), std::system_category(), "recvfrom failed");*/

	// 		// make the buffer zero terminated
	// 	buffer[ret] = 0;
	// 	return ret;
	// }

	int UDPSocket::RecvFrom(char* buffer, int len, int flags) // OOB = 0, Non-Blocking = 2
	{
		sockaddr_in from;
		socklen_t size = sizeof(from);

        ssize_t ret = recvfrom(udp_socket_, buffer, len, flags, (sockaddr*) &from, &size);

        if (ret < 0)
        {
            return -1;
        }
        return ret;
	}

	int UDPSocket::RecvFrom(autol_msgs::AutolPacket* packet, int len, int flags) // OOB = 0, Non-Blocking = 2
	{
		sockaddr_in from;
		socklen_t size = sizeof(from);

        ssize_t ret = recvfrom(udp_socket_, &packet->data[0], len, flags, (sockaddr*) &from, &size);

        if (ret < 0)
        {
            return -1;
        }
        return ret;
	}

    int UDPSocket::CloseSocket()
	{
		if(udp_socket_ != 0)
			return close(udp_socket_);
		return 0;
	}

}
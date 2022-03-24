#pragma once

#include <ros/ros.h>

#include <sys/socket.h>
#include <netinet/in.h>

#include <autol_msgs/AutolPacket.h>

#include "autol_driver/udp_packet.h"

namespace autol_driver
{
    class UDPSocket
    {
    public:
        UDPSocket();
        ~UDPSocket();

        int CreateSocket();
        // int SetMultiCast(const char* multicast_group);
        // int InitWinSockLib();
        // int CloseWinSockLib();
         int CloseSocket();
        // int SendTo(const std::string& address, unsigned short port, const char* buffer, int len, int flags = 0);
        // int SendTo(sockaddr_in& address, const char* buffer, int len, int flags = 0);
        //int RecvFrom(char* buffer, int len, sockaddr_in* from, int flags = 0); // OOB = 0, Non-Blocking = 2
        int RecvFrom(char* buffer, int len, int flags = 0); // OOB = 0, Non-Blocking = 2
        int RecvFrom(autol_msgs::AutolPacket* buffer, int len, int flags = 0);
        int Bind(unsigned short port);
        int SetTimeout(timeval tv);
        int SetSocketBuffer(int size);
        int PrintSocketBuffer();

        // void DeSerialize(ValeoUdpPacket* udp_packet, char* bytes);

        // void DeSerialize(VelodyneUdpPacket* udp_packet, char* bytes);

        // void DeSerialize(LeishenUdpPacket* udp_packet, char* bytes);

        int udp_socket_;
    };
}
#include "ssdp_discovery.h"
#include "psxemulator.h"
#include <regex>
#include <iostream>
#include <vector>

PCSX::SSDPClient::SSDPClient() {}

void PCSX::SSDPClient::start(uv_loop_t* loop) {
    uv_udp_init(loop, &m_recvSocket);
    uv_udp_set_multicast_loop(&m_recvSocket, 0); // Disable multicast loopback
    uv_ip4_addr("10.170.241.9", 0, &m_recvAddr);
    uv_udp_bind(&m_recvSocket, (const struct sockaddr*)&m_recvAddr, UV_UDP_REUSEADDR);
    uv_udp_recv_start(&m_recvSocket, allocCB, readCB);

    m_status = Status::STARTED;

}

void PCSX::SSDPClient::stop() {
    if (m_status == Status::STOPPED) return;
    uv_udp_recv_stop(&m_recvSocket);
    uv_close((uv_handle_t *)&m_recvSocket, closeCB);

    m_status == Status::STOPPED;
}

void PCSX::SSDPClient::closeCB(uv_handle_t* handle) {
    printf("closing\n");
//    if (handle != NULL) {
//        delete handle;
//    }
}

void PCSX::SSDPClient::search() {

    char msg[1024] = "\0";
    snprintf(msg, 1024, "M-SEARCH * HTTP/1.1\r\nHOST: %s:%d\r\nST: ssdp:all\r\nMAN: \"ssdp:discover\"\r\nMX: 2\r\n\r\n",
             "239.255.255.250", 1900);
    uv_buf_t send = uv_buf_init(msg, 1024);

    uv_ip4_addr("239.255.255.250", 1900, &m_sendAddr);
    uv_udp_send(&send_req, &m_recvSocket, &send, 1, (const struct sockaddr *)&m_sendAddr, sendCB);
}

void PCSX::SSDPClient::allocCB(uv_handle_t *handle, size_t suggested_size, uv_buf_t *buf) {
    buf->base = (char *)malloc(suggested_size);
    buf->len = 1024;
}

void PCSX::SSDPClient::readCB(uv_udp_t *req, ssize_t nread, const uv_buf_t *buf, const struct sockaddr *addr, unsigned int flags) {
    if (nread < 0 || addr == NULL) {
        uv_udp_recv_stop(req);
        free(buf->base);
        return;
    }

    char sender[16] = "";
    uv_ip4_name((const struct sockaddr_in *)addr, sender, 16);
    printf("%s\n", sender);

    std::string str(buf->base);
    std::smatch location, server;
    std::regex_search(str, location, std::regex("LOCATION:\\s(.*)"));
    std::regex_search(str, server, std::regex("SERVER:\\s(.*)"));
    std::cout << location[1] << std::endl;
    std::cout << server[1] << std::endl;
//    for (auto x: server) {
//        std::cout << x << std::endl;
//    }

    free(buf->base);
}

void PCSX::SSDPClient::sendCB(uv_udp_send_t *req, int status) {
    if (status == -1) {
        printf("Send Error\n");
        return;
    } else {
        printf("Sent\n");
        return;
    }

}

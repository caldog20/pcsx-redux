#pragma once
#include <string>
#include "core/debug.h"
#include "core/psxemulator.h"

namespace PCSX {
class SSDPClient;


class SSDPClient {
  public:
    void start(uv_loop_t* loop);
    void stop();
    void search();
    enum class Status {
        STOPPED,
        STARTED,
    };


  private:
    uv_loop_t* m_loop;
    uv_udp_t m_recvSocket;
    uv_udp_t m_sendSocket;
    struct sockaddr_in m_recvAddr;
    struct sockaddr_in m_sendAddr;
    uv_udp_send_t send_req;
    Status m_status = Status::STOPPED;
    static void allocCB(uv_handle_t *handle, size_t suggested_size, uv_buf_t *buf);
    static void readCB(uv_udp_t *req, ssize_t nread, const uv_buf_t *buf, const struct sockaddr *addr, unsigned int flags);
    static void sendCB(uv_udp_send_t *req, int status);
    static void closeCB(uv_handle_t* handle);
  public:
    SSDPClient();
    ~SSDPClient() = default;

};



} // namespace PCSX

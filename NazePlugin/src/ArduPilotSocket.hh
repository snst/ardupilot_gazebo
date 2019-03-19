#ifndef ARDUPILOTSOCKET_HH
#define ARDUPILOTSOCKET_HH

#include <fcntl.h>
#ifdef _WIN32
  #include <Winsock2.h>
  #include <Ws2def.h>
  #include <Ws2ipdef.h>
  #include <Ws2tcpip.h>
  using raw_type = char;
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <arpa/inet.h>
  #include <pthread.h>
  using raw_type = void;
#endif

class ArduPilotSocket
{
  /// \brief constructor
  public: ArduPilotSocket()
  {
    // initialize socket udp socket
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    #ifndef _WIN32
    // Windows does not support FD_CLOEXEC
    fcntl(fd, F_SETFD, FD_CLOEXEC);
    #endif
  }

  /// \brief destructor
  public: ~ArduPilotSocket()
  {
    if (fd != -1)
    {
      ::close(fd);
      fd = -1;
    }
  }

  /// \brief Bind to an adress and port
  /// \param[in] _address Address to bind to.
  /// \param[in] _port Port to bind to.
  /// \return True on success.
  public: bool Bind(const char *_address, const uint16_t _port)
  {
    struct sockaddr_in sockaddr;
    this->MakeSockAddr(_address, _port, sockaddr);

    if (bind(this->fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0)
    {
      shutdown(this->fd, 0);
      #ifdef _WIN32
      closesocket(this->fd);
      #else
      close(this->fd);
      #endif
      return false;
    }
    int one = 1;
    setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR,
        reinterpret_cast<const char *>(&one), sizeof(one));

    #ifdef _WIN32
    u_long on = 1;
    ioctlsocket(this->fd, FIONBIO,
              reinterpret_cast<u_long FAR *>(&on));
    #else
    fcntl(this->fd, F_SETFL,
        fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);
    #endif
    return true;
  }

  /// \brief Connect to an adress and port
  /// \param[in] _address Address to connect to.
  /// \param[in] _port Port to connect to.
  /// \return True on success.
  public : bool Connect(const char *_address, const uint16_t _port)
  {
    struct sockaddr_in sockaddr;
    this->MakeSockAddr(_address, _port, sockaddr);

    if (connect(this->fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0)
    {
      shutdown(this->fd, 0);
      #ifdef _WIN32
      closesocket(this->fd);
      #else
      close(this->fd);
      #endif
      return false;
    }
    int one = 1;
    setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR,
        reinterpret_cast<const char *>(&one), sizeof(one));

    #ifdef _WIN32
    u_long on = 1;
    ioctlsocket(this->fd, FIONBIO,
              reinterpret_cast<u_long FAR *>(&on));
    #else
    fcntl(this->fd, F_SETFL,
        fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);
    #endif
    return true;
  }

  /// \brief Make a socket
  /// \param[in] _address Socket address.
  /// \param[in] _port Socket port
  /// \param[out] _sockaddr New socket address structure.
  public: void MakeSockAddr(const char *_address, const uint16_t _port,
    struct sockaddr_in &_sockaddr)
  {
    memset(&_sockaddr, 0, sizeof(_sockaddr));

    #ifdef HAVE_SOCK_SIN_LEN
      _sockaddr.sin_len = sizeof(_sockaddr);
    #endif

    _sockaddr.sin_port = htons(_port);
    _sockaddr.sin_family = AF_INET;
    _sockaddr.sin_addr.s_addr = inet_addr(_address);
  }

  public: ssize_t Send(const void *_buf, size_t _size)
  {
    return send(this->fd, _buf, _size, 0);
  }

  /// \brief Receive data
  /// \param[out] _buf Buffer that receives the data.
  /// \param[in] _size Size of the buffer.
  /// \param[in] _timeoutMS Milliseconds to wait for data.
  public: ssize_t Recv(void *_buf, const size_t _size, uint32_t _timeoutMs)
  {
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(this->fd, &fds);

    tv.tv_sec = _timeoutMs / 1000;
    tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

    if (select(this->fd+1, &fds, NULL, NULL, &tv) != 1)
    {
        return -1;
    }

    #ifdef _WIN32
    return recv(this->fd, reinterpret_cast<char *>(_buf), _size, 0);
    #else
    return recv(this->fd, _buf, _size, 0);
    #endif
  }

  /// \brief Socket handle
  private: int fd;
};


#endif

#include <errno.h>

#include <netdb.h>
#include <netinet/in.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <fgfsclient.hpp>

//#include <iostream>

namespace {
   QUAN_QUANTITY_LITERAL(time,s)
}

template <unsigned int BufLen>
FGFSSocket<BufLen>::FGFSSocket(const char *hostname, unsigned port) :
	m_sock(-1),
	m_connected(false),
	m_timeout(1_s)
{
	m_sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (m_sock < 0)
		throw("FGFSSocket/socket");

	struct hostent *hostinfo;
	hostinfo = gethostbyname(hostname);
	if (!hostinfo) {
		close();
		throw("FGFSSocket/gethostbyname: unknown host");
	}

	struct sockaddr_in serv_addr;
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(port);
	serv_addr.sin_addr = *(struct in_addr *)hostinfo->h_addr;

	if (::connect(m_sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
		close();
		throw("FGFSSocket/connect");
	}
	m_connected = true;
	try {
		write("data");
	} catch (...) {
		close();
		throw;
	}
}


template <unsigned int BufLen>
FGFSSocket<BufLen>::~FGFSSocket()
{
   close();
}


template <unsigned int BufLen>
int FGFSSocket<BufLen>::close(void)
{
	if (m_connected){
		write("quit");
   }
	if (m_sock < 0){
		return 0;
   }
	int const ret = ::close(m_sock);
	m_sock = -1;
	return ret;
}

template <unsigned int BufLen>
int FGFSSocket<BufLen>::write(const char *msg, ...)
{
	va_list va;
	ssize_t len;
	char buf[BufLen];
	fd_set fd;
	struct timeval tv;

	FD_ZERO(&fd);
	FD_SET(m_sock, &fd);
	tv.tv_sec = static_cast<unsigned>(m_timeout.numeric_value());
	tv.tv_usec = 0;
	if (!select(FD_SETSIZE, 0, &fd, 0, &tv))
		throw("FGFSSocket::write/select: timeout exceeded");

	va_start(va, msg);
	vsnprintf(buf, BufLen - 2, msg, va);
	va_end(va);
	//std::cout << "SEND: " << buf << std::endl;
	strcat(buf, "\015\012");

	len = ::write(m_sock, buf, strlen(buf));
	if (len < 0)
		throw("FGFSSocket::write");
	return len;
}

template <unsigned int BufLen>
const char * FGFSSocket<BufLen>::read(void)
{
	char *p;
	fd_set fd;
	struct timeval tv;
	ssize_t len;

	FD_ZERO(&fd);
	FD_SET(m_sock, &fd);
	tv.tv_sec = static_cast<unsigned>(m_timeout.numeric_value());
	tv.tv_usec = 0;
	if (!select(FD_SETSIZE, &fd, 0, 0, &tv)) {
		if (m_timeout == 0_s)
			return 0;
		else
			throw("FGFSSocket::read/select: timeout exceeded");
	}

	len = ::read(m_sock, m_buffer, BufLen - 1);
	if (len < 0)
		throw("FGFSSocket::read/read");
	if (len == 0)
		return 0;

	for (p = &m_buffer[len - 1]; p >= m_buffer; p--)
		if (*p != '\015' && *p != '\012')
			break;
	*++p = '\0';
	return strlen(m_buffer) ? m_buffer : 0;
}

template <unsigned int BufLen>
inline void FGFSSocket<BufLen>::flush(void)
{
	auto const i = m_timeout;
	m_timeout = 0_s;
	while (read())
		;
	m_timeout = i;
}

template class FGFSSocket<256U>;

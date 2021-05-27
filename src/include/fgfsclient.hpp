#ifndef FG_EXTERNAL_TEST_FGFS_CLIENT_HPP_INCLUDED
#define FG_EXTERNAL_TEST_FGFS_CLIENT_HPP_INCLUDED

#include <quan/time.hpp>

template <unsigned int BufLen>
class FGFSSocket {
public:

   static constexpr char default_host[] = "localhost";
   static constexpr unsigned default_port = 5501;

	FGFSSocket(const char *name = default_host, unsigned port = default_port);
	~FGFSSocket();

	int		write(const char *msg, ...);
	const char	*read(void);
	void	flush(void);
	void		settimeout(quan::time_<int32_t>::s t) { m_timeout = t; }

private:
	int		close(void);

	int		m_sock;
	bool		m_connected;
	quan::time_<int32_t>::s	m_timeout;
	char		m_buffer[BufLen];
};

#endif // FG_EXTERNAL_TEST_FGFS_CLIENT_HPP_INCLUDED

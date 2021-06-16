#ifndef FG_EXTERNAL_TEST_FGFS_CLIENT_HPP_INCLUDED
#define FG_EXTERNAL_TEST_FGFS_CLIENT_HPP_INCLUDED

#include <quan/time.hpp>

/*
 Copyright (C) Andy Little 2021

 derived from https://sourceforge.net/p/flightgear/flightgear/ci/next/tree/scripts/example/fgfsclient.cxx

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see http://www.gnu.org/licenses./
*/

class fgfs_telnet {
public:

   static constexpr char default_host[] = "localhost";
   static constexpr unsigned default_port = 5501;
   static constexpr size_t default_buffer_size = 256;

	fgfs_telnet(const char *name = default_host, unsigned port = default_port, size_t buffer_size = default_buffer_size);
	~fgfs_telnet();
   fgfs_telnet(fgfs_telnet const &) = delete;
   fgfs_telnet& operator =(fgfs_telnet const &) = delete;

   bool is_readable(quan::time::s const & t) const;
   bool is_writeable(quan::time::s const & t) const;

   template <typename T>
   bool get(const char* prop, T& val);

   template <typename T>
   bool set(const char* prop, T const & val) const;

	bool write(const char *msg, ...)const;
/**
  @return pointer to internal buffer with result or null
*/
	const char* read();
	void flush();
	void settimeout(quan::time_<int32_t>::s t) { m_timeout = t; }
	int  close();
private:
	int		m_sock;
	char *	m_buffer;
   size_t   m_buflen;
	quan::time_<int32_t>::s	m_timeout;
	bool		m_connected;
};

#endif // FG_EXTERNAL_TEST_FGFS_CLIENT_HPP_INCLUDED

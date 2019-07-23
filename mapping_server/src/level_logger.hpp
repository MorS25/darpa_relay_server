/*
	restinio
*/

/*!
	Ready to use logger implementation for using with std::ostream.
*/

#pragma once

#include <string>
#include <iostream>
#include <chrono>
#include <mutex>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/time.h>

namespace restinio
{

enum log_level_t
{
  TRACE = 0,
  INFO = 1,
  WARN = 2,
  ERROR = 3
};

class level_logger_t
{
	public:
		level_logger_t( const level_logger_t & ) = delete;
		level_logger_t & operator = ( const level_logger_t & ) = delete;

		level_logger_t( log_level_t level ) noexcept
			:	m_level{ level }, m_out{ &std::cout }
		{}

		level_logger_t( log_level_t level, std::ostream & out ) noexcept
			:	m_level{ level }, m_out{ &out }
		{}

		template< typename Message_Builder >
		void
		trace( Message_Builder && msg_builder )
		{
      if (m_level <= TRACE)
        log_message( "TRACE", msg_builder() );
		}

		template< typename Message_Builder >
		void
		info( Message_Builder && msg_builder )
		{
      if (m_level <= INFO)
        log_message( " INFO", msg_builder() );
		}

		template< typename Message_Builder >
		void
		warn( Message_Builder && msg_builder )
		{
      if (m_level <= WARN)
        log_message( " WARN", msg_builder() );
		}

		template< typename Message_Builder >
		void
		error( Message_Builder && msg_builder )
		{
      if (m_level <= ERROR)
        log_message( "ERROR", msg_builder() );
		}

	private:
		void
		log_message( const char * tag, const std::string & msg )
		{
			using namespace std;
			using namespace chrono;

			auto now = system_clock::now();
			auto ms = duration_cast< milliseconds >( now.time_since_epoch() );
			time_t unix_time = duration_cast< seconds >( ms ).count();

			( *m_out )
				<< fmt::format(
						"[{:%Y-%m-%d %H:%M:%S}.{:03d}] {}: {}",
						make_localtime( unix_time ),
						static_cast< int >( ms.count() % 1000u ),
						tag,
						msg )
				<< std::endl;
		}

    log_level_t m_level;
		std::ostream * m_out;
};

} /* namespace restinio */

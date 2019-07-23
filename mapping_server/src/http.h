#pragma once

#include <exception>
#include <json/json.hpp>
#include <restinio/http_headers.hpp>

using restinio::request_handle_t;
using restinio::request_handling_status_t;
using restinio::http_status_line_t;
using nlohmann::json;

// ======================================================================
// HTTP Response construction routines
// ======================================================================

inline void
make_response(request_handle_t req,
    http_status_line_t status,
    const json& body)
{
  using nlohmann::json;
  req->create_response( status )
    .append_header( restinio::http_field::server, "RESTinio SubT Server")
    .append_header_date_field()
    .append_header( restinio::http_field::content_type, "application/json")
    .set_body(body.dump())
    .done();
}

// --------------------------------------------------
// HTTP OK response
// --------------------------------------------------

inline request_handling_status_t
response_ok(request_handle_t req, const json& body = json())
{
  make_response(std::move(req), restinio::status_ok(), body);
  return restinio::request_accepted();
}

// --------------------------------------------------
// HTTP error responses
// --------------------------------------------------

inline request_handling_status_t
response_error(request_handle_t req,
    http_status_line_t status,
    const json& body = json())
{
  make_response(std::move(req), std::move(status), body);
  return restinio::request_rejected();
}

// ======================================================================
// Error handling
// ======================================================================

// --------------------------------------------------
// HTTP errors as exceptions
// --------------------------------------------------
class HTTPError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;

  virtual restinio::http_status_line_t status() const = 0;
};

// --------------------------------------------------
// Exception types for each error code
// --------------------------------------------------

#define HTTP_ERROR_CLASS(Class, Status) \
  class Class : public HTTPError \
  { \
  public: \
    using HTTPError::HTTPError; \
    restinio::http_status_line_t status() const override { return Status(); } \
  };

HTTP_ERROR_CLASS(BadRequest,          restinio::status_bad_request);
HTTP_ERROR_CLASS(Unauthorized,        restinio::status_unauthorized);
HTTP_ERROR_CLASS(UnprocessableEntity, restinio::status_unprocessable_entity);
HTTP_ERROR_CLASS(NotFound,            restinio::status_not_found);
HTTP_ERROR_CLASS(TooManyRequests,     restinio::status_too_many_requests);

#undef HTTP_ERROR_CLASS


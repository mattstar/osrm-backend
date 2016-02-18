#ifndef SERVER_API_ROUTE_PARAMETERS_PARSER_HPP
#define SERVER_API_ROUTE_PARAMETERS_PARSER_HPP

#include "engine/api/route_parameters.hpp"
#include "engine/api/table_parameters.hpp"

#include <boost/optional/optional.hpp>

#include <type_traits>

namespace osrm
{
namespace server
{
namespace api
{

namespace detail
{
template <typename T> using is_parameter_t = std::is_base_of<engine::api::BaseParameters, T>;
} // ns detail

// Starts parsing and iter and modifies it until iter == end or parsing failed
template <typename ParameterT,
          typename std::enable_if<detail::is_parameter_t<ParameterT>::value, int>::type = 0>
boost::optional<ParameterT> parseParameters(std::string::iterator &iter, std::string::iterator end);

// Copy on purpose because we need mutability
template <typename ParameterT,
          typename std::enable_if<detail::is_parameter_t<ParameterT>::value, int>::type = 0>
boost::optional<ParameterT> parseParameters(std::string options_string)
{
    const auto first = options_string.begin();
    const auto last = options_string.end();
    return parseParameters<ParameterT>(first, last);
}

} // ns api
} // ns server
} // ns osrm

#endif

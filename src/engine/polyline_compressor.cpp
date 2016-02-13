#include "engine/polyline_compressor.hpp"

#include <boost/assert.hpp>
#include <cstddef>
#include <cstdlib>
#include <cmath>
#include <algorithm>

namespace osrm
{
namespace engine
{
namespace /*detail*/ // anonymous to keep TU local
{

std::string encode(int number_to_encode)
{
    std::string output;
    while (number_to_encode >= 0x20)
    {
        const int next_value = (0x20 | (number_to_encode & 0x1f)) + 63;
        output += static_cast<char>(next_value);
        number_to_encode >>= 5;
    }

    number_to_encode += 63;
    output += static_cast<char>(number_to_encode);
    return output;
}

std::string encode(std::vector<int> &numbers)
{
    std::string output;
    for (auto &number : numbers)
    {
        bool isNegative = number < 0;

        if (isNegative)
        {
            const unsigned binary = std::llabs(number);
            const unsigned twos = (~binary) + 1u;
            number = twos;
        }

        number <<= 1u;

        if (isNegative)
        {
            number = ~number;
        }
    }
    for (const int number : numbers)
    {
        output += encode(number);
    }
    return output;
}
} // anonymous ns


std::string encodePolyline(CoordVectorForwardIter begin, CoordVectorForwardIter end)
{
    auto size = std::distance(begin, end);
    if (size == 0)
    {
        return {};
    }

    std::vector<int> delta_numbers;
    BOOST_ASSERT(size > 0);
    delta_numbers.reserve((size - 1) * 2);
    util::FixedPointCoordinate previous_coordinate = {0, 0};
    std::for_each(begin, end, [&delta_numbers, &previous_coordinate](const FixedPointCoordinate loc)
    {
            const int lat_diff = (loc.lat - previous_coordinate.lat) * detail::COORDINATE_TO_POLYLINE;
            const int lon_diff = (loc.lon - previous_coordinate.lon) * detail::COORDINATE_TO_POLYLINE;
            delta_numbers.emplace_back(lat_diff);
            delta_numbers.emplace_back(lon_diff);
            previous_coordinate = loc;
    });
    return encode(delta_numbers);
}
std::vector<util::FixedPointCoordinate> decodePolyline(const std::string &geometry_string)
{
    std::vector<util::FixedPointCoordinate> new_coordinates;
    int index = 0, len = geometry_string.size();
    int lat = 0, lng = 0;

    while (index < len)
    {
        int b, shift = 0, result = 0;
        do
        {
            b = geometry_string.at(index++) - 63;
            result |= (b & 0x1f) << shift;
            shift += 5;
        } while (b >= 0x20);
        int dlat = ((result & 1) != 0 ? ~(result >> 1) : (result >> 1));
        lat += dlat;

        shift = 0;
        result = 0;
        do
        {
            b = geometry_string.at(index++) - 63;
            result |= (b & 0x1f) << shift;
            shift += 5;
        } while (b >= 0x20);
        int dlng = ((result & 1) != 0 ? ~(result >> 1) : (result >> 1));
        lng += dlng;

        util::FixedPointCoordinate p;
        p.lat = lat * detail::POLYLINE_TO_COORDINATE;
        p.lon = lng * detail::POLYLINE_TO_COORDINATE;
        new_coordinates.push_back(p);
    }

    return new_coordinates;
}
}
}

/*

Copyright (c) 2016, Project OSRM contributors
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef COORDINATE_HPP_
#define COORDINATE_HPP_

#include <iosfwd> //for std::ostream
#include <string>
#include <type_traits>

namespace osrm
{

constexpr const double COORDINATE_PRECISION = 1e6;

namespace util
{

struct FixedPointCoordinate
{
    int lat;
    int lon;

    FixedPointCoordinate();
    FixedPointCoordinate(int lat, int lon);

    template <class T>
    FixedPointCoordinate(const T &coordinate)
        : lat(coordinate.lat), lon(coordinate.lon)
    {
        static_assert(!std::is_same<T, FixedPointCoordinate>::value, "This constructor should not be used for FixedPointCoordinates");
        static_assert(std::is_same<decltype(lat), decltype(coordinate.lat)>::value,
                      "coordinate types incompatible");
        static_assert(std::is_same<decltype(lon), decltype(coordinate.lon)>::value,
                      "coordinate types incompatible");
    }

    bool IsValid() const;
    friend bool operator==(const FixedPointCoordinate lhs, const FixedPointCoordinate rhs);
    friend bool operator!=(const FixedPointCoordinate lhs, const FixedPointCoordinate rhs);
    friend std::ostream &operator<<(std::ostream &out, const FixedPointCoordinate coordinate);
};

bool operator==(const FixedPointCoordinate lhs, const FixedPointCoordinate rhs);
std::ostream &operator<<(std::ostream &out, const FixedPointCoordinate coordinate);
}

}

#endif /* COORDINATE_HPP_ */

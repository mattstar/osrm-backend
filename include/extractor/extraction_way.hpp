#ifndef EXTRACTION_WAY_HPP
#define EXTRACTION_WAY_HPP

#include "extractor/travel_mode.hpp"
#include "util/typedefs.hpp"
#include "engine/guidance/classification_data.hpp"

#include <string>
#include <vector>

namespace osrm
{
namespace extractor
{

/**
 * This struct is the direct result of the call to ```way_function```
 * in the lua based profile.
 *
 * It is split into multiple edge segments in the ExtractorCallback.
 */
struct ExtractionWay
{
    ExtractionWay() { clear(); }

    void clear()
    {
        forward_speed = -1;
        backward_speed = -1;
        duration = -1;
        roundabout = false;
        is_startpoint = true;
        is_access_restricted = false;
        name.clear();
        forward_travel_mode = TRAVEL_MODE_DEFAULT;
        backward_travel_mode = TRAVEL_MODE_DEFAULT;
        road_classification_data.invalidate();
    }

    enum Directions
    {
        notSure = 0,
        oneway,
        bidirectional,
        opposite
    };

    // These accessor methods exists to support the depreciated "way.direction" access
    // in LUA. Since the direction attribute was removed from ExtractionWay, the
    // accessors translate to/from the mode attributes.
    void set_direction(const Directions m)
    {
        if (Directions::oneway == m)
        {
            forward_travel_mode = TRAVEL_MODE_DEFAULT;
            backward_travel_mode = TRAVEL_MODE_INACCESSIBLE;
        }
        else if (Directions::opposite == m)
        {
            forward_travel_mode = TRAVEL_MODE_INACCESSIBLE;
            backward_travel_mode = TRAVEL_MODE_DEFAULT;
        }
        else if (Directions::bidirectional == m)
        {
            forward_travel_mode = TRAVEL_MODE_DEFAULT;
            backward_travel_mode = TRAVEL_MODE_DEFAULT;
        }
    }

    Directions get_direction() const
    {
        if (TRAVEL_MODE_INACCESSIBLE != forward_travel_mode &&
            TRAVEL_MODE_INACCESSIBLE != backward_travel_mode)
        {
            return Directions::bidirectional;
        }
        else if (TRAVEL_MODE_INACCESSIBLE != forward_travel_mode)
        {
            return Directions::oneway;
        }
        else if (TRAVEL_MODE_INACCESSIBLE != backward_travel_mode)
        {
            return Directions::opposite;
        }
        else
        {
            return Directions::notSure;
        }
    }

    // These accessors exists because it's not possible to take the address of a bitfield,
    // and LUA therefore cannot read/write the mode attributes directly.
    void set_forward_mode(const TravelMode m) { forward_travel_mode = m; }
    TravelMode get_forward_mode() const { return forward_travel_mode; }
    void set_backward_mode(const TravelMode m) { backward_travel_mode = m; }
    TravelMode get_backward_mode() const { return backward_travel_mode; }

    double forward_speed;
    double backward_speed;
    double duration;
    std::string name;
    bool roundabout;
    bool is_access_restricted;
    bool is_startpoint;
    TravelMode forward_travel_mode : 4;
    TravelMode backward_travel_mode : 4;
    engine::guidance::RoadClassificationData road_classification_data;
};
}
}

#endif // EXTRACTION_WAY_HPP

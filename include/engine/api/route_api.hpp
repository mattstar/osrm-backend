#ifndef ENGINE_API_ROUTE_HPP
#define ENGINE_API_ROUTE_HPP

#include "engine/api/base_api.hpp"
#include "engine/api/route_parameters.hpp"
#include "engine/api/json_factory.hpp"

#include "engine/datafacade/datafacade_base.hpp"

#include "engine/guidance/assemble_leg.hpp"
#include "engine/guidance/assemble_route.hpp"
#include "engine/guidance/assemble_geometry.hpp"
#include "engine/guidance/assemble_overview.hpp"
#include "engine/guidance/assemble_steps.hpp"

#include "engine/internal_route_result.hpp"

#include "util/integer_range.hpp"

namespace osrm
{
namespace engine
{
namespace api
{

class RouteAPI : public BaseAPI
{
  public:
    RouteAPI(const datafacade::BaseDataFacade &facade_, const RouteParameters &parameters_)
        : BaseAPI(facade_, parameters_), parameters(parameters_)
    {
    }

    void MakeResponse(const InternalRouteResult &raw_route, util::json::Object &response) const
    {
        auto number_of_routes = raw_route.has_alternative() ? 2UL : 1UL;
        util::json::Array routes;
        routes.values.resize(number_of_routes);
        routes.values[0] =
            MakeRoute(raw_route.segment_end_coordinates, raw_route.unpacked_path_segments,
                      raw_route.source_traversed_in_reverse, raw_route.target_traversed_in_reverse);
        if (raw_route.has_alternative())
        {
            std::vector<std::vector<PathData>> wrapped_leg(1);
            wrapped_leg.front() = std::move(raw_route.unpacked_alternative);
            routes.values[1] = MakeRoute(raw_route.segment_end_coordinates, wrapped_leg,
                                         raw_route.alt_source_traversed_in_reverse,
                                         raw_route.alt_target_traversed_in_reverse);
        }
        response.values["waypoints"] = BaseAPI::MakeWaypoints(raw_route.segment_end_coordinates);
        response.values["routes"] = std::move(routes);
        response.values["code"] = "ok";
    }

  protected:
    template <typename ForwardIter>
    util::json::Value MakeGeometry(ForwardIter begin, ForwardIter end) const
    {
        if (parameters.geometries == RouteParameters::GeometriesType::Polyline)
        {
            return json::makePolyline(begin, end);
        }

        BOOST_ASSERT(parameters.geometries == RouteParameters::GeometriesType::GeoJSON);
        return json::makeGeoJSONLineString(begin, end);
    }

    util::json::Object MakeRoute(const std::vector<PhantomNodes> &segment_end_coordinates,
                                 const std::vector<std::vector<PathData>> &unpacked_path_segments,
                                 const std::vector<bool> &source_traversed_in_reverse,
                                 const std::vector<bool> &target_traversed_in_reverse) const
    {
        std::vector<guidance::RouteLeg> legs;
        std::vector<guidance::LegGeometry> leg_geometries;
        auto number_of_legs = segment_end_coordinates.size();
        legs.reserve(number_of_legs);
        leg_geometries.reserve(number_of_legs);
        for (auto idx : util::irange(0UL, number_of_legs))
        {
            const auto &phantoms = segment_end_coordinates[idx];
            const auto &path_data = unpacked_path_segments[idx];
            const bool reversed_source = source_traversed_in_reverse[idx];
            const bool reversed_target = target_traversed_in_reverse[idx];

            auto leg_geometry = guidance::assembleGeometry(
                BaseAPI::facade, path_data, phantoms.source_phantom, phantoms.target_phantom);
            auto leg = guidance::assembleLeg(BaseAPI::facade, path_data, leg_geometry,
                                             phantoms.source_phantom, phantoms.target_phantom,
                                             reversed_source, reversed_target);

            if (parameters.steps)
            {
                leg.steps = guidance::assembleSteps(
                    BaseAPI::facade, path_data, leg_geometry, phantoms.source_phantom,
                    phantoms.target_phantom, reversed_source, reversed_target);
            }

            leg_geometries.push_back(std::move(leg_geometry));
            legs.push_back(std::move(leg));
        }
        auto route = guidance::assembleRoute(legs);
        boost::optional<util::json::Value> json_overview;
        if (parameters.overview != RouteParameters::OverviewType::False)
        {
            const auto use_simplification =
                parameters.overview == RouteParameters::OverviewType::Simplified;
            BOOST_ASSERT(use_simplification ||
                         parameters.overview == RouteParameters::OverviewType::Full);

            auto overview = guidance::assembleOverview(leg_geometries, use_simplification);
            json_overview = MakeGeometry(overview.begin(), overview.end());
        }

        return json::makeRoute(route, json::makeRouteLegs(std::move(legs), leg_geometries),
                               std::move(json_overview));
    }

    const RouteParameters &parameters;
};

} // ns api
} // ns engine
} // ns osrm

#endif

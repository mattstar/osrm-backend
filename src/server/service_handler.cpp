#include "server/service_handler.hpp"

#include "server/service/route_service.hpp"
#include "server/service/table_service.hpp"
#include "server/service/nearest_service.hpp"
#include "server/service/trip_service.hpp"
#include "server/service/match_service.hpp"

#include "server/api/parsed_url.hpp"
#include "util/json_util.hpp"
#include "util/make_unique.hpp"

namespace osrm
{
namespace server
{
ServiceHandler::ServiceHandler(osrm::EngineConfig &config) : routing_machine(config)
{
    service_map["route"] = util::make_unique<service::RouteService>(routing_machine);
    service_map["table"] = util::make_unique<service::TableService>(routing_machine);
    service_map["nearest"] = util::make_unique<service::NearestService>(routing_machine);
    service_map["trip"] = util::make_unique<service::TripService>(routing_machine);
    service_map["match"] = util::make_unique<service::MatchService>(routing_machine);
}

engine::Status ServiceHandler::RunQuery(api::ParsedURL parsed_url, util::json::Object &result)
{
    const auto &service_iter = service_map.find(parsed_url.service);
    if (service_iter == service_map.end())
    {
        result.values["code"] = "invalid-service";
        result.values["message"] = "Service " + parsed_url.service + " not found!";
        return engine::Status::Error;
    }
    auto &service = service_iter->second;

    if (service->GetVersion() != parsed_url.version)
    {
        result.values["code"] = "invalid-version";
        result.values["message"] = "Service " + parsed_url.service + " not found!";
        return engine::Status::Error;
    }

    return service->RunQuery(std::move(parsed_url.coordinates), parsed_url.options, result);
}
}
}

#ifndef SERVER_SERVICE_NEAREST_SERVICE_HPP
#define SERVER_SERVICE_NEAREST_SERVICE_HPP

#include "server/service/base_service.hpp"

#include "engine/status.hpp"
#include "util/coordinate.hpp"
#include "osrm/osrm.hpp"

#include <string>
#include <vector>

namespace osrm
{
namespace server
{
namespace service
{

class NearestService final : public BaseService
{
  public:
    NearestService(OSRM &routing_machine) : BaseService(routing_machine) {}

    engine::Status RunQuery(std::vector<util::FixedPointCoordinate> coordinates,
                            std::string &options,
                            util::json::Object &result) final override;

    unsigned GetVersion() final override { return 1; }
};
}
}
}

#endif

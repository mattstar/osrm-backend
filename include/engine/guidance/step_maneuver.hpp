#ifndef ENGINE_GUIDANCE_STEP_MANEUVER_HPP
#define ENGINE_GUIDANCE_STEP_MANEUVER_HPP

#include "util/coordinate.hpp"
#include "engine/guidance/turn_instruction.hpp"

namespace osrm
{
namespace engine
{
namespace guidance
{

struct StepManeuver
{
    util::FixedPointCoordinate location;
    double heading_before;
    double heading_after;
    TurnInstruction instruction;
};
} // namespace guidance
} // namespace engine
} // namespace osrmn
#endif `

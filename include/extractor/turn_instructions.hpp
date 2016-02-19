#ifndef TURN_INSTRUCTIONS_HPP
#define TURN_INSTRUCTIONS_HPP

#include <algorithm>
#include <cmath>

#include <boost/assert.hpp>

namespace osrm
{
namespace extractor
{

enum class TurnInstruction : unsigned char
{
    None = 0,
    Continue,
    BearRight,
    TurnRight,
    SharpRight,
    UTurn,
    SharpLeft,
    TurnLeft,
    BearLeft,
    ReachedWaypointLocation,
    EnterRoundAbout,
    LeaveRoundAbout,
    StayOnRoundAbout,
    StartAtEndOfStreet,
    ReachedYourDestination,
    NameChanges,
    EnterAgainstAllowedDirection,
    LeaveAgainstAllowedDirection,
    InverseAccessRestrictionFlag = 127,
    AccessRestrictionFlag = 128,
    AccessRestrictionPenalty = 129
};

// shiftable turns to left and right
const constexpr bool shiftable_left[] = {false, false, true, true, true, false, false, true, true};
const constexpr bool shiftable_right[] = {false, false, true, true, false, false, true, true, true};

inline TurnInstruction shiftTurnToLeft(TurnInstruction turn)
{
    BOOST_ASSERT_MSG(static_cast<int>(turn) < 9,
                     "Shift turn only supports basic turn instructions");
    if (turn > TurnInstruction::TurnSlightLeft)
        return turn;
    else
        return shiftable_left[static_cast<int>(turn)]
                   ? (static_cast<TurnInstruction>(static_cast<int>(turn) - 1))
                   : turn;
}

inline TurnInstruction shiftTurnToRight(TurnInstruction turn)
{
    BOOST_ASSERT_MSG(static_cast<int>(turn) < 9,
                     "Shift turn only supports basic turn instructions");
    if (turn > TurnInstruction::TurnSlightLeft)
        return turn;
    else
        return shiftable_right[static_cast<int>(turn)]
                   ? (static_cast<TurnInstruction>(static_cast<int>(turn) + 1))
                   : turn;
}

inline double angularDeviation(const double angle, const double from)
{
    const double deviation = std::abs(angle - from);
    return std::min(360 - deviation, deviation);
}

inline double getAngularPenalty(const double angle, TurnInstruction instruction)
{
    BOOST_ASSERT_MSG(static_cast<int>(instruction) < 9,
                     "Angular penalty only supports basic turn instructions");
    const double center[] = {180, 180, 135, 90, 45,
                             0,   315, 270, 225}; // centers of turns from getTurnDirection
    return angularDeviation(center[static_cast<int>(instruction)], angle);
}

inline double getTurnConfidence(const double angle, TurnInstruction instruction)
{

    // special handling of U-Turns and Roundabout
    if (instruction >= TurnInstruction::HeadOn || instruction == TurnInstruction::UTurn ||
        instruction == TurnInstruction::NoTurn || instruction == TurnInstruction::EnterRoundAbout ||
        instruction == TurnInstruction::StayOnRoundAbout || instruction == TurnInstruction::LeaveRoundAbout )
        return 1.0;

    BOOST_ASSERT_MSG(static_cast<int>(instruction) < 9,
                     "Turn confidence only supports basic turn instructions");
    const double deviations[] = {10, 10, 35, 50, 45, 0, 45, 50, 35};
    const double difference = getAngularPenalty(angle, instruction);
    const double max_deviation = deviations[static_cast<int>(instruction)];
    return 1.0 - (difference / max_deviation) * (difference / max_deviation);
}

// Translates between angles and their human-friendly directional representation
//               180°
//               (w)
//          o     ^     o
//        o       ^       o
//                ^
// 270°  o       (v)       o  90°
//                ^
//        o       ^       o
//          o     ^     o
//               (u)
//                0°
inline TurnInstruction getTurnDirection(const double angle)
{
    // An angle of zero is a u-turn
    // 180 goes perfectly straight
    // 0-180 are right turns
    // 180-360 are left turns
    if (angle > 0 && angle < 60)
        return TurnInstruction::TurnSharpRight;
    if (angle >= 60 && angle < 140)
        return TurnInstruction::TurnRight;
    if (angle >= 140 && angle < 170)
        return TurnInstruction::TurnSlightRight;
    if (angle >= 170 && angle <= 190)
        return TurnInstruction::GoStraight;
    if (angle > 190 && angle <= 220)
        return TurnInstruction::TurnSlightLeft;
    if (angle > 220 && angle <= 300)
        return TurnInstruction::TurnLeft;
    if (angle > 300 && angle < 360)
        return TurnInstruction::TurnSharpLeft;
    return TurnInstruction::UTurn;
}

// Decides if a turn is needed to be done for the current instruction
inline bool isTurnNecessary(const TurnInstruction turn_instruction)
{
    if (TurnInstruction::None == turn_instruction ||
        TurnInstruction::StayOnRoundAbout == turn_instruction)
    {
        return false;
    }
    return true;
}

inline bool resolve(TurnInstruction &to_resolve, const TurnInstruction neighbor, bool resolve_right)
{
    const auto shifted_turn =
        resolve_right ? shiftTurnToRight(to_resolve) : shiftTurnToLeft(to_resolve);
    if (shifted_turn == neighbor || shifted_turn == to_resolve)
        return false;

    to_resolve = shifted_turn;
    return true;
}

inline bool resolveTransitive(TurnInstruction &first,
                              TurnInstruction &second,
                              const TurnInstruction third,
                              bool resolve_right)
{
    if (resolve(second, third, resolve_right))
    {
        first = resolve_right ? shiftTurnToRight(first) : shiftTurnToLeft(first);
        return true;
    }
    return false;
}

inline bool isSlightTurn(const TurnInstruction turn)
{
    return turn == TurnInstruction::GoStraight || turn == TurnInstruction::TurnSlightRight ||
           turn == TurnInstruction::TurnSlightLeft || turn == TurnInstruction::NoTurn;
}

inline bool isSharpTurn(const TurnInstruction turn)
{
    return turn == TurnInstruction::TurnSharpLeft || turn == TurnInstruction::TurnSharpRight;
}

inline bool isStraight(const TurnInstruction turn)
{
    return turn == TurnInstruction::GoStraight || turn == TurnInstruction::NoTurn;
}

inline bool isConflict(const TurnInstruction first, const TurnInstruction second)
{
    return first == second || (isStraight(first) && isStraight(second));
}
}
}

#endif /* TURN_INSTRUCTIONS_HPP */

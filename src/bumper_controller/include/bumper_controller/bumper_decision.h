// bumper_decision.h — pure (ROS-free) choice of the bumper manoeuvre from the docking
// phase and which bumper was hit. Unit-tested in test/test_bumper_decision.cpp.
//
// Owner rules: never move blind toward the dock; every turn uses both wheels (the
// rotate-clear is a pivot); safety first (when unsure, hold rather than move).
//
//   dock phase (/mower_docking/state)        | front hit                | rear hit (front free)
//   -----------------------------------------+--------------------------+---------------------------
//   none / stale / other (mowing, manual...) | reverse back_distance    | (same, unchanged)
//                                            | + rotate clear           |
//   APPROACH: NAV_TO_APPROACH, ALIGNING,     | reverse dock_backoff_    | hold dock_hold_s
//             SEARCHING                      | distance, no turn, hold  |
//   REVERSE: DOCKING, FINAL_DOCKING, RETRY   | hold dock_hold_s (zero)  | forward dock_rear_backoff_
//                                            |                          | distance, then hold
//
// The Tron chassis only reports front bumper bits (bumper / bumper_l / bumper_r), so the
// rear column is never taken on this hardware; it is kept for completeness.
#pragma once
#include <cmath>
#include <string>

namespace mower_controller {

enum class DockPhase { NONE = 0, APPROACH = 1, REVERSE = 2 };

inline DockPhase dockPhaseFromState(const std::string& s) {
    if (s == "NAV_TO_APPROACH" || s == "ALIGNING" || s == "SEARCHING") return DockPhase::APPROACH;
    if (s == "DOCKING" || s == "FINAL_DOCKING" || s == "RETRY") return DockPhase::REVERSE;
    return DockPhase::NONE;
}

struct ManoeuvrePlan {
    double distance = 0.0;  // m to drive straight (0 = none)
    double speed = 0.0;     // signed m/s for that leg (<0 reverse, >0 forward)
    bool rotate = false;    // run the rotate-clear afterwards (legacy behaviour)
    double hold_s = 0.0;    // then publish zero for this long (bounded by max_duration_s)
};

struct DecisionConfig {
    double back_distance = 0.30;
    double back_speed = -0.20;
    double dock_backoff_distance = 0.10;
    double dock_rear_backoff_distance = 0.15;
    double dock_hold_s = 1.0;
};

inline ManoeuvrePlan decideManoeuvre(DockPhase phase, bool front, bool rear,
                                     const DecisionConfig& c) {
    const double v = std::fabs(c.back_speed);
    ManoeuvrePlan p;
    if (phase == DockPhase::NONE) {
        p.distance = c.back_distance;
        p.speed = -v;
        p.rotate = true;
        return p;
    }
    p.hold_s = c.dock_hold_s;
    if (phase == DockPhase::APPROACH) {
        if (front) {               // short straight reverse, no turn: re-plan from ~same spot
            p.distance = c.dock_backoff_distance;
            p.speed = -v;
        }
        return p;                  // rear-only: hold
    }
    // REVERSE (backing onto the dock): never reverse further toward it.
    if (rear && !front) {
        p.distance = c.dock_rear_backoff_distance;
        p.speed = v;               // forward, away from the dock
    }
    return p;                      // front (or both): hold
}

}  // namespace mower_controller

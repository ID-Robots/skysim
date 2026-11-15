#pragma once
// The vehicle's camera gimbal, read from the servo outputs ArduPilot sends every frame
// (TASK-2548).
//
// A SkyHub SITL is built with ArduPilot's BrushlessPWM mount (MNT1_TYPE 7): a self-stabilising
// gimbal whose earth-frame pitch and vehicle-relative yaw ArduPilot drives as PWM on two servo
// outputs, the way it would drive a real one. The mission points it (DO_MOUNT_CONTROL: -60
// degrees for a 3D mapping survey), so decoding those two outputs is how the pilot camera and
// the survey stills look where the autopilot thinks its camera looks — in lockstep with the
// physics, with no second channel to the vehicle.
//
// The numbers are a contract with whoever builds the vehicle (the gateway's
// SITLDroneService._boot_params): pitch on output 10 over -90..20 degrees, yaw on output 11
// over -180..180, both across 1100..1900 us. ArduPilot maps a mount angle linearly between
// SERVOn_MIN and SERVOn_MAX over MNT1_<axis>_MIN..MAX; measured on ArduCopter 4.8 SITL, a -60
// degree DO_MOUNT_CONTROL put output 10 at 1318 us.
#include <algorithm>
#include <array>
#include <cstdint>

namespace skysim::vehicle {

struct GimbalConfig {
    int pitch_channel{10}; // 1-based servo output; 0 = vehicles have no gimbal
    int yaw_channel{11};   // 1-based; 0 = the gimbal has no yaw axis (it looks along the nose)
    double pitch_min_deg{-90.0};
    double pitch_max_deg{20.0};
    double yaw_min_deg{-180.0};
    double yaw_max_deg{180.0};
    double pwm_min_us{1100.0};
    double pwm_max_us{1900.0};
};

struct GimbalState {
    bool present{false};   // the vehicle drives a gimbal
    double pitch_deg{0.0}; // earth frame: -90 straight down, 0 level
    double yaw_deg{0.0};   // relative to the vehicle's heading, clockwise
};

// A servo output that carries a pulse. ArduPilot sends 0 on an output with no function, so a
// vehicle built without a gimbal has nothing here.
inline bool live_pulse(uint16_t pwm_us) { return pwm_us >= 800 && pwm_us <= 2200; }

inline double pulse_to_angle(uint16_t pwm_us, double pwm_min, double pwm_max, double min_deg,
                             double max_deg) {
    if (!(pwm_max > pwm_min)) {
        return min_deg;
    }
    const double t = std::clamp((static_cast<double>(pwm_us) - pwm_min) / (pwm_max - pwm_min), 0.0, 1.0);
    return min_deg + t * (max_deg - min_deg);
}

// The gimbal as one servo frame drives it. ``channel_count`` outputs of ``pwm`` are valid.
inline GimbalState gimbal_from_servos(const GimbalConfig &cfg, const std::array<uint16_t, 32> &pwm,
                                      int channel_count) {
    GimbalState g;
    const int count = std::clamp(channel_count, 0, static_cast<int>(pwm.size()));
    if (cfg.pitch_channel < 1 || cfg.pitch_channel > count) {
        return g;
    }
    const uint16_t pitch_us = pwm[static_cast<size_t>(cfg.pitch_channel - 1)];
    if (!live_pulse(pitch_us)) {
        return g;
    }
    g.present = true;
    g.pitch_deg =
        pulse_to_angle(pitch_us, cfg.pwm_min_us, cfg.pwm_max_us, cfg.pitch_min_deg, cfg.pitch_max_deg);
    if (cfg.yaw_channel >= 1 && cfg.yaw_channel <= count) {
        const uint16_t yaw_us = pwm[static_cast<size_t>(cfg.yaw_channel - 1)];
        if (live_pulse(yaw_us)) {
            g.yaw_deg =
                pulse_to_angle(yaw_us, cfg.pwm_min_us, cfg.pwm_max_us, cfg.yaw_min_deg, cfg.yaw_max_deg);
        }
    }
    return g;
}

} // namespace skysim::vehicle

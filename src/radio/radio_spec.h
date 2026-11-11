#pragma once
// The simulated radio field's spec: what PUT /radio and --radio-spec accept, validated.
// The model it drives is in radio_model.h; the full description is docs/RADIO.md.
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace skysim::radio {

// Size limits. They bound the work one sample costs (every tower is evaluated for every
// position) and match what a gateway validates before it forwards a spec here.
inline constexpr size_t kMaxCarriers = 8;
inline constexpr size_t kMaxTowers = 64; // across all carriers
inline constexpr size_t kMaxHoles = 64;
inline constexpr size_t kMaxNameLength = 64; // carrier names (shown as the operator) and tower ids
// Largest body PUT /radio or --radio-spec reads. A spec at every limit above is ~20 KB.
inline constexpr size_t kMaxSpecBytes = 256 * 1024;

struct Tower {
    std::string id;
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    // Antenna height on the same vertical axis as the vehicle's h, i.e. metres AMSL. Over
    // ground at 0 m AMSL (the reference case) that is also its height above ground.
    double height_m = 0.0;
    double rsrp_1km_dbm = 0.0;
    std::optional<double> azimuth_deg;   // absent or null: omnidirectional
    std::optional<double> beamwidth_deg; // required for a sector, unused by an omni
    double downtilt_deg = 0.0;
    std::optional<int> pci;
};

struct Carrier {
    std::string name; // reported as the operator
    std::optional<std::string> mcc_mnc;
    std::optional<std::string> rat;
    std::optional<int> band;
    std::optional<int> earfcn;
    std::vector<Tower> towers;
};

struct Hole {
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double radius_m = 0.0;
    double extra_loss_db = 0.0;
};

struct RadioSpec {
    uint64_t seed = 0;
    double shadowing_sigma_db = 0.0;
    double shadow_corr_m = 0.0;
    double noise_floor_dbm = 0.0;
    // Carrier names, one per simulated modem; [0] is the primary. Defaults to every carrier
    // in spec order when the spec leaves it out.
    std::vector<std::string> modems;
    std::vector<Carrier> carriers;
    std::vector<Hole> holes;
};

struct SpecResult {
    std::optional<RadioSpec> spec;
    std::string error; // set when spec is empty: what is wrong and where ("radio.carriers[0].name: ...")
};

// Parses and validates a spec. Accepts the wrapped form {"radio": {...}} — so a whole
// simulator-link object can be passed through with its other keys ignored — or the bare
// spec object. Unknown keys are ignored; every number must be finite.
SpecResult parse_radio_spec(std::string_view body);

// The spec as canonical JSON (the bare object, defaults filled in). Feeding it back to
// parse_radio_spec yields the same spec.
std::string radio_spec_json(const RadioSpec &spec);

} // namespace skysim::radio

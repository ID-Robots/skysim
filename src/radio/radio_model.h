#pragma once
// Simulated 4G (LTE) radio field: deterministic RSRP / SINR / RSRQ / RSSI per carrier at any
// position, from towers, antenna patterns, correlated shadowing and coverage holes.
//
// The model is a shared contract: a flight recorder elsewhere implements the same formulas,
// and the two are cross-checked against tests/fixtures/radio_test_vector.json. Change a
// formula here and that file — and the other implementation — must change with it. The
// formulas, frames and API are described in docs/RADIO.md.
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "radio/radio_spec.h"

namespace skysim::radio {

inline constexpr double kEarthRadiusM = 6371000.0;
inline constexpr double kMinDistanceM = 10.0;
// Resource blocks the RSSI is integrated over (10 MHz LTE): RSSI adds 10·log10(12·50).
inline constexpr int kResourceBlocks = 50;
// Registration thresholds: below either the modem has no service.
inline constexpr double kMinRegisteredRsrpDbm = -124.0;
inline constexpr double kMinRegisteredSinrDb = -8.0;

// --- the individual terms, exposed so each can be tested on its own ------------------------

// Path-loss exponent: 3.5 on the ground falling 0.01 per metre above home, floored at 2.0
// (free space) — more line of sight higher up.
double path_loss_exponent(double alt_rel_m);

// Smallest angle between two bearings, in [0, 180] degrees.
double angle_between_deg(double a_deg, double b_deg);

// Horizontal antenna pattern loss (dB). nullopt azimuth is an omni: 0.
double horizontal_pattern_db(std::optional<double> azimuth_deg, double beamwidth_deg, double bearing_deg);

// Vertical antenna pattern loss (dB) at elevation θ above the antenna's horizon.
double vertical_pattern_db(double elevation_deg, double downtilt_deg);

// One splitmix64 step: advances `state` and returns the next output.
uint64_t splitmix64_next(uint64_t &state);

// Standard normal value of shadowing grid node (ix, iy) for carrier `carrier_index`:
// splitmix64 seeded with seed ^ ix·73856093 ^ iy·19349663 ^ c·83492791 (64-bit wrapping),
// two draws → Box–Muller, first output.
double shadow_node(uint64_t seed, int64_t ix, int64_t iy, uint64_t carrier_index);

double dbm_to_mw(double dbm);
double mw_to_dbm(double mw);

// --- positions -----------------------------------------------------------------------------

// A point the field is sampled at. alt_rel_m drives the path-loss exponent; alt_amsl_m is
// the vertical coordinate compared with tower heights.
struct Position {
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double alt_rel_m = 0.0;
    double alt_amsl_m = 0.0;
};

// Where a world's NED origin sits on the earth: the spawn home, and its ground's AMSL height.
struct GeoAnchor {
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double ground_amsl_m = 0.0;
};

// A world-frame NED position (metres from the anchor) as a Position: equirectangular about
// the anchor, alt_rel = -down, alt_amsl = ground + alt_rel.
Position position_from_ned(const GeoAnchor &anchor, const double pos_ned[3]);

// --- results -------------------------------------------------------------------------------

enum class LinkQuality { kNone, kBad, kWeak, kGood };

const char *to_string(LinkQuality q);

// One carrier's values at one position. All values are computed even when unregistered (the
// API writes them as null then); `rsrp_dbm` is the contract's rounded figure and
// `rsrp_exact_dbm` the serving tower's unrounded RSRP.
struct CarrierSample {
    size_t serving_tower = 0; // index into the carrier's towers
    double rsrp_exact_dbm = 0.0;
    double rsrp_dbm = 0.0; // rounded to 0.1 dB
    double sinr_db = 0.0;  // clamped to [-20, 40]
    double rsrq_db = 0.0;  // clamped to [-19.5, -3]
    double rssi_dbm = 0.0;
    bool registered = false;
};

// none if unregistered; bad if rsrp < -110 or sinr < 0; weak if rsrp < -100 or sinr < 5;
// else good.
LinkQuality classify(const CarrierSample &s);

// --- the model -----------------------------------------------------------------------------

// Immutable once built, so any number of threads may sample one concurrently.
class RadioModel {
  public:
    explicit RadioModel(RadioSpec spec);

    const RadioSpec &spec() const { return spec_; }

    // Local metric frame: east/north metres from the first tower of the first carrier,
    // equirectangular (x = R·Δlon·cos(lat0), y = R·Δlat). All horizontal geometry —
    // distances, bearings, holes, the shadowing grid — is measured in it.
    void to_local(double lat_deg, double lon_deg, double &x_east_m, double &y_north_m) const;

    // Correlated shadowing S (dB) for carrier `carrier_index` at a local-frame point:
    // bilinear between the four surrounding grid nodes, times shadowing_sigma_db.
    double shadowing_db(size_t carrier_index, double x_east_m, double y_north_m) const;

    // Hole loss H (dB) at a local-frame point, summed over the holes covering it.
    double hole_loss_db(double x_east_m, double y_north_m) const;

    // RSRP (dBm) of one tower at a position, every term included.
    double tower_rsrp_dbm(size_t carrier_index, size_t tower_index, const Position &p) const;

    // The serving tower and the carrier's values at a position.
    CarrierSample sample(size_t carrier_index, const Position &p) const;

    // Every carrier, in spec order.
    std::vector<CarrierSample> sample_all(const Position &p) const;

    // Carrier indices of the modems; [0] is the primary.
    const std::vector<size_t> &modem_carriers() const { return modem_carriers_; }

    // Index of a carrier by name.
    std::optional<size_t> carrier_index(const std::string &name) const;

  private:
    struct LocalPoint {
        double x = 0.0;
        double y = 0.0;
    };

    // Step 7 for one tower at local point (x, y), given S - H for that carrier and point.
    double tower_rsrp_at(size_t carrier_index, size_t tower_index, const Position &p, double x, double y,
                         double shadowing_minus_holes_db) const;

    RadioSpec spec_;
    double lat0_deg_ = 0.0;
    double lon0_deg_ = 0.0;
    double cos_lat0_ = 1.0;
    std::vector<std::vector<LocalPoint>> towers_; // [carrier][tower]
    std::vector<LocalPoint> holes_;
    std::vector<size_t> modem_carriers_;
};

// --- JSON views ------------------------------------------------------------------------------

// The values at one position: the link quality, the primary modem as `cell`, the other
// modems as `cells` (each with `modem` = its carrier name), and every carrier in spec order
// as `carriers`. Cell objects use the flight-recorder sample field names (docs/RADIO.md).
std::string sample_json(const RadioModel &model, const Position &p);

// A ground-truth raster over a lat/lon box: cell centres, row-major from the south-west
// corner, `step_m` apart. One layer per requested carrier.
struct GridRequest {
    double min_lon_deg = 0.0;
    double min_lat_deg = 0.0;
    double max_lon_deg = 0.0;
    double max_lat_deg = 0.0;
    double step_m = 50.0;
    double alt_rel_m = 0.0;
    double alt_amsl_m = 0.0;
    std::vector<size_t> carriers; // indices; every carrier when empty
};

struct GridShape {
    size_t rows = 0;
    size_t cols = 0;
    double step_lat_deg = 0.0;
    double step_lon_deg = 0.0;
};

// Rows/columns for a request. The longitude step uses the box's mid latitude. Returns
// nullopt when the grid would exceed `max_cells` (rows × cols × layers).
std::optional<GridShape> grid_shape(const GridRequest &req, size_t layers, size_t max_cells);

std::string grid_json(const RadioModel &model, const GridRequest &req, const GridShape &shape);

// --- the live field --------------------------------------------------------------------------

// The current model, shared between the control plane's threads. Readers take a reference
// to the model and sample it without holding any lock; a new spec replaces the model in one
// swap, so a reader sees either the old field or the new one, never a mix.
class RadioField {
  public:
    std::shared_ptr<const RadioModel> get() const;
    void set(std::shared_ptr<const RadioModel> model);
    // Clears the field; returns whether one was set.
    bool clear();

  private:
    mutable std::mutex mutex_;
    std::shared_ptr<const RadioModel> model_;
};

} // namespace skysim::radio

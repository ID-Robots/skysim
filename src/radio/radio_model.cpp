// See radio_model.h. Every formula below is the shared contract's, term for term; the
// comments name the step they implement so the two implementations can be read side by side.
#include "radio/radio_model.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <utility>

#include "radio/json.h"

namespace skysim::radio {

namespace {

constexpr double kDegToRad = std::numbers::pi / 180.0;
constexpr double kRadToDeg = 180.0 / std::numbers::pi;
// Below this the cosine of a latitude is treated as this, so a pole maps to a finite
// (if meaningless) longitude instead of a division by zero.
constexpr double kMinCosLat = 1e-9;

double round_to(double v, double unit) { return std::round(v / unit) * unit; }

} // namespace

double path_loss_exponent(double alt_rel_m) { return std::max(2.0, 3.5 - 0.01 * std::max(0.0, alt_rel_m)); }

double angle_between_deg(double a_deg, double b_deg) {
    double d = std::fmod(std::fabs(a_deg - b_deg), 360.0);
    return d > 180.0 ? 360.0 - d : d;
}

double horizontal_pattern_db(std::optional<double> azimuth_deg, double beamwidth_deg, double bearing_deg) {
    if (!azimuth_deg.has_value()) {
        return 0.0;
    }
    const double r = angle_between_deg(bearing_deg, *azimuth_deg) / beamwidth_deg;
    return std::min(20.0, 12.0 * r * r);
}

double vertical_pattern_db(double elevation_deg, double downtilt_deg) {
    const double r = (elevation_deg + downtilt_deg) / 10.0;
    return std::min(20.0, 12.0 * r * r);
}

uint64_t splitmix64_next(uint64_t &state) {
    state += 0x9E3779B97F4A7C15ULL;
    uint64_t z = state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

double shadow_node(uint64_t seed, int64_t ix, int64_t iy, uint64_t carrier_index) {
    // Negative indices wrap as two's complement, which is what masking an arbitrary-precision
    // integer to 64 bits gives as well.
    uint64_t state = seed ^ (static_cast<uint64_t>(ix) * 73856093ULL) ^
                     (static_cast<uint64_t>(iy) * 19349663ULL) ^ (carrier_index * 83492791ULL);
    constexpr double k2Pow53 = 1.0 / 9007199254740992.0; // 2^-53
    const double u1 = std::max(1e-12, static_cast<double>(splitmix64_next(state) >> 11) * k2Pow53);
    const double u2 = static_cast<double>(splitmix64_next(state) >> 11) * k2Pow53;
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * std::numbers::pi * u2);
}

double dbm_to_mw(double dbm) { return std::pow(10.0, dbm / 10.0); }

double mw_to_dbm(double mw) { return 10.0 * std::log10(mw); }

Position position_from_ned(const GeoAnchor &anchor, const double pos_ned[3]) {
    const double cos_lat = std::max(kMinCosLat, std::cos(anchor.lat_deg * kDegToRad));
    Position p;
    p.lat_deg = anchor.lat_deg + pos_ned[0] / kEarthRadiusM * kRadToDeg;
    p.lon_deg = anchor.lon_deg + pos_ned[1] / (kEarthRadiusM * cos_lat) * kRadToDeg;
    p.alt_rel_m = -pos_ned[2];
    p.alt_amsl_m = anchor.ground_amsl_m + p.alt_rel_m;
    return p;
}

const char *to_string(LinkQuality q) {
    switch (q) {
    case LinkQuality::kGood:
        return "good";
    case LinkQuality::kWeak:
        return "weak";
    case LinkQuality::kBad:
        return "bad";
    case LinkQuality::kNone:
        break;
    }
    return "none";
}

LinkQuality classify(const CarrierSample &s) {
    if (!s.registered) {
        return LinkQuality::kNone;
    }
    if (s.rsrp_dbm < -110.0 || s.sinr_db < 0.0) {
        return LinkQuality::kBad;
    }
    if (s.rsrp_dbm < -100.0 || s.sinr_db < 5.0) {
        return LinkQuality::kWeak;
    }
    return LinkQuality::kGood;
}

RadioModel::RadioModel(RadioSpec spec) : spec_(std::move(spec)) {
    // The frame is anchored on the first tower of the first carrier. A spec from
    // parse_radio_spec always has one; a hand-built one without gets the origin instead.
    if (!spec_.carriers.empty() && !spec_.carriers.front().towers.empty()) {
        lat0_deg_ = spec_.carriers.front().towers.front().lat_deg;
        lon0_deg_ = spec_.carriers.front().towers.front().lon_deg;
    }
    cos_lat0_ = std::max(kMinCosLat, std::cos(lat0_deg_ * kDegToRad));

    for (const Carrier &c : spec_.carriers) {
        std::vector<LocalPoint> local;
        for (const Tower &t : c.towers) {
            LocalPoint lp;
            to_local(t.lat_deg, t.lon_deg, lp.x, lp.y);
            local.push_back(lp);
        }
        towers_.push_back(std::move(local));
    }
    for (const Hole &h : spec_.holes) {
        LocalPoint lp;
        to_local(h.lat_deg, h.lon_deg, lp.x, lp.y);
        holes_.push_back(lp);
    }
    for (const std::string &name : spec_.modems) {
        if (const auto idx = carrier_index(name)) {
            modem_carriers_.push_back(*idx);
        }
    }
    // A spec that names no modem (only possible when built by hand) gets one per carrier,
    // the same default parse_radio_spec applies.
    if (modem_carriers_.empty()) {
        for (size_t i = 0; i < spec_.carriers.size(); ++i) {
            modem_carriers_.push_back(i);
        }
    }
}

std::optional<size_t> RadioModel::carrier_index(const std::string &name) const {
    for (size_t i = 0; i < spec_.carriers.size(); ++i) {
        if (spec_.carriers[i].name == name) {
            return i;
        }
    }
    return std::nullopt;
}

void RadioModel::to_local(double lat_deg, double lon_deg, double &x_east_m, double &y_north_m) const {
    // Δlon taken the short way round, so a field straddling the antimeridian stays local.
    double dlon = lon_deg - lon0_deg_;
    if (dlon > 180.0) {
        dlon -= 360.0;
    } else if (dlon < -180.0) {
        dlon += 360.0;
    }
    x_east_m = kEarthRadiusM * dlon * kDegToRad * cos_lat0_;
    y_north_m = kEarthRadiusM * (lat_deg - lat0_deg_) * kDegToRad;
}

double RadioModel::shadowing_db(size_t carrier_index, double x_east_m, double y_north_m) const {
    // Step 5: grid of shadow_corr_m cells; ix = floor(x / corr), fractional part = weight.
    const double gx = x_east_m / spec_.shadow_corr_m;
    const double gy = y_north_m / spec_.shadow_corr_m;
    const double fx0 = std::floor(gx);
    const double fy0 = std::floor(gy);
    const auto ix = static_cast<int64_t>(fx0);
    const auto iy = static_cast<int64_t>(fy0);
    const double fx = gx - fx0;
    const double fy = gy - fy0;
    const uint64_t c = carrier_index;
    const uint64_t seed = spec_.seed;
    const double g = shadow_node(seed, ix, iy, c) * (1.0 - fx) * (1.0 - fy) +
                     shadow_node(seed, ix + 1, iy, c) * fx * (1.0 - fy) +
                     shadow_node(seed, ix, iy + 1, c) * (1.0 - fx) * fy +
                     shadow_node(seed, ix + 1, iy + 1, c) * fx * fy;
    return g * spec_.shadowing_sigma_db;
}

double RadioModel::hole_loss_db(double x_east_m, double y_north_m) const {
    // Step 6: extra_loss · sqrt(max(0, 1 - dist2d / radius)), summed.
    double h = 0.0;
    for (size_t i = 0; i < holes_.size(); ++i) {
        const Hole &hole = spec_.holes[i];
        const double d = std::hypot(x_east_m - holes_[i].x, y_north_m - holes_[i].y);
        h += hole.extra_loss_db * std::sqrt(std::max(0.0, 1.0 - d / hole.radius_m));
    }
    return h;
}

double RadioModel::tower_rsrp_dbm(size_t carrier_index, size_t tower_index, const Position &p) const {
    double x = 0.0;
    double y = 0.0;
    to_local(p.lat_deg, p.lon_deg, x, y);
    return tower_rsrp_at(carrier_index, tower_index, p, x, y,
                         shadowing_db(carrier_index, x, y) - hole_loss_db(x, y));
}

double RadioModel::tower_rsrp_at(size_t carrier_index, size_t tower_index, const Position &p, double x,
                                 double y, double shadowing_minus_holes_db) const {
    const Tower &t = spec_.carriers[carrier_index].towers[tower_index];
    const LocalPoint &tl = towers_[carrier_index][tower_index];

    const double dx = x - tl.x;
    const double dy = y - tl.y;
    const double ground_m = std::hypot(dx, dy);
    const double dh = p.alt_amsl_m - t.height_m;
    // Step 1: 3D distance, at least 10 m.
    const double d_km = std::max(kMinDistanceM, std::hypot(ground_m, dh)) / 1000.0;
    // Step 2: exponent from height above home.
    const double n = path_loss_exponent(p.alt_rel_m);
    // Step 3: bearing tower → vehicle, clockwise from north.
    const double bearing = std::atan2(dx, dy) * kRadToDeg;
    const double a_h = horizontal_pattern_db(t.azimuth_deg, t.beamwidth_deg.value_or(0.0), bearing);
    // Step 4: elevation of the vehicle above the antenna's horizon.
    const double a_v = vertical_pattern_db(std::atan2(dh, ground_m) * kRadToDeg, t.downtilt_deg);
    // Step 7, with S - H (steps 5 and 6) passed in: both depend only on the carrier and the
    // position, so a sample computes them once rather than once per tower.
    return t.rsrp_1km_dbm - 10.0 * n * std::log10(d_km) - a_h - a_v + shadowing_minus_holes_db;
}

CarrierSample RadioModel::sample(size_t carrier_index, const Position &p) const {
    const Carrier &c = spec_.carriers[carrier_index];
    double x = 0.0;
    double y = 0.0;
    to_local(p.lat_deg, p.lon_deg, x, y);
    const double s_minus_h = shadowing_db(carrier_index, x, y) - hole_loss_db(x, y);
    std::vector<double> rsrp(c.towers.size());
    for (size_t t = 0; t < c.towers.size(); ++t) {
        rsrp[t] = tower_rsrp_at(carrier_index, t, p, x, y, s_minus_h);
    }
    CarrierSample out;
    // Serving tower = the strongest; the first listed wins a tie.
    for (size_t t = 1; t < rsrp.size(); ++t) {
        if (rsrp[t] > rsrp[out.serving_tower]) {
            out.serving_tower = t;
        }
    }
    const double s_lin = dbm_to_mw(rsrp[out.serving_tower]);
    double i_lin = 0.0;
    for (size_t t = 0; t < rsrp.size(); ++t) {
        i_lin += t == out.serving_tower ? 0.0 : dbm_to_mw(rsrp[t]);
    }
    const double n_lin = dbm_to_mw(spec_.noise_floor_dbm);

    out.rsrp_exact_dbm = rsrp[out.serving_tower];
    out.rsrp_dbm = round_to(out.rsrp_exact_dbm, 0.1);
    out.sinr_db = std::clamp(mw_to_dbm(s_lin / (i_lin + n_lin)), -20.0, 40.0);
    const double sinr_lin = dbm_to_mw(out.sinr_db);
    out.rsrq_db = std::clamp(-3.0 - mw_to_dbm(1.0 + 1.0 / sinr_lin) - 5.0, -19.5, -3.0);
    out.rssi_dbm = mw_to_dbm(s_lin + i_lin + n_lin) + mw_to_dbm(12.0 * kResourceBlocks);
    out.registered = out.rsrp_dbm >= kMinRegisteredRsrpDbm && out.sinr_db >= kMinRegisteredSinrDb;
    return out;
}

std::vector<CarrierSample> RadioModel::sample_all(const Position &p) const {
    std::vector<CarrierSample> out;
    out.reserve(spec_.carriers.size());
    for (size_t c = 0; c < spec_.carriers.size(); ++c) {
        out.push_back(sample(c, p));
    }
    return out;
}

namespace {

void append_cell(std::string &out, const RadioModel &model, size_t carrier, const CarrierSample &s,
                 const std::optional<std::string> &modem) {
    const Carrier &c = model.spec().carriers[carrier];
    const Tower &t = c.towers[s.serving_tower];
    const bool on = s.registered;
    auto optional_int = [](const std::optional<int> &v) {
        return v ? std::to_string(*v) : std::string("null");
    };

    out += R"({"source":"sim","modem":)";
    if (modem.has_value()) {
        json::append_string(out, *modem);
    } else {
        out += "null";
    }
    out += R"(,"operator":)";
    json::append_string(out, c.name);
    out += R"(,"mcc_mnc":)";
    if (c.mcc_mnc.has_value()) {
        json::append_string(out, *c.mcc_mnc);
    } else {
        out += "null";
    }
    out += R"(,"rat":)";
    if (c.rat.has_value()) {
        json::append_string(out, *c.rat);
    } else {
        out += "null";
    }
    out += R"(,"band":)" + optional_int(c.band);
    out += R"(,"earfcn":)" + optional_int(c.earfcn);
    // No service means no serving cell: its identity goes null with the signal values.
    out += R"(,"pci":)" + (on ? optional_int(t.pci) : std::string("null"));
    out += R"(,"cell_id":)";
    if (on) {
        json::append_string(out, c.mcc_mnc.has_value() ? *c.mcc_mnc + "-" + t.id : t.id);
        out += R"(,"tower":)";
        json::append_string(out, t.id);
    } else {
        out += R"(null,"tower":null)";
    }
    out += R"(,"rsrp":)";
    json::append_fixed(out, on ? std::optional<double>(s.rsrp_dbm) : std::nullopt, 1);
    out += R"(,"rsrq":)";
    json::append_fixed(out, on ? std::optional<double>(s.rsrq_db) : std::nullopt, 1);
    out += R"(,"sinr":)";
    json::append_fixed(out, on ? std::optional<double>(s.sinr_db) : std::nullopt, 1);
    out += R"(,"rssi":)";
    json::append_fixed(out, on ? std::optional<double>(s.rssi_dbm) : std::nullopt, 1);
    out += on ? R"(,"registered":true})" : R"(,"registered":false})";
}

} // namespace

std::string sample_json(const RadioModel &model, const Position &p) {
    const std::vector<CarrierSample> samples = model.sample_all(p);
    const std::vector<size_t> &modems = model.modem_carriers();
    const auto &carriers = model.spec().carriers;

    // The role a carrier plays: "primary", its own name for another modem, or none.
    auto modem_of = [&](size_t carrier) -> std::optional<std::string> {
        for (size_t m = 0; m < modems.size(); ++m) {
            if (modems[m] == carrier) {
                return m == 0 ? std::string("primary") : carriers[carrier].name;
            }
        }
        return std::nullopt;
    };

    // Callers bound the inputs (the API validates ranges; a vehicle is where physics put it),
    // so the fixed-point fields fit with room to spare.
    char buf[256];
    std::snprintf(buf, sizeof(buf), R"({"lat":%.7f,"lon":%.7f,"alt_rel":%.2f,"alt_amsl":%.2f,"link":")",
                  p.lat_deg, p.lon_deg, p.alt_rel_m, p.alt_amsl_m);
    std::string out = buf;
    out += to_string(classify(samples[modems.front()]));
    out += R"(","cell":)";
    append_cell(out, model, modems.front(), samples[modems.front()], "primary");
    out += R"(,"cells":[)";
    for (size_t m = 1; m < modems.size(); ++m) {
        out += m > 1 ? "," : "";
        append_cell(out, model, modems[m], samples[modems[m]], carriers[modems[m]].name);
    }
    out += R"(],"carriers":[)";
    for (size_t c = 0; c < samples.size(); ++c) {
        out += c > 0 ? "," : "";
        append_cell(out, model, c, samples[c], modem_of(c));
    }
    out += "]}";
    return out;
}

std::optional<GridShape> grid_shape(const GridRequest &req, size_t layers, size_t max_cells) {
    GridShape shape;
    const double mid_lat = 0.5 * (req.min_lat_deg + req.max_lat_deg);
    shape.step_lat_deg = req.step_m / kEarthRadiusM * kRadToDeg;
    shape.step_lon_deg =
        req.step_m / (kEarthRadiusM * std::max(kMinCosLat, std::cos(mid_lat * kDegToRad))) * kRadToDeg;
    // A box an exact multiple of the step must not grow a row from rounding noise.
    const double rows =
        std::max(1.0, std::ceil((req.max_lat_deg - req.min_lat_deg) / shape.step_lat_deg - 1e-9));
    const double cols =
        std::max(1.0, std::ceil((req.max_lon_deg - req.min_lon_deg) / shape.step_lon_deg - 1e-9));
    // Compared as doubles: a tiny step over a large box overflows size_t long before it is
    // rejected otherwise.
    if (rows * cols * static_cast<double>(std::max<size_t>(1, layers)) > static_cast<double>(max_cells)) {
        return std::nullopt;
    }
    shape.rows = static_cast<size_t>(rows);
    shape.cols = static_cast<size_t>(cols);
    return shape;
}

std::string grid_json(const RadioModel &model, const GridRequest &req, const GridShape &shape) {
    std::vector<size_t> carriers = req.carriers;
    if (carriers.empty()) {
        for (size_t c = 0; c < model.spec().carriers.size(); ++c) {
            carriers.push_back(c);
        }
    }
    const size_t cells = shape.rows * shape.cols;

    char buf[512];
    std::snprintf(
        buf, sizeof(buf),
        R"({"bbox":[%.7f,%.7f,%.7f,%.7f],"origin":{"lat":%.7f,"lon":%.7f},"step_m":%.3f,)"
        R"("step_deg":{"lat":%.9f,"lon":%.9f},"rows":%zu,"cols":%zu,"order":"row_major_south_to_north",)"
        R"("sample_at":"cell_center","alt_rel":%.2f,"alt_amsl":%.2f,"layers":[)",
        req.min_lon_deg, req.min_lat_deg, req.max_lon_deg, req.max_lat_deg, req.min_lat_deg, req.min_lon_deg,
        req.step_m, shape.step_lat_deg, shape.step_lon_deg, shape.rows, shape.cols, req.alt_rel_m,
        req.alt_amsl_m);
    std::string out = buf;
    // ~7 bytes per value, five arrays per layer.
    out.reserve(out.size() + carriers.size() * cells * 36);

    for (size_t l = 0; l < carriers.size(); ++l) {
        std::vector<CarrierSample> samples;
        samples.reserve(cells);
        for (size_t r = 0; r < shape.rows; ++r) {
            for (size_t col = 0; col < shape.cols; ++col) {
                Position p;
                p.lat_deg = req.min_lat_deg + (static_cast<double>(r) + 0.5) * shape.step_lat_deg;
                p.lon_deg = req.min_lon_deg + (static_cast<double>(col) + 0.5) * shape.step_lon_deg;
                p.alt_rel_m = req.alt_rel_m;
                p.alt_amsl_m = req.alt_amsl_m;
                samples.push_back(model.sample(carriers[l], p));
            }
        }
        out += l > 0 ? R"(,{"carrier":)" : R"({"carrier":)";
        json::append_string(out, model.spec().carriers[carriers[l]].name);
        const std::pair<const char *, double CarrierSample::*> metrics[] = {
            {"rsrp", &CarrierSample::rsrp_dbm},
            {"sinr", &CarrierSample::sinr_db},
            {"rsrq", &CarrierSample::rsrq_db},
            {"rssi", &CarrierSample::rssi_dbm}};
        for (const auto &[name, field] : metrics) {
            out += ",\"";
            out += name;
            out += "\":[";
            for (size_t i = 0; i < cells; ++i) {
                out += i > 0 ? "," : "";
                const CarrierSample &s = samples[i];
                json::append_fixed(out, s.registered ? std::optional<double>(s.*field) : std::nullopt, 1);
            }
            out += ']';
        }
        out += R"(,"quality":[)";
        for (size_t i = 0; i < cells; ++i) {
            out += i > 0 ? ",\"" : "\"";
            out += to_string(classify(samples[i]));
            out += '"';
        }
        out += "]}";
    }
    out += "]}";
    return out;
}

std::shared_ptr<const RadioModel> RadioField::get() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return model_;
}

void RadioField::set(std::shared_ptr<const RadioModel> model) {
    std::lock_guard<std::mutex> lock(mutex_);
    // The old model is released when `model` goes out of scope, after the lock: a reader
    // still sampling it keeps it alive, and nobody waits on its destruction here.
    model_.swap(model);
}

bool RadioField::clear() {
    std::shared_ptr<const RadioModel> old;
    std::lock_guard<std::mutex> lock(mutex_);
    old.swap(model_);
    return old != nullptr;
}

} // namespace skysim::radio

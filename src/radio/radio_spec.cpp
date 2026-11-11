// See radio_spec.h. Validation reports the first problem with its JSON path, because the
// caller is usually a program forwarding someone else's spec and the 400 is all it sees.
#include "radio/radio_spec.h"

#include <charconv>
#include <cmath>
#include <cstdio>

#include "radio/json.h"

namespace skysim::radio {

namespace {

using json::Value;

// Accepted values of a number field. `lo_open` makes the lower bound exclusive.
//
// Every field is bounded, not merely finite: the model raises 10 to a tenth of a dBm figure,
// and a finite but absurd one (a tower at +3100 dBm) overflows to infinity downstream. The
// bounds sit far outside anything physical, so they only ever refuse a mistake.
struct Range {
    double lo = 0.0;
    double hi = 0.0;
    bool lo_open = false;
};

constexpr Range kLatitude{-90.0, 90.0};
constexpr Range kLongitude{-180.0, 180.0};
constexpr Range kPower{-200.0, 0.0};       // dBm: rsrp_1km_dbm, noise_floor_dbm
constexpr Range kHeight{-1000.0, 10000.0}; // m AMSL

class Reader {
  public:
    std::string error;

    bool fail(const std::string &path, const std::string &what) {
        error = path + ": " + what;
        return false;
    }

    // Absent and null read the same: "not given".
    static const Value *given(const Value &obj, const char *key) {
        const Value *v = obj.find(key);
        return v == nullptr || v->is_null() ? nullptr : v;
    }

    bool number(const Value &obj, const char *key, const std::string &path, Range range,
                std::optional<double> &out, bool required) {
        const std::string at = path + "." + key;
        const Value *v = given(obj, key);
        if (v == nullptr) {
            out.reset();
            return !required || fail(at, "required number is missing");
        }
        if (!v->is_number()) {
            return fail(at, "must be a number");
        }
        const double x = v->number;
        if (x < range.lo || x > range.hi || (range.lo_open && x == range.lo)) {
            char buf[96];
            std::snprintf(buf, sizeof(buf),
                          range.lo_open ? "must be > %g and <= %g" : "must be between %g and %g", range.lo,
                          range.hi);
            return fail(at, buf);
        }
        out = x;
        return true;
    }

    bool required_number(const Value &obj, const char *key, const std::string &path, Range range,
                         double &out) {
        std::optional<double> v;
        if (!number(obj, key, path, range, v, true)) {
            return false;
        }
        out = *v;
        return true;
    }

    bool integer(const Value &obj, const char *key, const std::string &path, int hi,
                 std::optional<int> &out) {
        std::optional<double> v;
        if (!number(obj, key, path, Range{0.0, static_cast<double>(hi)}, v, false)) {
            return false;
        }
        if (v.has_value() && std::floor(*v) != *v) {
            return fail(path + "." + key, "must be an integer");
        }
        out = v.has_value() ? std::optional<int>(static_cast<int>(*v)) : std::nullopt;
        return true;
    }

    bool string(const Value &obj, const char *key, const std::string &path, std::optional<std::string> &out,
                bool required, size_t max_length = kMaxNameLength) {
        const std::string at = path + "." + key;
        const Value *v = given(obj, key);
        if (v == nullptr) {
            out.reset();
            return !required || fail(at, "required string is missing");
        }
        if (!v->is_string()) {
            return fail(at, "must be a string");
        }
        if (v->text.empty() || v->text.size() > max_length) {
            return fail(at, "must be 1 to " + std::to_string(max_length) + " characters");
        }
        out = v->text;
        return true;
    }

    bool required_string(const Value &obj, const char *key, const std::string &path, std::string &out) {
        std::optional<std::string> v;
        if (!string(obj, key, path, v, true)) {
            return false;
        }
        out = std::move(*v);
        return true;
    }

    // An array member; absent or null gives nullptr (and fails when required).
    bool array(const Value &obj, const char *key, const std::string &path, const Value *&out, bool required) {
        out = given(obj, key);
        if (out == nullptr) {
            return !required || fail(path + "." + key, "required array is missing");
        }
        return out->is_array() || fail(path + "." + key, "must be an array");
    }

    bool seed(const Value &obj, const std::string &path, uint64_t &out) {
        const std::string at = path + ".seed";
        const Value *v = given(obj, "seed");
        if (v == nullptr) {
            return fail(at, "required number is missing");
        }
        // Read from the literal, not the double: a 64-bit seed does not survive a double.
        // Rejects a sign, a fraction or exponent (the parse stops short of the end) and
        // anything past 2^64-1 (out of range).
        const char *first = v->text.data();
        const char *last = first + v->text.size();
        const auto parsed = std::from_chars(first, last, out);
        if (!v->is_number() || parsed.ec != std::errc() || parsed.ptr != last) {
            return fail(at, "must be an integer from 0 to 18446744073709551615");
        }
        return true;
    }
};

bool read_tower(Reader &r, const Value &v, const std::string &path, Tower &t) {
    if (!v.is_object()) {
        return r.fail(path, "must be an object");
    }
    if (!r.required_string(v, "id", path, t.id) || !r.required_number(v, "lat", path, kLatitude, t.lat_deg) ||
        !r.required_number(v, "lon", path, kLongitude, t.lon_deg) ||
        !r.required_number(v, "height_m", path, kHeight, t.height_m) ||
        !r.required_number(v, "rsrp_1km_dbm", path, kPower, t.rsrp_1km_dbm) ||
        !r.number(v, "azimuth_deg", path, Range{-360.0, 360.0}, t.azimuth_deg, false) ||
        !r.number(v, "beamwidth_deg", path, Range{0.0, 360.0, true}, t.beamwidth_deg, false) ||
        !r.required_number(v, "downtilt_deg", path, Range{-90.0, 90.0}, t.downtilt_deg) ||
        !r.integer(v, "pci", path, 1007, t.pci)) {
        return false;
    }
    // A sector's pattern divides by its beamwidth; an omni never reads it.
    if (t.azimuth_deg.has_value() && !t.beamwidth_deg.has_value()) {
        return r.fail(path + ".beamwidth_deg", "required for a sector (azimuth_deg set)");
    }
    return true;
}

bool read_carrier(Reader &r, const Value &v, const std::string &path, size_t &towers_total, Carrier &c) {
    if (!v.is_object()) {
        return r.fail(path, "must be an object");
    }
    const Value *towers = nullptr;
    if (!r.required_string(v, "name", path, c.name) || !r.string(v, "mcc_mnc", path, c.mcc_mnc, false, 6) ||
        !r.string(v, "rat", path, c.rat, false) || !r.integer(v, "band", path, 1'000'000, c.band) ||
        !r.integer(v, "earfcn", path, 1'000'000'000, c.earfcn) || !r.array(v, "towers", path, towers, true)) {
        return false;
    }
    if (c.mcc_mnc.has_value() &&
        (c.mcc_mnc->size() < 5 || c.mcc_mnc->find_first_not_of("0123456789") != std::string::npos)) {
        return r.fail(path + ".mcc_mnc", "must be 5 or 6 digits");
    }
    if (c.rat.has_value() && *c.rat != "LTE" && *c.rat != "NR" && *c.rat != "WCDMA" && *c.rat != "GSM") {
        return r.fail(path + ".rat", "must be LTE, NR, WCDMA or GSM");
    }
    if (towers->items.empty()) {
        return r.fail(path + ".towers", "must hold at least one tower");
    }
    for (size_t i = 0; i < towers->items.size(); ++i) {
        const std::string at = path + ".towers[" + std::to_string(i) + "]";
        if (++towers_total > kMaxTowers) {
            return r.fail(at, "more than " + std::to_string(kMaxTowers) + " towers in total");
        }
        Tower t;
        if (!read_tower(r, towers->items[i], at, t)) {
            return false;
        }
        for (const Tower &other : c.towers) {
            if (other.id == t.id) {
                return r.fail(at + ".id", "duplicate tower id \"" + t.id + "\" in this carrier");
            }
        }
        c.towers.push_back(std::move(t));
    }
    return true;
}

bool read_hole(Reader &r, const Value &v, const std::string &path, Hole &h) {
    if (!v.is_object()) {
        return r.fail(path, "must be an object");
    }
    return r.required_number(v, "lat", path, kLatitude, h.lat_deg) &&
           r.required_number(v, "lon", path, kLongitude, h.lon_deg) &&
           r.required_number(v, "radius_m", path, Range{0.0, 100000.0, true}, h.radius_m) &&
           r.required_number(v, "extra_loss_db", path, Range{0.0, 300.0}, h.extra_loss_db);
}

bool find_carrier(const RadioSpec &spec, const std::string &name) {
    for (const Carrier &c : spec.carriers) {
        if (c.name == name) {
            return true;
        }
    }
    return false;
}

bool read_modems(Reader &r, const Value &v, const std::string &path, RadioSpec &spec) {
    const Value *modems = nullptr;
    if (!r.array(v, "modems", path, modems, false)) {
        return false;
    }
    if (modems == nullptr) {
        for (const Carrier &c : spec.carriers) {
            spec.modems.push_back(c.name);
        }
        return true;
    }
    if (modems->items.empty() || modems->items.size() > kMaxCarriers) {
        return r.fail(path + ".modems", "must name 1 to " + std::to_string(kMaxCarriers) + " carriers");
    }
    for (size_t i = 0; i < modems->items.size(); ++i) {
        const std::string at = path + ".modems[" + std::to_string(i) + "]";
        const Value &m = modems->items[i];
        if (!m.is_string()) {
            return r.fail(at, "must be a carrier name");
        }
        if (!find_carrier(spec, m.text)) {
            return r.fail(at, "\"" + m.text + "\" is not a carrier in this spec");
        }
        for (const std::string &seen : spec.modems) {
            if (seen == m.text) {
                return r.fail(at, "\"" + m.text + "\" is listed twice");
            }
        }
        spec.modems.push_back(m.text);
    }
    return true;
}

bool read_spec(Reader &r, const Value &v, const std::string &path, RadioSpec &spec) {
    const Value *carriers = nullptr;
    const Value *holes = nullptr;
    if (!r.seed(v, path, spec.seed) ||
        !r.required_number(v, "shadowing_sigma_db", path, Range{0.0, 30.0}, spec.shadowing_sigma_db) ||
        // Below a metre the correlation grid stops meaning anything for a survey on ~50 m
        // cells, and it keeps the node indices far inside int64.
        !r.required_number(v, "shadow_corr_m", path, Range{1.0, 100000.0}, spec.shadow_corr_m) ||
        !r.required_number(v, "noise_floor_dbm", path, kPower, spec.noise_floor_dbm) ||
        !r.array(v, "carriers", path, carriers, true) || !r.array(v, "holes", path, holes, false)) {
        return false;
    }
    if (carriers->items.empty() || carriers->items.size() > kMaxCarriers) {
        return r.fail(path + ".carriers", "must hold 1 to " + std::to_string(kMaxCarriers) + " carriers");
    }
    size_t towers_total = 0;
    for (size_t i = 0; i < carriers->items.size(); ++i) {
        const std::string at = path + ".carriers[" + std::to_string(i) + "]";
        Carrier c;
        if (!read_carrier(r, carriers->items[i], at, towers_total, c)) {
            return false;
        }
        if (find_carrier(spec, c.name)) {
            return r.fail(at + ".name", "duplicate carrier name \"" + c.name + "\"");
        }
        spec.carriers.push_back(std::move(c));
    }
    if (holes != nullptr) {
        if (holes->items.size() > kMaxHoles) {
            return r.fail(path + ".holes", "must hold at most " + std::to_string(kMaxHoles) + " holes");
        }
        for (size_t i = 0; i < holes->items.size(); ++i) {
            Hole h;
            if (!read_hole(r, holes->items[i], path + ".holes[" + std::to_string(i) + "]", h)) {
                return false;
            }
            spec.holes.push_back(h);
        }
    }
    return read_modems(r, v, path, spec);
}

void append_key(std::string &out, const char *key) {
    json::append_string(out, key);
    out += ':';
}

void append_optional_number(std::string &out, const std::optional<double> &v) {
    if (v.has_value()) {
        json::append_number(out, *v);
    } else {
        out += "null";
    }
}

void append_optional_int(std::string &out, const std::optional<int> &v) {
    out += v.has_value() ? std::to_string(*v) : std::string("null");
}

void append_optional_string(std::string &out, const std::optional<std::string> &v) {
    if (v.has_value()) {
        json::append_string(out, *v);
    } else {
        out += "null";
    }
}

} // namespace

SpecResult parse_radio_spec(std::string_view body) {
    SpecResult result;
    if (body.size() > kMaxSpecBytes) {
        result.error = "radio spec larger than " + std::to_string(kMaxSpecBytes) + " bytes";
        return result;
    }
    const auto doc = json::parse(body, result.error);
    if (!doc.has_value()) {
        return result;
    }
    if (!doc->is_object()) {
        result.error = "radio: must be a JSON object";
        return result;
    }
    // {"radio": {...}} or the bare spec. A wrapped null/non-object is an error rather than
    // a fall-through to "bare", which would report a confusing "carriers missing".
    const Value *spec_value = doc->find("radio");
    if (spec_value == nullptr) {
        spec_value = &*doc;
    } else if (!spec_value->is_object()) {
        result.error = "radio: must be a JSON object";
        return result;
    }
    Reader reader;
    RadioSpec spec;
    if (!read_spec(reader, *spec_value, "radio", spec)) {
        result.error = reader.error;
        return result;
    }
    result.spec = std::move(spec);
    return result;
}

std::string radio_spec_json(const RadioSpec &spec) {
    std::string out = "{";
    append_key(out, "seed");
    out += std::to_string(spec.seed);
    out += ',';
    append_key(out, "shadowing_sigma_db");
    json::append_number(out, spec.shadowing_sigma_db);
    out += ',';
    append_key(out, "shadow_corr_m");
    json::append_number(out, spec.shadow_corr_m);
    out += ',';
    append_key(out, "noise_floor_dbm");
    json::append_number(out, spec.noise_floor_dbm);
    out += ',';
    append_key(out, "modems");
    out += '[';
    for (size_t i = 0; i < spec.modems.size(); ++i) {
        out += i ? "," : "";
        json::append_string(out, spec.modems[i]);
    }
    out += "],";
    append_key(out, "carriers");
    out += '[';
    for (size_t i = 0; i < spec.carriers.size(); ++i) {
        const Carrier &c = spec.carriers[i];
        out += i ? ",{" : "{";
        append_key(out, "name");
        json::append_string(out, c.name);
        out += ',';
        append_key(out, "mcc_mnc");
        append_optional_string(out, c.mcc_mnc);
        out += ',';
        append_key(out, "rat");
        append_optional_string(out, c.rat);
        out += ',';
        append_key(out, "band");
        append_optional_int(out, c.band);
        out += ',';
        append_key(out, "earfcn");
        append_optional_int(out, c.earfcn);
        out += ',';
        append_key(out, "towers");
        out += '[';
        for (size_t k = 0; k < c.towers.size(); ++k) {
            const Tower &t = c.towers[k];
            out += k ? ",{" : "{";
            append_key(out, "id");
            json::append_string(out, t.id);
            out += ',';
            append_key(out, "lat");
            json::append_number(out, t.lat_deg);
            out += ',';
            append_key(out, "lon");
            json::append_number(out, t.lon_deg);
            out += ',';
            append_key(out, "height_m");
            json::append_number(out, t.height_m);
            out += ',';
            append_key(out, "rsrp_1km_dbm");
            json::append_number(out, t.rsrp_1km_dbm);
            out += ',';
            append_key(out, "azimuth_deg");
            append_optional_number(out, t.azimuth_deg);
            out += ',';
            append_key(out, "beamwidth_deg");
            append_optional_number(out, t.beamwidth_deg);
            out += ',';
            append_key(out, "downtilt_deg");
            json::append_number(out, t.downtilt_deg);
            out += ',';
            append_key(out, "pci");
            append_optional_int(out, t.pci);
            out += '}';
        }
        out += "]}";
    }
    out += "],";
    append_key(out, "holes");
    out += '[';
    for (size_t i = 0; i < spec.holes.size(); ++i) {
        const Hole &h = spec.holes[i];
        out += i ? ",{" : "{";
        append_key(out, "lat");
        json::append_number(out, h.lat_deg);
        out += ',';
        append_key(out, "lon");
        json::append_number(out, h.lon_deg);
        out += ',';
        append_key(out, "radius_m");
        json::append_number(out, h.radius_m);
        out += ',';
        append_key(out, "extra_loss_db");
        json::append_number(out, h.extra_loss_db);
        out += '}';
    }
    out += "]}";
    return out;
}

} // namespace skysim::radio

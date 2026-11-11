// Simulated radio field: the JSON reader, spec validation, every formula term on its own,
// determinism, the JSON views, the shared field holder, and the contract's cross-check vector
// (tests/fixtures/radio_test_vector.json).
//
//   test_radio                       run the checks
//   test_radio --write-vector PATH   regenerate the vector from tests/fixtures/radio_test_spec.json
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "radio/json.h"
#include "radio/radio_model.h"
#include "radio/radio_spec.h"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                                                          \
    do {                                                                                                     \
        if (!(cond)) {                                                                                       \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                      \
            ++g_failures;                                                                                    \
        }                                                                                                    \
    } while (0)

#define NEAR(a, b, tol)                                                                                      \
    do {                                                                                                     \
        const double a_ = (a);                                                                               \
        const double b_ = (b);                                                                               \
        if (!(std::fabs(a_ - b_) <= (tol))) {                                                                \
            std::printf("FAIL %s:%d: %s = %.12g, want %.12g (±%g)\n", __FILE__, __LINE__, #a, a_, b_,        \
                        static_cast<double>(tol));                                                           \
            ++g_failures;                                                                                    \
        }                                                                                                    \
    } while (0)

using namespace skysim::radio;

// shadow_node() values from an independent implementation of the contract's recipe (Python,
// arbitrary-precision integers masked to 64 bits).
constexpr double kNode7_0_0_0 = 1.3649922974572284;
constexpr double kNode7_m1_2_1 = -0.7075445477561018;
constexpr double kNode7_3_m4_0 = -0.08895334509673265;
constexpr double kNodeBig = -2.127114764068878;

const std::string kFixtures = std::string(SKYSIM_SOURCE_DIR) + "/tests/fixtures/";

std::string read_file(const std::string &path) {
    std::ifstream in(path, std::ios::binary);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// The contract's reference positions (lat, lon, alt_rel); ground is at 0 m AMSL.
struct VectorPosition {
    double lat;
    double lon;
    double alt_rel;
};
constexpr VectorPosition kVectorPositions[] = {
    {47.0000, 8.0020, 2},  {47.0000, 8.0020, 60}, {47.0040, 8.0060, 30}, {47.0040, 8.0060, 100},
    {47.0060, 8.0080, 30}, {47.0100, 8.0100, 50}, {46.9980, 8.0120, 20}, {46.9980, 8.0120, 120},
    {47.0020, 7.9950, 40}, {47.0150, 8.0200, 80}, {47.0035, 8.0055, 10}, {46.9900, 7.9900, 60},
};

void append_value(std::string &out, double v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.2f", v);
    // -0.00 and 0.00 are one value; print it one way.
    out += std::strcmp(buf, "-0.00") == 0 ? "0.00" : buf;
}

// The vector as the contract lists it: {lat, lon, alt_rel, carrier, rsrp, sinr, rsrq,
// registered}, values rounded to 0.01, signal values null when not registered. rsrp here is
// the serving tower's unrounded figure (the API's is rounded to 0.1 dB).
std::string vector_json(const RadioModel &model) {
    std::string out = "[\n";
    bool first = true;
    for (const auto &vp : kVectorPositions) {
        const Position p{vp.lat, vp.lon, vp.alt_rel, vp.alt_rel};
        for (size_t c = 0; c < model.spec().carriers.size(); ++c) {
            const CarrierSample s = model.sample(c, p);
            char head[160];
            std::snprintf(head, sizeof(head),
                          "%s  {\"lat\": %.4f, \"lon\": %.4f, \"alt_rel\": %.1f, \"carrier\": ",
                          first ? "" : ",\n", vp.lat, vp.lon, vp.alt_rel);
            out += head;
            json::append_string(out, model.spec().carriers[c].name);
            const std::pair<const char *, double> values[] = {
                {"rsrp", s.rsrp_exact_dbm}, {"sinr", s.sinr_db}, {"rsrq", s.rsrq_db}};
            for (const auto &[key, v] : values) {
                out += ", \"";
                out += key;
                out += "\": ";
                if (s.registered) {
                    append_value(out, v);
                } else {
                    out += "null";
                }
            }
            out += s.registered ? ", \"registered\": true}" : ", \"registered\": false}";
            first = false;
        }
    }
    out += "\n]\n";
    return out;
}

RadioModel load_vector_model() {
    const SpecResult r = parse_radio_spec(read_file(kFixtures + "radio_test_spec.json"));
    if (!r.spec.has_value()) {
        std::printf("FAIL: radio_test_spec.json: %s\n", r.error.c_str());
        ++g_failures;
        return RadioModel(RadioSpec{});
    }
    return RadioModel(*r.spec);
}

// A one-carrier spec with no shadowing and no holes, so a tower's RSRP is the path loss and
// the patterns alone and can be checked by hand.
RadioSpec plain_spec() {
    RadioSpec spec;
    spec.seed = 1;
    spec.shadowing_sigma_db = 0.0;
    spec.shadow_corr_m = 60.0;
    spec.noise_floor_dbm = -104.0;
    Carrier c;
    c.name = "SimTel A";
    c.mcc_mnc = "00101";
    c.rat = "LTE";
    c.band = 20;
    c.earfcn = 6300;
    Tower t;
    t.id = "A1";
    t.lat_deg = 47.0;
    t.lon_deg = 8.0;
    t.height_m = 30.0;
    t.rsrp_1km_dbm = -75.0;
    t.downtilt_deg = 0.0;
    t.pci = 101;
    c.towers.push_back(t);
    spec.carriers.push_back(c);
    spec.modems = {"SimTel A"};
    return spec;
}

// Metres north/east of (47, 8) as a Position at `alt` (ground at 0 m AMSL).
Position offset(const RadioModel &m, double north_m, double east_m, double alt) {
    (void)m;
    const GeoAnchor anchor{47.0, 8.0, 0.0};
    const double ned[3] = {north_m, east_m, -alt};
    return position_from_ned(anchor, ned);
}

void test_json_reader() {
    std::string err;
    auto doc = json::parse(
        R"( {"a": [1, -2.5e3, true, false, null], "s": "q\"\\\/\b\f\n\r\t\u00e9\ud83d\ude00", "o": {}} )",
        err);
    CHECK(doc.has_value());
    if (doc) {
        const json::Value *a = doc->find("a");
        CHECK(a != nullptr && a->is_array() && a->items.size() == 5);
        if (a != nullptr && a->items.size() == 5) {
            CHECK(a->items[0].number == 1.0 && a->items[0].text == "1");
            CHECK(a->items[1].number == -2500.0);
            CHECK(a->items[2].boolean && !a->items[3].boolean && a->items[4].is_null());
        }
        const json::Value *s = doc->find("s");
        CHECK(s != nullptr && s->text == "q\"\\/\b\f\n\r\t\xC3\xA9\xF0\x9F\x98\x80");
        CHECK(doc->find("o") != nullptr && doc->find("o")->is_object());
        CHECK(doc->find("missing") == nullptr);
        CHECK(a != nullptr && a->find("x") == nullptr); // find on a non-object
    }
    CHECK(json::parse("[]", err).has_value());
    CHECK(json::parse("\"\\u0041\\u07FF\\uFFFF\"", err).has_value());

    // Each malformed document is refused with an offset, never accepted or crashed on.
    const char *bad[] = {"",
                         "{",
                         "{\"a\" 1}",
                         "{\"a\":1,}",
                         "{a:1}",
                         "[1 2]",
                         "[1,",
                         "tru",
                         "nul",
                         "\"abc",
                         "\"a\x01\"",
                         "\"\\x\"",
                         "\"\\u12\"",
                         "\"\\uD800\"",
                         "\"\\uD800\\u0041\"",
                         "\"\\uDC00\"",
                         "01",
                         "-",
                         "1.",
                         "1e",
                         "+1",
                         "1e999",
                         "1e-999",
                         "{\"a\":1,\"a\":2}",
                         "[1] x",
                         ".5",
                         "{\"a\": tru}",
                         "{\"ab",
                         "{\"a\":1 \"b\":2}",
                         "\"\\u1\"",
                         "\"\\u12G4\""};
    for (const char *text : bad) {
        err.clear();
        const bool parsed = json::parse(text, err).has_value();
        CHECK(!parsed);
        CHECK(err.find("invalid JSON at offset") == 0);
        if (parsed) {
            std::printf("  accepted: %s\n", text);
        }
    }
    std::string deep(json::kMaxDepth + 2, '[');
    deep += std::string(json::kMaxDepth + 2, ']');
    CHECK(!json::parse(deep, err).has_value());
    CHECK(err.find("nesting too deep") != std::string::npos);

    std::string out;
    json::append_string(out, std::string("a\"b\\c\nd\re\tf\x01g", 13));
    CHECK(out == "\"a\\\"b\\\\c\\nd\\re\\tf\\u0001g\"");
    out.clear();
    json::append_number(out, 47.0);
    out += ',';
    json::append_number(out, 8.015);
    out += ',';
    json::append_number(out, -104.0);
    CHECK(out == "47,8.015,-104");
    out.clear();
    json::append_fixed(out, std::nullopt, 1);
    out += ',';
    json::append_fixed(out, -60.25, 1);
    CHECK(out == "null,-60.2" || out == "null,-60.3");
}

void test_terms() {
    // Step 2: 3.5 at or below home, 0.01 less per metre, floored at free space.
    NEAR(path_loss_exponent(0.0), 3.5, 1e-12);
    NEAR(path_loss_exponent(-20.0), 3.5, 1e-12);
    NEAR(path_loss_exponent(50.0), 3.0, 1e-12);
    NEAR(path_loss_exponent(150.0), 2.0, 1e-12);
    NEAR(path_loss_exponent(400.0), 2.0, 1e-12);

    // Smallest angle, either way round and past a full turn.
    NEAR(angle_between_deg(350.0, 10.0), 20.0, 1e-9);
    NEAR(angle_between_deg(10.0, 350.0), 20.0, 1e-9);
    NEAR(angle_between_deg(0.0, 180.0), 180.0, 1e-9);
    NEAR(angle_between_deg(725.0, 0.0), 5.0, 1e-9);
    NEAR(angle_between_deg(-90.0, 225.0), 45.0, 1e-9);

    // Step 3: omni is flat; a sector loses 12 dB at one beamwidth off axis, capped at 20.
    NEAR(horizontal_pattern_db(std::nullopt, 65.0, 123.0), 0.0, 0.0);
    NEAR(horizontal_pattern_db(90.0, 65.0, 90.0), 0.0, 1e-12);
    NEAR(horizontal_pattern_db(90.0, 65.0, 155.0), 12.0, 1e-9);
    NEAR(horizontal_pattern_db(90.0, 60.0, 120.0), 3.0, 1e-9);
    NEAR(horizontal_pattern_db(0.0, 65.0, 180.0), 20.0, 1e-12);

    // Step 4: zero on the tilted beam, 12 dB ten degrees off it, capped at 20.
    NEAR(vertical_pattern_db(-6.0, 6.0), 0.0, 1e-12);
    NEAR(vertical_pattern_db(0.0, 6.0), 4.32, 1e-9);
    NEAR(vertical_pattern_db(4.0, 6.0), 12.0, 1e-9);
    NEAR(vertical_pattern_db(90.0, 6.0), 20.0, 1e-12);
    NEAR(vertical_pattern_db(-90.0, 6.0), 20.0, 1e-12);

    // Unit conversions.
    NEAR(dbm_to_mw(0.0), 1.0, 1e-12);
    NEAR(dbm_to_mw(-30.0), 1e-3, 1e-15);
    NEAR(mw_to_dbm(100.0), 20.0, 1e-12);

    CHECK(std::string(to_string(LinkQuality::kGood)) == "good");
    CHECK(std::string(to_string(LinkQuality::kWeak)) == "weak");
    CHECK(std::string(to_string(LinkQuality::kBad)) == "bad");
    CHECK(std::string(to_string(LinkQuality::kNone)) == "none");
}

void test_prng() {
    // splitmix64's published reference outputs for a zero seed.
    uint64_t state = 0;
    CHECK(splitmix64_next(state) == 0xE220A8397B1DCDAFULL);
    CHECK(splitmix64_next(state) == 0x6E789E6AA1B965F4ULL);
    CHECK(splitmix64_next(state) == 0x06C45D188009454FULL);

    // Node values: computed independently (Python, integers masked to 64 bits) from the
    // contract's recipe, so they pin the hash, the wrap of negative indices, and Box–Muller.
    NEAR(shadow_node(7, 0, 0, 0), kNode7_0_0_0, 1e-12);
    NEAR(shadow_node(7, -1, 2, 1), kNode7_m1_2_1, 1e-12);
    NEAR(shadow_node(7, 3, -4, 0), kNode7_3_m4_0, 1e-12);
    NEAR(shadow_node(12345678901234567890ULL, -1000, -1000, 7), kNodeBig, 1e-12);

    // Deterministic, and the inputs all matter.
    CHECK(shadow_node(7, 5, 6, 0) == shadow_node(7, 5, 6, 0));
    CHECK(shadow_node(7, 5, 6, 0) != shadow_node(8, 5, 6, 0));
    CHECK(shadow_node(7, 5, 6, 0) != shadow_node(7, 6, 5, 0));
    CHECK(shadow_node(7, 5, 6, 0) != shadow_node(7, 5, 6, 1));

    // Standard normal over many nodes.
    double sum = 0.0;
    double sum2 = 0.0;
    const int n = 200 * 200;
    for (int ix = -100; ix < 100; ++ix) {
        for (int iy = -100; iy < 100; ++iy) {
            const double z = shadow_node(7, ix, iy, 0);
            sum += z;
            sum2 += z * z;
        }
    }
    const double mean = sum / n;
    NEAR(mean, 0.0, 0.03);
    NEAR(std::sqrt(sum2 / n - mean * mean), 1.0, 0.03);
}

void test_frame_and_fields() {
    RadioSpec spec = plain_spec();
    spec.shadowing_sigma_db = 6.0;
    spec.seed = 7;
    spec.holes.push_back(Hole{47.0, 8.01, 150.0, 35.0});
    spec.holes.push_back(Hole{47.0, 8.011, 150.0, 10.0});
    const RadioModel m(spec);

    // Equirectangular about the first tower: 0.001° of latitude is R·0.001·π/180 north.
    double x = 0.0;
    double y = 0.0;
    m.to_local(47.001, 8.0, x, y);
    NEAR(x, 0.0, 1e-9);
    NEAR(y, 111.19492664455873, 1e-6);
    m.to_local(47.0, 8.001, x, y);
    NEAR(x, 111.19492664455873 * std::cos(47.0 * M_PI / 180.0), 1e-6);
    NEAR(y, 0.0, 1e-9);

    // Shadowing: exactly the node value on a node, the mean of four at a cell centre.
    const double corr = spec.shadow_corr_m;
    NEAR(m.shadowing_db(0, 2 * corr, -3 * corr), 6.0 * shadow_node(7, 2, -3, 0), 1e-9);
    const double centre = m.shadowing_db(0, 2.5 * corr, -2.5 * corr);
    NEAR(centre,
         6.0 * 0.25 *
             (shadow_node(7, 2, -3, 0) + shadow_node(7, 3, -3, 0) + shadow_node(7, 2, -2, 0) +
              shadow_node(7, 3, -2, 0)),
         1e-9);
    // Bilinear: continuous across a node line.
    NEAR(m.shadowing_db(0, 3 * corr - 1e-7, 10.0), m.shadowing_db(0, 3 * corr + 1e-7, 10.0), 1e-6);
    // Each carrier has its own field.
    CHECK(m.shadowing_db(0, 10.0, 10.0) != m.shadowing_db(1, 10.0, 10.0));

    // Holes: full loss at the centre, sqrt falloff, nothing outside, overlapping holes add.
    double hx = 0.0;
    double hy = 0.0;
    m.to_local(47.0, 8.01, hx, hy);
    double hx2 = 0.0;
    double hy2 = 0.0;
    m.to_local(47.0, 8.011, hx2, hy2);
    const double gap = hx2 - hx; // ~76 m: inside both holes
    NEAR(m.hole_loss_db(hx, hy), 35.0 + 10.0 * std::sqrt(1.0 - gap / 150.0), 1e-9);
    NEAR(m.hole_loss_db(hx - 75.0, hy),
         35.0 * std::sqrt(0.5) + 10.0 * std::sqrt(std::max(0.0, 1.0 - (75.0 + gap) / 150.0)), 1e-9);
    NEAR(m.hole_loss_db(hx, hy + 400.0), 0.0, 0.0);

    // Δlon is taken the short way round the antimeridian.
    RadioSpec dateline = plain_spec();
    dateline.carriers[0].towers[0].lon_deg = 179.9;
    const RadioModel east(dateline);
    east.to_local(47.0, -179.9, x, y);
    CHECK(x > 0.0 && x < 20000.0);
    dateline.carriers[0].towers[0].lon_deg = -179.9;
    const RadioModel west(dateline);
    west.to_local(47.0, 179.9, x, y);
    CHECK(x < 0.0 && x > -20000.0);

    // NED → geodetic about an anchor; alt_amsl adds the anchor's ground height.
    const GeoAnchor anchor{47.0, 8.0, 400.0};
    const double ned[3] = {111.19492664455873, 0.0, -30.0};
    const Position p = position_from_ned(anchor, ned);
    NEAR(p.lat_deg, 47.001, 1e-9);
    NEAR(p.lon_deg, 8.0, 1e-12);
    NEAR(p.alt_rel_m, 30.0, 1e-12);
    NEAR(p.alt_amsl_m, 430.0, 1e-12);
    const double east_ned[3] = {0.0, 111.19492664455873 * std::cos(47.0 * M_PI / 180.0), 0.0};
    NEAR(position_from_ned(anchor, east_ned).lon_deg, 8.001, 1e-9);
}

void test_tower_rsrp() {
    const RadioModel m(plain_spec());
    // Level with the antenna, 1 km out: d_km = 1 so path loss is zero, θ = 0 with no tilt.
    NEAR(m.tower_rsrp_dbm(0, 0, offset(m, 1000.0, 0.0, 30.0)), -75.0, 1e-6);
    // Twice as far: 10·n·log10(2) with n = 3.5 - 0.3.
    NEAR(m.tower_rsrp_dbm(0, 0, offset(m, 2000.0, 0.0, 30.0)), -75.0 - 32.0 * std::log10(2.0), 1e-6);
    // Same point, other way: the omni pattern does not care about bearing.
    NEAR(m.tower_rsrp_dbm(0, 0, offset(m, 0.0, -2000.0, 30.0)), -75.0 - 32.0 * std::log10(2.0), 1e-6);
    // On top of the antenna the distance floors at 10 m (and θ = 90° caps A_v at 20).
    NEAR(m.tower_rsrp_dbm(0, 0, offset(m, 0.0, 0.0, 30.0)), -75.0 - 32.0 * std::log10(0.01) - 0.0, 1e-6);
    NEAR(m.tower_rsrp_dbm(0, 0, offset(m, 0.0, 0.0, 35.0)), -75.0 - 10.0 * 3.15 * std::log10(0.01) - 20.0,
         1e-6);

    // Elevation: 1 km out and 1 km up is θ = 45° → A_v capped at 20, d = √2 km, n at 1030 m = 2.
    const double up = m.tower_rsrp_dbm(0, 0, offset(m, 1000.0, 0.0, 1030.0));
    NEAR(up, -75.0 - 20.0 * std::log10(std::sqrt(2.0)) - 20.0, 1e-6);

    // A sector pointing east: full gain east, 20 dB down north (90° off a 65° beam).
    RadioSpec sector = plain_spec();
    sector.carriers[0].towers[0].azimuth_deg = 90.0;
    sector.carriers[0].towers[0].beamwidth_deg = 65.0;
    const RadioModel s(sector);
    NEAR(s.tower_rsrp_dbm(0, 0, offset(s, 0.0, 1000.0, 30.0)), -75.0, 1e-6);
    NEAR(s.tower_rsrp_dbm(0, 0, offset(s, 1000.0, 0.0, 30.0)), -95.0, 1e-6);
    // 30° off axis to the south-east of the bearing: 12·(30/65)².
    const double b = 120.0 * M_PI / 180.0;
    NEAR(s.tower_rsrp_dbm(0, 0, offset(s, 1000.0 * std::cos(b), 1000.0 * std::sin(b), 30.0)),
         -75.0 - 12.0 * (30.0 / 65.0) * (30.0 / 65.0), 1e-4);
}

void test_sample() {
    // One tower: no interference, so SINR is RSRP over the noise floor.
    RadioModel one(plain_spec());
    const Position p1 = offset(one, 2000.0, 0.0, 30.0);
    const CarrierSample s1 = one.sample(0, p1);
    const double rsrp1 = -75.0 - 32.0 * std::log10(2.0);
    NEAR(s1.rsrp_exact_dbm, rsrp1, 1e-6);
    NEAR(s1.rsrp_dbm, std::round(rsrp1 * 10.0) / 10.0, 1e-9);
    NEAR(s1.sinr_db, std::min(40.0, rsrp1 + 104.0), 1e-6);
    const double sinr_lin = std::pow(10.0, s1.sinr_db / 10.0);
    NEAR(s1.rsrq_db, std::max(-19.5, -3.0 - 10.0 * std::log10(1.0 + 1.0 / sinr_lin) - 5.0), 1e-9);
    NEAR(s1.rssi_dbm, 10.0 * std::log10(dbm_to_mw(rsrp1) + dbm_to_mw(-104.0)) + 10.0 * std::log10(600.0),
         1e-6);
    CHECK(s1.registered);
    CHECK(classify(s1) == LinkQuality::kGood);

    // Two towers: the stronger serves, the other is interference.
    RadioSpec two = plain_spec();
    Tower t2 = two.carriers[0].towers[0];
    t2.id = "A2";
    t2.pci = 102;
    t2.lat_deg = 47.02; // ~2.2 km north
    two.carriers[0].towers.push_back(t2);
    const RadioModel m2(two);
    const Position near_a2 = offset(m2, 2000.0, 0.0, 30.0);
    const CarrierSample s2 = m2.sample(0, near_a2);
    CHECK(s2.serving_tower == 1);
    const double r_a1 = m2.tower_rsrp_dbm(0, 0, near_a2);
    const double r_a2 = m2.tower_rsrp_dbm(0, 1, near_a2);
    CHECK(r_a2 > r_a1);
    NEAR(s2.rsrp_exact_dbm, r_a2, 1e-9);
    NEAR(s2.sinr_db, 10.0 * std::log10(dbm_to_mw(r_a2) / (dbm_to_mw(r_a1) + dbm_to_mw(-104.0))), 1e-9);
    NEAR(s2.rssi_dbm,
         10.0 * std::log10(dbm_to_mw(r_a1) + dbm_to_mw(r_a2) + dbm_to_mw(-104.0)) + 10.0 * std::log10(600.0),
         1e-9);
    // Halfway between two equal towers SINR is ~0 dB: bad, but registered.
    const CarrierSample mid = m2.sample(0, offset(m2, 1112.0, 0.0, 30.0));
    NEAR(mid.sinr_db, 0.0, 0.2);
    // A tie goes to the first tower.
    RadioSpec twin = plain_spec();
    twin.carriers[0].towers.push_back(twin.carriers[0].towers[0]);
    twin.carriers[0].towers[1].id = "A1b";
    const RadioModel mt(twin);
    CHECK(mt.sample(0, offset(mt, 500.0, 0.0, 30.0)).serving_tower == 0);

    // Co-sited copies of one tower, 300 m out where noise is ~45 dB down: SINR is 1/(k-1).
    NEAR(one.sample(0, offset(one, 50.0, 0.0, 30.0)).sinr_db, 40.0, 0.0); // clamp at the top
    auto cosited = [](size_t count) {
        RadioSpec spec = plain_spec();
        while (spec.carriers[0].towers.size() < count) {
            spec.carriers[0].towers.push_back(spec.carriers[0].towers[0]);
            spec.carriers[0].towers.back().id = "X" + std::to_string(spec.carriers[0].towers.size());
        }
        return RadioModel(spec);
    };
    const Position at300 = offset(one, 300.0, 0.0, 30.0);
    // 4 towers: -4.8 dB — registered, bad.
    const CarrierSample s4 = cosited(4).sample(0, at300);
    NEAR(s4.sinr_db, 10.0 * std::log10(1.0 / 3.0), 0.01);
    NEAR(s4.rsrq_db, -3.0 - 10.0 * std::log10(1.0 + 1.0 / dbm_to_mw(s4.sinr_db)) - 5.0, 1e-9);
    CHECK(s4.registered);
    CHECK(classify(s4) == LinkQuality::kBad);
    // 7 towers: -7.8 dB is still at or above the -8 dB registration floor.
    const CarrierSample s7 = cosited(7).sample(0, at300);
    NEAR(s7.sinr_db, 10.0 * std::log10(1.0 / 6.0), 0.01);
    CHECK(s7.registered);
    // 20 towers: -12.8 dB — no service, and RSRQ (-21 unclamped) bottoms out at -19.5.
    const CarrierSample s20 = cosited(20).sample(0, at300);
    NEAR(s20.sinr_db, 10.0 * std::log10(1.0 / 19.0), 0.01);
    NEAR(s20.rsrq_db, -19.5, 0.0);
    CHECK(!s20.registered);
    // 120 towers (only a hand-built spec can): SINR bottoms out at -20.
    NEAR(cosited(120).sample(0, at300).sinr_db, -20.0, 0.0);

    // Far away: below -124 dBm the modem has no service.
    const CarrierSample far = one.sample(0, offset(one, 60000.0, 0.0, 30.0));
    CHECK(far.rsrp_dbm < -124.0);
    CHECK(!far.registered);
    CHECK(classify(far) == LinkQuality::kNone);

    // Quality thresholds, at and either side of each edge.
    CarrierSample q;
    q.registered = true;
    q.rsrp_dbm = -100.0;
    q.sinr_db = 5.0;
    CHECK(classify(q) == LinkQuality::kGood);
    q.rsrp_dbm = -100.1;
    CHECK(classify(q) == LinkQuality::kWeak);
    q.rsrp_dbm = -90.0;
    q.sinr_db = 4.9;
    CHECK(classify(q) == LinkQuality::kWeak);
    q.sinr_db = 0.0;
    CHECK(classify(q) == LinkQuality::kWeak);
    q.sinr_db = -0.1;
    CHECK(classify(q) == LinkQuality::kBad);
    q.sinr_db = 20.0;
    q.rsrp_dbm = -110.0;
    CHECK(classify(q) == LinkQuality::kWeak);
    q.rsrp_dbm = -110.1;
    CHECK(classify(q) == LinkQuality::kBad);
    q.registered = false;
    CHECK(classify(q) == LinkQuality::kNone);
}

void test_determinism() {
    const SpecResult r = parse_radio_spec(read_file(kFixtures + "radio_test_spec.json"));
    CHECK(r.spec.has_value());
    if (!r.spec) {
        return;
    }
    const RadioModel a(*r.spec);
    const RadioModel b(*r.spec);
    RadioSpec reseeded = *r.spec;
    reseeded.seed = 8;
    const RadioModel c(reseeded);
    for (const auto &vp : kVectorPositions) {
        const Position p{vp.lat, vp.lon, vp.alt_rel, vp.alt_rel};
        for (size_t k = 0; k < 2; ++k) {
            const CarrierSample sa = a.sample(k, p);
            const CarrierSample sb = b.sample(k, p);
            CHECK(sa.rsrp_exact_dbm == sb.rsrp_exact_dbm && sa.sinr_db == sb.sinr_db &&
                  sa.rsrq_db == sb.rsrq_db && sa.rssi_dbm == sb.rssi_dbm && sa.registered == sb.registered);
            CHECK(sa.rsrp_exact_dbm != c.sample(k, p).rsrp_exact_dbm);
        }
        CHECK(sample_json(a, p) == sample_json(b, p));
    }
}

void test_spec_parsing() {
    const std::string fixture = read_file(kFixtures + "radio_test_spec.json");
    const SpecResult wrapped = parse_radio_spec(fixture);
    CHECK(wrapped.spec.has_value());
    if (!wrapped.spec) {
        std::printf("  %s\n", wrapped.error.c_str());
        return;
    }
    const RadioSpec &spec = *wrapped.spec;
    CHECK(spec.seed == 7);
    CHECK(spec.carriers.size() == 2 && spec.carriers[0].towers.size() == 2 && spec.holes.size() == 1);
    CHECK(!spec.carriers[0].towers[0].azimuth_deg.has_value()); // null = omni
    CHECK(spec.carriers[0].towers[1].azimuth_deg == 225.0);
    CHECK(spec.modems.size() == 2 && spec.modems[0] == "SimTel A");

    // Canonical JSON round-trips, bare and inside a simulator-link object with other keys.
    const std::string canon = radio_spec_json(spec);
    const SpecResult bare = parse_radio_spec(canon);
    CHECK(bare.spec.has_value() && radio_spec_json(*bare.spec) == canon);
    const SpecResult link = parse_radio_spec(R"({"zones":[{"x":1}],"timeline":[],"radio":)" + canon + "}");
    CHECK(link.spec.has_value() && radio_spec_json(*link.spec) == canon);

    // Defaults: modems are every carrier, holes none, optional identity fields null.
    const SpecResult minimal = parse_radio_spec(
        R"({"seed":18446744073709551615,"shadowing_sigma_db":0,"shadow_corr_m":1,"noise_floor_dbm":-100,)"
        R"("carriers":[{"name":"N1","towers":[{"id":"t","lat":0,"lon":0,"height_m":10,"rsrp_1km_dbm":-80,)"
        R"("downtilt_deg":0}]},{"name":"N2","towers":[{"id":"t","lat":0,"lon":0,"height_m":10,)"
        R"("rsrp_1km_dbm":-80,"downtilt_deg":0,"azimuth_deg":10,"beamwidth_deg":30}]}]})");
    CHECK(minimal.spec.has_value());
    if (minimal.spec) {
        CHECK(minimal.spec->seed == 18446744073709551615ULL);
        CHECK(minimal.spec->modems == (std::vector<std::string>{"N1", "N2"}));
        CHECK(minimal.spec->holes.empty());
        CHECK(!minimal.spec->carriers[0].mcc_mnc && !minimal.spec->carriers[0].rat &&
              !minimal.spec->carriers[0].band);
        CHECK(!minimal.spec->carriers[0].towers[0].pci && !minimal.spec->carriers[0].towers[0].beamwidth_deg);
        const std::string mj = radio_spec_json(*minimal.spec);
        CHECK(mj.find("\"seed\":18446744073709551615") != std::string::npos);
        CHECK(mj.find("\"mcc_mnc\":null,\"rat\":null,\"band\":null,\"earfcn\":null") != std::string::npos);
        const SpecResult again = parse_radio_spec(mj);
        CHECK(again.spec.has_value() && radio_spec_json(*again.spec) == mj);
    }

    // Each bad spec: rejected, and the error names where.
    struct Bad {
        std::string body;
        const char *error;
    };
    auto with = [&canon](const std::string &from, const std::string &to) {
        std::string s = canon;
        const size_t at = s.find(from);
        if (at == std::string::npos) {
            std::printf("FAIL: test fixture lacks %s\n", from.c_str());
            ++g_failures;
            return s;
        }
        return s.replace(at, from.size(), to);
    };
    std::string many_carriers = canon;
    {
        const size_t start = canon.find("{\"name\":\"SimTel B\"");
        const size_t end = canon.find("]}]", start) + 2;
        const std::string b = canon.substr(start, end - start);
        std::string extra;
        for (int k = 0; k < 7; ++k) {
            std::string copy = b;
            copy.replace(copy.find("SimTel B"), 8, "SimTel X" + std::to_string(k));
            extra += "," + copy;
        }
        many_carriers.insert(end, extra);
    }
    std::string many_towers = canon;
    {
        const size_t start = canon.find("{\"id\":\"B1\"");
        const size_t end = canon.find('}', start) + 1;
        const std::string t = canon.substr(start, end - start);
        std::string extra;
        for (int k = 0; k < 62; ++k) {
            std::string copy = t;
            copy.replace(copy.find("B1"), 2, "T" + std::to_string(k));
            extra += "," + copy;
        }
        many_towers.insert(end, extra);
    }
    std::string many_holes = canon;
    {
        const size_t start = canon.find("\"holes\":[") + 9;
        const size_t end = canon.find('}', start) + 1;
        const std::string h = canon.substr(start, end - start);
        std::string extra;
        for (int k = 0; k < 64; ++k) {
            extra += "," + h;
        }
        many_holes.insert(end, extra);
    }
    const Bad bad[] = {
        {"not json", "invalid JSON at offset 0"},
        {"[1,2]", "radio: must be a JSON object"},
        {R"({"radio":null})", "radio: must be a JSON object"},
        {R"({"radio":[]})", "radio: must be a JSON object"},
        {with("\"seed\":7", "\"seed\":null"), "radio.seed: required number is missing"},
        {with("\"seed\":7", "\"seed\":-1"), "radio.seed: must be an integer"},
        {with("\"seed\":7", "\"seed\":7.5"), "radio.seed: must be an integer"},
        {with("\"seed\":7", "\"seed\":\"7\""), "radio.seed: must be an integer"},
        {with("\"seed\":7", "\"seed\":18446744073709551616"), "radio.seed: must be an integer"},
        {with("\"shadowing_sigma_db\":6", "\"shadowing_sigma_db\":-1"),
         "radio.shadowing_sigma_db: must be >= 0"},
        {with("\"shadowing_sigma_db\":6", "\"shadowing_sigma_db\":\"6\""),
         "radio.shadowing_sigma_db: must be a number"},
        {with("\"shadow_corr_m\":60", "\"shadow_corr_m\":0.5"), "radio.shadow_corr_m: must be >= 1"},
        {with("\"noise_floor_dbm\":-104", "\"nf\":-104"),
         "radio.noise_floor_dbm: required number is missing"},
        {with("\"carriers\":", "\"carriers_\":"), "radio.carriers: required array is missing"},
        {R"({"seed":1,"shadowing_sigma_db":1,"shadow_corr_m":60,"noise_floor_dbm":-104,"carriers":{}})",
         "radio.carriers: must be an array"},
        {R"({"seed":1,"shadowing_sigma_db":1,"shadow_corr_m":60,"noise_floor_dbm":-104,"carriers":[]})",
         "radio.carriers: must hold 1 to 8 carriers"},
        {many_carriers, "radio.carriers: must hold 1 to 8 carriers"},
        {R"({"seed":1,"shadowing_sigma_db":1,"shadow_corr_m":60,"noise_floor_dbm":-104,"carriers":[5]})",
         "radio.carriers[0]: must be an object"},
        {with("\"name\":\"SimTel A\"", "\"name\":\"\""),
         "radio.carriers[0].name: must be 1 to 64 characters"},
        {with("\"name\":\"SimTel A\"", "\"name\":\"" + std::string(65, 'x') + "\""),
         "radio.carriers[0].name: must be 1 to 64 characters"},
        {with("\"name\":\"SimTel A\"", "\"name\":7"), "radio.carriers[0].name: must be a string"},
        {with("\"name\":\"SimTel B\"", "\"name\":\"SimTel A\""),
         "radio.carriers[1].name: duplicate carrier name"},
        {with("\"mcc_mnc\":\"00101\"", "\"mcc_mnc\":\"0010a\""),
         "radio.carriers[0].mcc_mnc: must be 5 or 6 digits"},
        {with("\"mcc_mnc\":\"00101\"", "\"mcc_mnc\":\"0010\""),
         "radio.carriers[0].mcc_mnc: must be 5 or 6 digits"},
        {with("\"mcc_mnc\":\"00101\"", "\"mcc_mnc\":\"0010111\""),
         "radio.carriers[0].mcc_mnc: must be 1 to 6"},
        {with("\"rat\":\"LTE\"", "\"rat\":\"5G\""), "radio.carriers[0].rat: must be LTE, NR, WCDMA or GSM"},
        {with("\"band\":20", "\"band\":-1"), "radio.carriers[0].band: must be between 0 and"},
        {with("\"band\":20", "\"band\":20.5"), "radio.carriers[0].band: must be an integer"},
        {with("\"towers\":[{\"id\":\"A1\"", "\"towers\":5,\"x\":[{\"id\":\"A1\""),
         "radio.carriers[0].towers: must be"},
        {with("\"towers\":[{\"id\":\"B1\"", "\"t\":[{\"id\":\"B1\""),
         "radio.carriers[1].towers: required array"},
        {R"({"seed":1,"shadowing_sigma_db":1,"shadow_corr_m":60,"noise_floor_dbm":-104,)"
         R"("carriers":[{"name":"a","towers":[]}]})",
         "radio.carriers[0].towers: must hold at least one tower"},
        {many_towers, "radio.carriers[1].towers[62]: more than 64 towers in total"},
        {with("{\"id\":\"A1\"", "7,{\"id\":\"A1\""), "radio.carriers[0].towers[0]: must be an object"},
        {with("{\"id\":\"A2\"", "{\"id\":\"A1\""), "radio.carriers[0].towers[1].id: duplicate tower id"},
        {with("{\"id\":\"A1\"", "{\"ident\":\"A1\""),
         "radio.carriers[0].towers[0].id: required string is missing"},
        {with("\"lat\":47,\"lon\":8,", "\"lat\":91,\"lon\":8,"),
         "radio.carriers[0].towers[0].lat: must be between -90"},
        {with("\"lat\":47,\"lon\":8,", "\"lat\":47,\"lon\":-180.5,"),
         "radio.carriers[0].towers[0].lon: must be between"},
        {with("\"height_m\":30,", ""), "radio.carriers[0].towers[0].height_m: required number is missing"},
        {with("\"rsrp_1km_dbm\":-75,", ""), "radio.carriers[0].towers[0].rsrp_1km_dbm: required number"},
        {with("\"azimuth_deg\":225,\"beamwidth_deg\":65", "\"azimuth_deg\":225"),
         "radio.carriers[0].towers[1].beamwidth_deg: required for a sector"},
        {with("\"azimuth_deg\":225,\"beamwidth_deg\":65", "\"azimuth_deg\":225,\"beamwidth_deg\":0"),
         "radio.carriers[0].towers[1].beamwidth_deg: must be between 0 and 360"},
        {with("\"azimuth_deg\":225", "\"azimuth_deg\":\"sw\""),
         "radio.carriers[0].towers[1].azimuth_deg: must be a number"},
        {with("\"downtilt_deg\":6", "\"downtilt_deg\":91"),
         "radio.carriers[0].towers[0].downtilt_deg: must be between"},
        {with("\"downtilt_deg\":6,", ""),
         "radio.carriers[0].towers[0].downtilt_deg: required number is missing"},
        {with("\"pci\":101", "\"pci\":1008"), "radio.carriers[0].towers[0].pci: must be between 0 and 1007"},
        {with("\"holes\":[", "\"holes\":7,\"h\":["), "radio.holes: must be an array"},
        {many_holes, "radio.holes: must hold at most 64 holes"},
        {with("\"holes\":[", "\"holes\":[1,"), "radio.holes[0]: must be an object"},
        {with("\"radius_m\":150", "\"radius_m\":0"), "radio.holes[0].radius_m: must be > 0"},
        {with("\"extra_loss_db\":35", "\"extra_loss_db\":-1"), "radio.holes[0].extra_loss_db: must be >= 0"},
        {with("{\"lat\":47.004,", "{"), "radio.holes[0].lat: required number is missing"},
        {with("\"modems\":[\"SimTel A\",\"SimTel B\"]", "\"modems\":\"SimTel A\""),
         "radio.modems: must be an array"},
        {with("\"modems\":[\"SimTel A\",\"SimTel B\"]", "\"modems\":[]"),
         "radio.modems: must name 1 to 8 carriers"},
        {with("\"modems\":[\"SimTel A\",\"SimTel B\"]",
              "\"modems\":[\"a\",\"b\",\"c\",\"d\",\"e\",\"f\",\"g\",\"h\",\"i\"]"),
         "radio.modems: must name 1 to 8 carriers"},
        {with("\"modems\":[\"SimTel A\",\"SimTel B\"]", "\"modems\":[1]"),
         "radio.modems[0]: must be a carrier name"},
        {with("\"modems\":[\"SimTel A\",\"SimTel B\"]", "\"modems\":[\"SimTel A\",\"SimTel C\"]"),
         "radio.modems[1]: \"SimTel C\" is not a carrier in this spec"},
        {with("\"modems\":[\"SimTel A\",\"SimTel B\"]", "\"modems\":[\"SimTel B\",\"SimTel B\"]"),
         "radio.modems[1]: \"SimTel B\" is listed twice"},
        {std::string(kMaxSpecBytes + 1, ' '), "radio spec larger than"},
    };
    for (const Bad &b : bad) {
        const SpecResult r = parse_radio_spec(b.body);
        const bool ok = !r.spec.has_value() && r.error.find(b.error) == 0;
        CHECK(ok);
        if (!ok) {
            std::printf("  want \"%s\", got \"%s\"\n", b.error, r.spec ? "(accepted)" : r.error.c_str());
        }
    }
}

void test_hand_built_specs() {
    // Modems naming nothing known fall back to one per carrier, as an absent list does.
    RadioSpec spec = plain_spec();
    spec.modems = {"nobody"};
    const RadioModel m(spec);
    CHECK(m.modem_carriers() == std::vector<size_t>{0});
    CHECK(m.carrier_index("SimTel A") == std::optional<size_t>(0));
    CHECK(!m.carrier_index("SimTel Z").has_value());
    // No carriers at all: a model still builds (anchored at 0,0), it just has nothing to sample.
    const RadioModel empty{RadioSpec{}};
    CHECK(empty.modem_carriers().empty());
}

void test_sample_json() {
    const RadioModel m = load_vector_model();
    const Position strong{47.0000, 8.0020, 60.0, 60.0};
    const std::string j = sample_json(m, strong);
    std::string err;
    const auto doc = json::parse(j, err);
    CHECK(doc.has_value());
    if (!doc) {
        std::printf("  %s\n  %s\n", err.c_str(), j.c_str());
        return;
    }
    const json::Value *cell = doc->find("cell");
    const json::Value *cells = doc->find("cells");
    const json::Value *carriers = doc->find("carriers");
    CHECK(cell != nullptr && cells != nullptr && carriers != nullptr);
    if (cell == nullptr || cells == nullptr || carriers == nullptr) {
        return;
    }
    CHECK(cell->find("source")->text == "sim");
    CHECK(cell->find("modem")->text == "primary");
    CHECK(cell->find("operator")->text == "SimTel A");
    CHECK(cell->find("mcc_mnc")->text == "00101");
    CHECK(cell->find("rat")->text == "LTE");
    CHECK(cell->find("band")->number == 20.0 && cell->find("earfcn")->number == 6300.0);
    CHECK(cell->find("cell_id")->text == "00101-A1");
    CHECK(cell->find("pci")->number == 101.0);
    const CarrierSample a = m.sample(0, strong);
    NEAR(cell->find("rsrp")->number, a.rsrp_dbm, 1e-9);
    NEAR(cell->find("sinr")->number, a.sinr_db, 0.05 + 1e-9);
    NEAR(cell->find("rsrq")->number, a.rsrq_db, 0.05 + 1e-9);
    NEAR(cell->find("rssi")->number, a.rssi_dbm, 0.05 + 1e-9);
    CHECK(doc->find("link")->text == to_string(classify(a)));
    CHECK(cells->items.size() == 1 && cells->items[0].find("modem")->text == "SimTel B");
    CHECK(carriers->items.size() == 2 && carriers->items[1].find("cell_id")->text == "00102-B1");
    CHECK(carriers->items[0].find("modem")->text == "primary");

    // Unregistered: signal values and the serving cell's identity go null; the carrier stays.
    RadioSpec weak = plain_spec();
    weak.carriers[0].mcc_mnc.reset();
    weak.carriers[0].rat.reset();
    weak.carriers[0].towers[0].pci.reset();
    Carrier other = weak.carriers[0];
    other.name = "SimTel C";
    weak.carriers.push_back(other); // not a modem
    const RadioModel mw(weak);
    const std::string far = sample_json(mw, offset(mw, 60000.0, 0.0, 30.0));
    CHECK(far.find(R"("link":"none")") != std::string::npos);
    CHECK(
        far.find(R"("pci":null,"cell_id":null,"tower":null,"rsrp":null,"rsrq":null,"sinr":null,"rssi":null,)"
                 R"("registered":false})") != std::string::npos);
    CHECK(far.find(R"("operator":"SimTel A","mcc_mnc":null,"rat":null)") != std::string::npos);
    CHECK(far.find(R"("cells":[])") != std::string::npos);
    CHECK(far.find(R"({"source":"sim","modem":null,"operator":"SimTel C")") != std::string::npos);
    // Registered without an MCC/MNC: the cell id is the tower id.
    const std::string close = sample_json(mw, offset(mw, 500.0, 0.0, 30.0));
    CHECK(close.find(R"("pci":null,"cell_id":"A1","tower":"A1")") != std::string::npos);
}

void test_grid() {
    const RadioModel m = load_vector_model();
    GridRequest req;
    req.step_m = 50.0;
    const double dlat = 50.0 / kEarthRadiusM * 180.0 / M_PI;
    req.min_lat_deg = 47.0;
    req.max_lat_deg = 47.0 + 4 * dlat; // exactly four rows
    req.min_lon_deg = 8.0;
    req.max_lon_deg = 8.002;
    req.alt_rel_m = 30.0;
    req.alt_amsl_m = 30.0;
    const auto shape = grid_shape(req, 2, 250000);
    CHECK(shape.has_value());
    if (!shape) {
        return;
    }
    CHECK(shape->rows == 4);
    const double mid = (req.min_lat_deg + req.max_lat_deg) / 2.0;
    const double dlon = dlat / std::cos(mid * M_PI / 180.0);
    NEAR(shape->step_lon_deg, dlon, 1e-12);
    CHECK(shape->cols == static_cast<size_t>(std::ceil(0.002 / dlon)));
    CHECK(!grid_shape(req, 2, shape->rows * shape->cols * 2 - 1).has_value());
    CHECK(grid_shape(req, 2, shape->rows * shape->cols * 2).has_value());
    // A degenerate box is still one cell.
    GridRequest point = req;
    point.max_lat_deg = point.min_lat_deg;
    point.max_lon_deg = point.min_lon_deg;
    CHECK(grid_shape(point, 1, 1).has_value() && grid_shape(point, 1, 1)->rows == 1);
    // A box far too big for its step is refused before anything is sized from it.
    GridRequest huge = req;
    huge.min_lat_deg = -80;
    huge.max_lat_deg = 80;
    huge.min_lon_deg = -180;
    huge.max_lon_deg = 180;
    huge.step_m = 0.001;
    CHECK(!grid_shape(huge, 8, 250000).has_value());

    std::string err;
    const std::string all = grid_json(m, req, *shape);
    const auto doc = json::parse(all, err);
    CHECK(doc.has_value());
    if (!doc) {
        return;
    }
    const json::Value *layers = doc->find("layers");
    CHECK(layers != nullptr && layers->items.size() == 2);
    CHECK(doc->find("rows")->number == 4.0 && doc->find("cols")->number == static_cast<double>(shape->cols));
    const size_t cells = shape->rows * shape->cols;
    for (const json::Value &layer : layers->items) {
        for (const char *k : {"rsrp", "sinr", "rsrq", "rssi", "quality"}) {
            CHECK(layer.find(k) != nullptr && layer.find(k)->items.size() == cells);
        }
    }
    // Row-major from the south-west corner, sampled at cell centres.
    const size_t r = 2;
    const size_t c = 1;
    const Position centre{47.0 + 2.5 * dlat, 8.0 + 1.5 * shape->step_lon_deg, 30.0, 30.0};
    const CarrierSample s = m.sample(1, centre);
    const json::Value &layer_b = layers->items[1];
    CHECK(layer_b.find("carrier")->text == "SimTel B");
    const json::Value &v = layer_b.find("rsrp")->items[r * shape->cols + c];
    if (s.registered) {
        NEAR(v.number, s.rsrp_dbm, 1e-9);
    } else {
        CHECK(v.is_null());
    }
    CHECK(layer_b.find("quality")->items[r * shape->cols + c].text == to_string(classify(s)));

    // One carrier only.
    GridRequest only_b = req;
    only_b.carriers = {1};
    const std::string one = grid_json(m, only_b, *shape);
    CHECK(one.find("\"SimTel A\"") == std::string::npos && one.find("\"SimTel B\"") != std::string::npos);

    // A hole's centre shows up as "none" or "bad" when the box is placed over it.
    GridRequest over_hole = req;
    over_hole.min_lat_deg = 47.0040 - dlat / 2;
    over_hole.max_lat_deg = 47.0040 + dlat / 2;
    over_hole.min_lon_deg = 8.0060 - shape->step_lon_deg / 2;
    over_hole.max_lon_deg = 8.0060 + shape->step_lon_deg / 2;
    over_hole.carriers = {0};
    const auto hs = grid_shape(over_hole, 1, 10);
    CHECK(hs.has_value() && hs->rows * hs->cols == 1);
    if (hs) {
        const std::string hj = grid_json(m, over_hole, *hs);
        CHECK(hj.find("\"quality\":[\"none\"]") != std::string::npos ||
              hj.find("\"quality\":[\"bad\"]") != std::string::npos);
    }
}

void test_field() {
    RadioField field;
    CHECK(field.get() == nullptr);
    CHECK(!field.clear());
    auto a = std::make_shared<const RadioModel>(plain_spec());
    RadioSpec other = plain_spec();
    other.carriers[0].towers[0].rsrp_1km_dbm = -60.0;
    auto b = std::make_shared<const RadioModel>(other);
    field.set(a);
    CHECK(field.get() == a);

    // Readers sampling while a writer swaps: each read sees one whole model, A or B.
    const Position p = offset(*a, 1000.0, 0.0, 30.0);
    const double ra = a->sample(0, p).rsrp_exact_dbm;
    const double rb = b->sample(0, p).rsrp_exact_dbm;
    std::atomic<bool> stop{false};
    std::atomic<int> torn{0};
    std::atomic<int> reads{0};
    std::vector<std::thread> readers;
    for (int k = 0; k < 3; ++k) {
        readers.emplace_back([&] {
            while (!stop.load()) {
                const auto model = field.get();
                if (model) {
                    const double v = model->sample(0, p).rsrp_exact_dbm;
                    if (v != ra && v != rb) {
                        ++torn;
                    }
                    ++reads;
                }
            }
        });
    }
    for (int k = 0; k < 2000 || reads.load() < 1000; ++k) {
        field.set(k % 2 == 0 ? b : a);
        if (k % 100 == 99) {
            field.clear();
        }
    }
    stop = true;
    for (auto &t : readers) {
        t.join();
    }
    CHECK(torn.load() == 0);
    field.set(b);
    CHECK(field.clear());
    CHECK(field.get() == nullptr);
}

void test_vector() {
    const RadioModel m = load_vector_model();
    if (m.spec().carriers.size() != 2) {
        return;
    }
    // The committed vector must be exactly what this build computes. If the model changes,
    // regenerate it (test_radio --write-vector) and re-run the cross-check with the other
    // implementation: a drift here is a contract change, not a refactor.
    const std::string committed = read_file(kFixtures + "radio_test_vector.json");
    const std::string computed = vector_json(m);
    CHECK(committed == computed);
    if (committed != computed) {
        std::printf("  tests/fixtures/radio_test_vector.json differs from the model:\n%s", computed.c_str());
    }
    std::string err;
    const auto doc = json::parse(committed, err);
    CHECK(doc.has_value() && doc->is_array() && doc->items.size() == 24);

    // Spot values derived by hand from the formulas, independent of vector_json: at
    // (47.0000, 8.0020, 60) the vehicle is ~152 m east of A1 and 30 m above it.
    const Position p{47.0000, 8.0020, 60.0, 60.0};
    double x = 0.0;
    double y = 0.0;
    m.to_local(p.lat_deg, p.lon_deg, x, y);
    const double ground = std::hypot(x, y);
    const double theta = std::atan2(30.0, ground) * 180.0 / M_PI;
    const double a1 = -75.0 - 10.0 * 2.9 * std::log10(std::hypot(ground, 30.0) / 1000.0) -
                      std::min(20.0, 12.0 * std::pow((theta + 6.0) / 10.0, 2.0)) + m.shadowing_db(0, x, y) -
                      m.hole_loss_db(x, y);
    NEAR(m.tower_rsrp_dbm(0, 0, p), a1, 1e-9);
}

} // namespace

int main(int argc, char **argv) {
    if (argc == 3 && std::strcmp(argv[1], "--write-vector") == 0) {
        const RadioModel m = load_vector_model();
        if (g_failures != 0) {
            return 1;
        }
        std::ofstream out(argv[2], std::ios::binary);
        out << vector_json(m);
        std::printf("test_radio: wrote %s\n", argv[2]);
        return out.good() ? 0 : 1;
    }

    test_json_reader();
    test_terms();
    test_prng();
    test_frame_and_fields();
    test_tower_rsrp();
    test_sample();
    test_determinism();
    test_spec_parsing();
    test_hand_built_specs();
    test_sample_json();
    test_grid();
    test_field();
    test_vector();

    if (g_failures == 0) {
        std::printf("test_radio: all checks OK\n");
        return 0;
    }
    std::printf("test_radio: %d failure(s)\n", g_failures);
    return 1;
}

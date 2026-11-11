// /radio routes and the per-vehicle radio object in GET /vehicles, through the real HTTP
// stack (httplib::Client) against canned snapshots. The model itself is test_radio's.
#include <httplib.h>

#include <atomic>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "api/control_server.h"
#include "radio/radio_model.h"
#include "radio/json.h"
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

constexpr int kPort = 18646;
constexpr int kNoHomePort = 18648;

std::string read_file(const std::string &path) {
    std::ifstream in(path, std::ios::binary);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

bool has(const httplib::Result &r, int status, const std::string &needle = "") {
    return r && r->status == status && r->body.find(needle) != std::string::npos;
}

} // namespace

int main() {
    using namespace skysim::api;
    namespace radio = skysim::radio;

    const std::string spec_text =
        read_file(std::string(SKYSIM_SOURCE_DIR) + "/tests/fixtures/radio_test_spec.json");
    const radio::SpecResult parsed = radio::parse_radio_spec(spec_text);
    CHECK(parsed.spec.has_value());
    if (!parsed.spec) {
        return 1;
    }
    const radio::RadioModel reference(*parsed.spec);
    const std::string canonical = radio::radio_spec_json(*parsed.spec);

    CommandQueue queue; // no route here queues anything
    ControlServer::Snapshots snaps;
    // One vehicle 60 m up, ~152 m east of home: the contract's (47.0000, 8.0020, 60) point.
    const radio::GeoAnchor home{47.0, 8.0, 0.0};
    VehicleInfo vehicle;
    vehicle.id = 1;
    vehicle.instance = 3;
    vehicle.pos_ned[0] = 0.0;
    vehicle.pos_ned[1] = 151.6573;
    vehicle.pos_ned[2] = -60.0;
    snaps.vehicles = [vehicle] { return std::vector<VehicleInfo>{vehicle}; };
    snaps.metrics = [] { return MetricsInfo{}; };

    auto field = std::make_shared<radio::RadioField>();
    ControlServer::RadioConfig cfg;
    cfg.field = field;
    cfg.home = home;
    ControlServer server("127.0.0.1", kPort, queue, snaps, cfg);
    httplib::Client client("127.0.0.1", kPort);
    client.set_read_timeout(30, 0);

    // --- no spec yet: an empty field, not an error, except where a sample is asked for. ---
    CHECK(has(client.Get("/radio"), 200, "{\"radio\":null}"));
    CHECK(has(client.Get("/radio/sample?lat=47&lon=8&alt_amsl=30"), 404, "no radio spec set"));
    CHECK(has(client.Get("/radio/grid?bbox=7.99,46.99,8.01,47.01&alt_rel=30"), 404, "no radio spec set"));
    CHECK(has(client.Delete("/radio"), 200, "\"cleared\":false"));
    auto bare_list = client.Get("/vehicles");
    CHECK(has(bare_list, 200, "\"id\":1") && bare_list->body.find("radio") == std::string::npos);

    // --- PUT validates before anything is replaced. ---
    CHECK(has(client.Put("/radio", "{not json", "application/json"), 400, "invalid JSON at offset 1"));
    CHECK(has(client.Put("/radio", R"({"radio":{"seed":1}})", "application/json"), 400,
              "radio.shadowing_sigma_db: required number is missing"));
    CHECK(has(client.Get("/radio"), 200, "{\"radio\":null}"));

    // --- set it: wrapped form, as a gateway forwards a simulator-link object. ---
    auto put = client.Put("/radio", spec_text, "application/json");
    CHECK(has(put, 200, R"({"ok":true,"carriers":2,"towers":3,"holes":1,"modems":["SimTel A","SimTel B"]})"));
    auto got = client.Get("/radio");
    CHECK(got && got->status == 200 && got->body == "{\"radio\":" + canonical + "}");
    // What GET returns can be PUT back as is (and the bare form works too).
    CHECK(has(client.Put("/radio", got->body, "application/json"), 200, "\"ok\":true"));
    CHECK(has(client.Put("/radio", canonical, "application/json"), 200, "\"ok\":true"));
    auto again = client.Get("/radio");
    CHECK(again && again->body == got->body);

    // --- sample: every altitude form resolves to the same point; values are the model's. ---
    const std::string expect = radio::sample_json(reference, radio::Position{47.0, 8.002, 60.0, 60.0});
    for (const char *q :
         {"/radio/sample?lat=47&lon=8.002&alt_amsl=60", "/radio/sample?lat=47&lon=8.002&alt_rel=60",
          "/radio/sample?lat=47.0&lon=8.002&alt_rel=60&ground_amsl=0",
          "/radio/sample?lat=47&lon=8.002&alt_amsl=60&alt_rel=60"}) {
        auto r = client.Get(q);
        CHECK(r && r->status == 200 && r->body == expect);
        if (!r || r->body != expect) {
            std::printf("  %s\n", q);
        }
    }
    // Over higher ground alt_rel follows from it: 460 AMSL over 400 m ground is 60 m up.
    auto high = client.Get("/radio/sample?lat=47&lon=8.002&alt_amsl=460&ground_amsl=400");
    CHECK(high && high->body == radio::sample_json(reference, radio::Position{47.0, 8.002, 60.0, 460.0}));
    auto high2 = client.Get("/radio/sample?lat=47&lon=8.002&alt_rel=60&ground_amsl=400");
    CHECK(high2 && high2->body == high->body);
    CHECK(has(client.Get("/radio/sample?lat=47&lon=8.002&alt_amsl=60"), 200, R"("cell":{"source":"sim")"));

    const char *bad_samples[][2] = {
        {"/radio/sample?lon=8&alt_amsl=60", "lat and lon are required"},
        {"/radio/sample?lat=47&alt_amsl=60", "lat and lon are required"},
        {"/radio/sample?lat=91&lon=8&alt_amsl=60", "lat must be between -90 and 90"},
        {"/radio/sample?lat=47&lon=181&alt_amsl=60", "lon must be between -180 and 180"},
        {"/radio/sample?lat=abc&lon=8&alt_amsl=60", "lat must be a number"},
        {"/radio/sample?lat=&lon=8&alt_amsl=60", "lat must be a number"},
        {"/radio/sample?lat=47x&lon=8&alt_amsl=60", "lat must be a number"},
        {"/radio/sample?lat=nan&lon=8&alt_amsl=60", "lat must be a number"},
        {"/radio/sample?lat=47&lon=8", "alt_amsl or alt_rel (with ground_amsl) is required"},
        {"/radio/sample?lat=47&lon=8&ground_amsl=10", "alt_amsl or alt_rel (with ground_amsl) is required"},
        {"/radio/sample?lat=47&lon=8&alt_rel=1e9", "alt_rel must be between -1000 and 100000"},
        {"/radio/sample?lat=47&lon=8&alt_amsl=inf", "alt_amsl must be a number"},
        {"/radio/sample?lat=47&lon=8&alt_rel=10&ground_amsl=x", "ground_amsl must be a number"},
    };
    for (const auto &b : bad_samples) {
        auto r = client.Get(b[0]);
        CHECK(has(r, 400, b[1]));
        if (!has(r, 400, b[1])) {
            std::printf("  %s -> %s\n", b[0], r ? r->body.c_str() : "(no response)");
        }
    }

    // --- grid ---
    auto grid = client.Get("/radio/grid?bbox=7.999,46.999,8.001,47.001&alt_rel=30&step_m=50");
    CHECK(has(grid, 200, R"("layers":[{"carrier":"SimTel A")"));
    CHECK(grid && grid->body.find(R"({"carrier":"SimTel B")") != std::string::npos);
    CHECK(grid && grid->body.find("\"rows\":5,") != std::string::npos);
    CHECK(grid && grid->body.find("\"alt_amsl\":30.00") != std::string::npos);
    auto grid_b = client.Get("/radio/grid?bbox=7.999,46.999,8.001,47.001&alt_amsl=30&carrier=SimTel%20B");
    CHECK(has(grid_b, 200, R"("layers":[{"carrier":"SimTel B")"));
    CHECK(grid_b && grid_b->body.find("SimTel A") == std::string::npos);
    // Default step is 50 m: same raster as asking for it.
    auto grid_default = client.Get("/radio/grid?bbox=7.999,46.999,8.001,47.001&alt_rel=30");
    CHECK(grid_default && grid && grid_default->body == grid->body);

    const char *bad_grids[][2] = {
        {"/radio/grid?alt_rel=30", "bbox must be minLon,minLat,maxLon,maxLat"},
        {"/radio/grid?bbox=1,2,3&alt_rel=30", "bbox must be minLon,minLat,maxLon,maxLat"},
        {"/radio/grid?bbox=1,2,3,4,5&alt_rel=30", "bbox must be minLon,minLat,maxLon,maxLat"},
        {"/radio/grid?bbox=1;2,3,4&alt_rel=30", "bbox must be minLon,minLat,maxLon,maxLat"},
        {"/radio/grid?bbox=1,2,3,nan&alt_rel=30", "bbox must be minLon,minLat,maxLon,maxLat"},
        {"/radio/grid?bbox=8.01,47,8,47.01&alt_rel=30", "bbox must lie within"},
        {"/radio/grid?bbox=8,47.01,8.01,47&alt_rel=30", "bbox must lie within"},
        {"/radio/grid?bbox=-181,47,8,47.01&alt_rel=30", "bbox must lie within"},
        {"/radio/grid?bbox=8,47,8.01,91&alt_rel=30", "bbox must lie within"},
        {"/radio/grid?bbox=8,47,8.01,47.01", "alt_amsl or alt_rel (with ground_amsl) is required"},
        {"/radio/grid?bbox=8,47,8.01,47.01&alt_rel=30&step_m=0.5", "step_m must be between 1 and 100000"},
        {"/radio/grid?bbox=8,47,8.01,47.01&alt_rel=30&carrier=Nobody",
         R"(carrier \"Nobody\" is not in the radio spec)"},
        {"/radio/grid?bbox=-10,-10,10,10&alt_rel=30&step_m=100", "grid too large: more than 250000 cells"},
    };
    for (const auto &b : bad_grids) {
        auto r = client.Get(b[0]);
        CHECK(has(r, 400, b[1]));
        if (!has(r, 400, b[1])) {
            std::printf("  %s -> %s\n", b[0], r ? r->body.c_str() : "(no response)");
        }
    }
    // The cap counts layers: a box that fits for one carrier can be too big for two.
    // 1° x 1° at 250 m is ~445 x ~304 = ~135k cells per carrier.
    CHECK(has(client.Get("/radio/grid?bbox=8,47,9,48&alt_rel=30&step_m=250&carrier=SimTel%20A"), 200));
    CHECK(has(client.Get("/radio/grid?bbox=8,47,9,48&alt_rel=30&step_m=250"), 400, "grid too large"));

    // A query value that is not UTF-8 is quoted back as U+FFFD, so the error is still JSON.
    auto not_utf8 = client.Get("/radio/grid?bbox=8,47,8.01,47.01&alt_rel=30&carrier=%FF");
    CHECK(has(not_utf8, 400, R"(carrier \"\ufffd\" is not in the radio spec)"));
    std::string json_error;
    CHECK(not_utf8 && radio::json::parse(not_utf8->body, json_error).has_value());

    // At most kMaxGridsInFlight rasters compute at once; the rest are told to come back.
    // Rounds of simultaneous near-cap requests until one is turned away.
    {
        int ok = 0;
        int busy = 0;
        int other = 0;
        for (int round = 0; round < 20 && busy == 0; ++round) {
            std::atomic<int> r_ok{0};
            std::atomic<int> r_busy{0};
            std::atomic<int> r_other{0};
            std::vector<std::thread> burst;
            for (int k = 0; k < 8; ++k) {
                burst.emplace_back([&] {
                    httplib::Client bc("127.0.0.1", kPort);
                    bc.set_read_timeout(60, 0);
                    // ~233k cells: just under the cap, so each takes as long as one can.
                    auto r = bc.Get("/radio/grid?bbox=8,47,9,48&alt_rel=30&step_m=190&carrier=SimTel%20A");
                    if (has(r, 200, "\"layers\"")) {
                        ++r_ok;
                    } else if (has(r, 503, "grids already computing")) {
                        ++r_busy;
                    } else {
                        ++r_other;
                    }
                });
            }
            for (auto &t : burst) {
                t.join();
            }
            ok += r_ok.load();
            busy += r_busy.load();
            other += r_other.load();
        }
        CHECK(busy > 0);
        CHECK(ok >= ControlServer::kMaxGridsInFlight);
        CHECK(other == 0);
        // Every slot came back.
        CHECK(has(client.Get("/radio/grid?bbox=8,47,8.01,47.01&alt_rel=30"), 200));
    }

    // --- vehicles carry the field at their truth position. ---
    auto list = client.Get("/vehicles");
    const std::string vradio = radio::sample_json(reference, radio::position_from_ned(home, vehicle.pos_ned));
    CHECK(has(list, 200, ",\"radio\":" + vradio + "}]"));
    CHECK(list && list->body.find("\"pos_ned\":[0.000,151.657,-60.000]") != std::string::npos);

    // A server that does not know where its world is cannot place a vehicle, but it shares
    // the field: a spec set through one server is visible through the other.
    {
        ControlServer::RadioConfig no_home;
        no_home.field = field;
        ControlServer other("127.0.0.1", kNoHomePort, queue, snaps, no_home);
        httplib::Client oc("127.0.0.1", kNoHomePort);
        oc.set_read_timeout(30, 0);
        CHECK(has(oc.Get("/radio"), 200, canonical));
        auto ol = oc.Get("/vehicles");
        CHECK(has(ol, 200, "\"id\":1") && ol->body.find("radio") == std::string::npos);
    }

    // --- concurrent readers while the spec is replaced and cleared. Each answer is whole:
    // a sample of the spec, or "no spec" — never a 500 or a dropped connection. ---
    {
        std::atomic<bool> stop{false};
        std::atomic<int> bad{0};
        std::atomic<int> answered{0};
        std::vector<std::thread> readers;
        for (int k = 0; k < 3; ++k) {
            readers.emplace_back([&] {
                httplib::Client rc("127.0.0.1", kPort);
                rc.set_read_timeout(30, 0);
                while (!stop.load()) {
                    auto r = rc.Get("/radio/sample?lat=47&lon=8.002&alt_amsl=60");
                    if (!r || (r->status != 200 && r->status != 404) ||
                        (r->status == 200 && r->body != expect)) {
                        ++bad;
                    }
                    ++answered;
                }
            });
        }
        httplib::Client writer("127.0.0.1", kPort);
        writer.set_read_timeout(30, 0);
        for (int k = 0; k < 40 || answered.load() < 60; ++k) {
            if (k % 2 == 0) {
                writer.Put("/radio", spec_text, "application/json");
            } else {
                writer.Delete("/radio");
            }
        }
        stop = true;
        for (auto &t : readers) {
            t.join();
        }
        CHECK(bad.load() == 0);
    }

    // --- delete: back to no field. ---
    CHECK(has(client.Put("/radio", spec_text, "application/json"), 200));
    CHECK(has(client.Delete("/radio"), 200, "\"cleared\":true"));
    CHECK(has(client.Get("/radio"), 200, "{\"radio\":null}"));
    auto after = client.Get("/vehicles");
    CHECK(has(after, 200, "\"id\":1") && after->body.find("radio") == std::string::npos);
    CHECK(field->get() == nullptr);

    // A server given no field keeps one of its own.
    {
        ControlServer own("127.0.0.1", kNoHomePort, queue, snaps);
        httplib::Client oc("127.0.0.1", kNoHomePort);
        oc.set_read_timeout(30, 0);
        CHECK(has(oc.Put("/radio", spec_text, "application/json"), 200));
        CHECK(has(oc.Get("/radio"), 200, canonical));
        CHECK(field->get() == nullptr);
    }

    if (g_failures == 0) {
        std::printf("test_radio_api: all checks OK\n");
        return 0;
    }
    std::printf("test_radio_api: %d failure(s)\n", g_failures);
    return 1;
}

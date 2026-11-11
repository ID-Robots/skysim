# Simulated 4G radio field

skysim can carry a simulated LTE radio field: a few carriers, each with towers, over a world
with patches of extra loss ("holes"). Given a spec, it answers what a modem would report at
any position — RSRP, RSRQ, SINR, RSSI, the serving cell, whether it is registered at all —
and attaches those values to every vehicle in `GET /vehicles`.

It exists so a network-quality survey can be developed and tested in simulation: a flight
recorder that simulates its modems from the same spec writes plausible measurements, and
skysim holds the ground truth they can be checked against — per position, per vehicle, or as
a raster over an area.

The field is deterministic: the same spec and position always give the same values, on any
machine. It is also a shared model — another implementation (a flight recorder's simulated
modem provider) implements the same formulas, and both commit the output of one fixed case
as a cross-check (see [Cross-check vector](#cross-check-vector)). Changing a formula here is a
contract change, not a refactor.

Code: `src/radio/` (no Jolt, no HTTP). Tests: `tests/test_radio.cpp` (model, spec, vector) and
`tests/test_radio_api.cpp` (routes).

## Spec

```json
{"radio": {
  "seed": 7,
  "shadowing_sigma_db": 6.0,
  "shadow_corr_m": 60.0,
  "noise_floor_dbm": -104.0,
  "modems": ["SimTel A", "SimTel B"],
  "carriers": [
    {"name": "SimTel A", "mcc_mnc": "00101", "rat": "LTE", "band": 20, "earfcn": 6300,
     "towers": [
       {"id": "A1", "lat": 47.0000, "lon": 8.0000, "height_m": 30, "rsrp_1km_dbm": -75,
        "azimuth_deg": null, "beamwidth_deg": 65, "downtilt_deg": 6, "pci": 101},
       {"id": "A2", "lat": 47.0100, "lon": 8.0150, "height_m": 25, "rsrp_1km_dbm": -78,
        "azimuth_deg": 225, "beamwidth_deg": 65, "downtilt_deg": 4, "pci": 102}]},
    {"name": "SimTel B", "mcc_mnc": "00102", "rat": "LTE", "band": 3, "earfcn": 1300,
     "towers": [
       {"id": "B1", "lat": 46.9950, "lon": 8.0100, "height_m": 35, "rsrp_1km_dbm": -80,
        "azimuth_deg": 0, "beamwidth_deg": 90, "downtilt_deg": 5, "pci": 201}]}],
  "holes": [{"lat": 47.0040, "lon": 8.0060, "radius_m": 150, "extra_loss_db": 35}]
}}
```

Accepted wrapped (`{"radio": {...}}`, so a whole simulator-link object can be passed through
with its other keys ignored) or bare. Unknown keys are ignored. Strings must be UTF-8. Every
number is range-checked — the bounds below sit far outside anything physical, but a finite
yet absurd value (a tower at +3000 dBm) would otherwise overflow the model to infinity.

| Field | Type | Rule |
|---|---|---|
| `seed` | integer | required, 0 … 2⁶⁴−1 |
| `shadowing_sigma_db` | number | required, 0 … 30 |
| `shadow_corr_m` | number | required, 1 … 100 000 |
| `noise_floor_dbm` | number | required, −200 … 0 |
| `modems` | list of carrier names | optional; default: every carrier in spec order. 1–8 names, each a carrier of this spec, no repeats. `[0]` is the primary modem |
| `carriers` | list | required, 1–8 |
| `carriers[].name` | string | required, 1–64 bytes, unique. Reported as `operator` |
| `carriers[].mcc_mnc` | string \| null | optional, 5 or 6 digits |
| `carriers[].rat` | string \| null | optional, `LTE`, `NR`, `WCDMA` or `GSM` |
| `carriers[].band`, `earfcn` | integer \| null | optional, ≥ 0 |
| `carriers[].towers` | list | required, ≥ 1; at most **64 towers across all carriers** |
| `towers[].id` | string | required, 1–64 bytes, unique within its carrier |
| `towers[].lat`, `lon` | number | required, degrees, within ±90 / ±180 |
| `towers[].height_m` | number | required, −1000 … 10 000; antenna height on the same axis as the vehicle's `alt_amsl` (see below) |
| `towers[].rsrp_1km_dbm` | number | required, −200 … 0; RSRP 1 km out on the main beam |
| `towers[].azimuth_deg` | number \| null | optional, −360 … 360; null or absent = omnidirectional |
| `towers[].beamwidth_deg` | number | 0 < bw ≤ 360; required when `azimuth_deg` is set |
| `towers[].downtilt_deg` | number | required, −90 … 90 |
| `towers[].pci` | integer \| null | optional, 0 … 1007 |
| `holes` | list | optional, at most 64 |
| `holes[].lat`, `lon` | number | required |
| `holes[].radius_m` | number | required, 0 < r ≤ 100 000 |
| `holes[].extra_loss_db` | number | required, 0 … 300 |

A bad spec is refused with a 400 whose `error` names the first problem and where it is,
e.g. `{"error":"radio.carriers[0].towers[1].beamwidth_deg: required for a sector (azimuth_deg set)"}`.

## Model

For a position (lat, lon, `alt_rel` above home, `h` = `alt_amsl`) and each carrier `c`, every
tower `t` of `c` gets:

1. `d` = 3-D distance from the antenna (tower lat, lon, `height_m`) to the position, at least
   10 m; `d_km = d / 1000`.
2. Path-loss exponent `n = max(2.0, 3.5 − 0.01 · max(0, alt_rel))` — more line of sight
   higher up.
3. Horizontal pattern, sectors only: `A_h = min(20, 12 · (Δaz / beamwidth)²)`, Δaz the smallest
   angle between the tower→position bearing (clockwise from north) and `azimuth_deg`. Omni: 0.
4. Vertical pattern: `θ = atan2(h − height_m, ground distance)` in degrees,
   `A_v = min(20, 12 · ((θ + downtilt_deg) / 10)²)`.
5. Shadowing `S`: a grid of `shadow_corr_m` cells in the local frame below. Node `(ix, iy)` of
   carrier index `c` (its position in `carriers`) holds a standard normal value from
   splitmix64 seeded with `seed ^ (ix·73856093) ^ (iy·19349663) ^ (c·83492791)` (64-bit
   wrapping arithmetic, negative indices as two's complement): two draws,
   `u = (next() >> 11) · 2⁻⁵³`, `u1` clamped to ≥ 1e−12, `z = sqrt(−2 ln u1) · cos(2π u2)`.
   `ix = floor(x / corr)`, `iy = floor(y / corr)`; `S` is the bilinear interpolation of the
   four surrounding nodes by the fractional parts, times `shadowing_sigma_db`.
6. Holes: `H = Σ extra_loss_db · sqrt(max(0, 1 − dist / radius_m))` over the holes.
7. `rsrp_t = rsrp_1km_dbm − 10 · n · log10(d_km) − A_h − A_v + S − H`.

The **serving tower** is the strongest `rsrp_t` (the first listed wins a tie). With `S_lin` its
power in mW, `I_lin` the sum of the carrier's other towers in mW and `N_lin` the noise floor
in mW:

| Value | Formula |
|---|---|
| `rsrp` | serving `rsrp_t`, rounded to 0.1 dB |
| `sinr` | `10·log10(S_lin / (I_lin + N_lin))`, clamped to [−20, 40] |
| `rsrq` | `−3 − 10·log10(1 + 1/sinr_lin) − 5`, clamped to [−19.5, −3] (`sinr_lin` from the clamped `sinr`) |
| `rssi` | `10·log10(S_lin + I_lin + N_lin) + 10·log10(12 · 50)` (50 resource blocks) |
| `registered` | `rsrp ≥ −124` and `sinr ≥ −8` |

Link quality, from the primary modem: `none` if not registered; `bad` if `rsrp < −110` or
`sinr < 0`; `weak` if `rsrp < −100` or `sinr < 5`; else `good`.

Frames and conventions:

- **Horizontal geometry** — distances, bearings, hole distances and the shadowing grid — is
  measured in one local frame: east/north metres from the first tower of the first carrier,
  equirectangular (`x = R·Δlon·cos(lat0)`, `y = R·Δlat`, `R` = 6 371 000 m, Δlon taken the
  short way round). Over the few kilometres a field spans this is within centimetres of a
  great-circle distance.
- **Vertical**: `height_m` is compared directly with the position's `alt_amsl`, i.e. it is the
  antenna's height AMSL. Over ground at 0 m AMSL (the reference case) that is also its height
  above ground; over higher ground, add the ground elevation to the antenna height.
- Values are computed unrounded; `registered` and the quality class use the 0.1 dB `rsrp`.
  The API prints `rsrp`, `rsrq`, `sinr` and `rssi` at 0.1 dB.
- The model has no RSRP reporting floor or ceiling: right next to a tower RSRP can exceed the
  −44 dBm a real modem would report.

## API

All on the control plane (`--api-port`). The field lives on the HTTP side — no tick reads it —
so these routes never wait on the simulation. A new spec is parsed and built first, then
swapped in at once: a concurrent reader sees the old field or the new one, never a mix.

### `PUT /radio`

Body: the spec, wrapped or bare. Replaces the current field.

```bash
curl -X PUT --data-binary @tests/fixtures/radio_test_spec.json http://127.0.0.1:8642/radio
# {"ok":true,"carriers":2,"towers":3,"holes":1,"modems":["SimTel A","SimTel B"]}
```

400 with `{"error": "..."}` for invalid JSON or an invalid spec (the current field stays);
bodies over 256 KiB are refused the same way.

### `GET /radio`, `DELETE /radio`

`GET` returns `{"radio": <spec>}` — the canonical form, defaults filled in, which can be PUT back
as is — or `{"radio": null}` when no field is set. `DELETE` clears it:
`{"ok":true,"cleared":true|false}`.

### `GET /radio/sample`

| Parameter | |
|---|---|
| `lat`, `lon` | required, degrees |
| `alt_amsl` | metres AMSL — the `h` towers are compared with |
| `alt_rel` | metres above home — sets the path-loss exponent |
| `ground_amsl` | home ground AMSL, default 0 |

At least one of `alt_amsl` / `alt_rel` is required. Given `alt_amsl` alone,
`alt_rel = alt_amsl − ground_amsl`; given `alt_rel` alone, `alt_amsl = alt_rel + ground_amsl`.
Given both, both are used as given and `ground_amsl` plays no part — the way a recorder
holds them, a GPS altitude and a height above home that need not agree to the metre.
Altitudes must lie within −1000 … 100 000 m.

```bash
curl 'http://127.0.0.1:8642/radio/sample?lat=47.0&lon=8.002&alt_rel=60'
```

```json
{"lat":47.0000000,"lon":8.0020000,"alt_rel":60.00,"alt_amsl":60.00,"link":"good",
 "cell":{"source":"sim","modem":"primary","operator":"SimTel A","mcc_mnc":"00101","rat":"LTE",
         "band":20,"earfcn":6300,"pci":101,"cell_id":"00101-A1","tower":"A1",
         "rsrp":-77.8,"rsrq":-8.1,"sinr":14.7,"rssi":-49.9,"registered":true},
 "cells":[{"source":"sim","modem":"SimTel B","operator":"SimTel B","mcc_mnc":"00102", "...": "..."}],
 "carriers":[{"modem":"primary","operator":"SimTel A", "...": "..."},
             {"modem":"SimTel B","operator":"SimTel B", "...": "..."}]}
```

- `link` — the quality class of the primary modem.
- `cell` — the primary modem (`modems[0]`), `modem: "primary"`.
- `cells` — the other modems, each with `modem` = its carrier name.
- `carriers` — every carrier in spec order, including any that is not a modem
  (`modem: null`).

Cell objects use the flight-recorder sample field names: `source` (always `"sim"`), `modem`,
`operator`, `mcc_mnc`, `rat`, `band`, `earfcn`, `pci`, `cell_id` (`"<mcc_mnc>-<tower id>"`, or
the tower id when the carrier has no MCC/MNC), `tower` (serving tower id), `rsrp`, `rsrq`,
`sinr`, `rssi`, `registered`. When not registered, `rsrp`, `rsrq`, `sinr`, `rssi` and the serving
cell's identity (`pci`, `cell_id`, `tower`) are null; the carrier's fields stay.

404 when no field is set; 400 for a missing or malformed parameter.

### `GET /radio/grid`

A ground-truth raster.

| Parameter | |
|---|---|
| `bbox` | required, `minLon,minLat,maxLon,maxLat` |
| `alt_rel` / `alt_amsl` / `ground_amsl` | as for `/radio/sample` |
| `step_m` | cell size in metres, 1 … 100 000, default 50 |
| `carrier` | a carrier name; default every carrier |

Cells are `step_m` apart (the longitude step uses the box's mid latitude), sampled at cell
centres, row-major from the south-west corner: value `i` is row `i / cols` (south → north),
column `i % cols` (west → east). The grid covers the box, so the last row and column may
reach past it.

```bash
curl 'http://127.0.0.1:8642/radio/grid?bbox=8.003,47.003,8.0045,47.0045&alt_rel=30&step_m=100&carrier=SimTel%20A'
```

```json
{"bbox":[8.0030000,47.0030000,8.0045000,47.0045000],"origin":{"lat":47.0030000,"lon":8.0030000},
 "step_m":100.000,"step_deg":{"lat":0.000899322,"lon":0.001318749},"rows":2,"cols":2,
 "order":"row_major_south_to_north","sample_at":"cell_center","alt_rel":30.00,"alt_amsl":30.00,
 "layers":[{"carrier":"SimTel A","rsrp":[-73.5,-91.9,-69.5,-93.5],"sinr":[12.9,8.1,10.1,5.8],
            "rsrq":[-8.2,-8.6,-8.4,-9.0],"rssi":[-45.5,-63.4,-41.3,-64.7],
            "quality":["good","good","good","good"]}]}
```

Values are null where the carrier is not registered (`quality` is then `"none"`). A request
over **250 000 cells** (rows × columns × carriers) is refused with a 400 — raise `step_m`,
shrink the box or pick a carrier. At most two rasters compute at once; a request beyond that
gets a 503 to retry, so a burst of them cannot take the cores the physics tick needs.

### `GET /vehicles`

With a field set, each vehicle gains a `radio` object — the `/radio/sample` response at its
truth position:

- lat/lon: the world's NED origin is `--spawn-home` (the same point every managed SITL gets as
  `--home`); the vehicle's north/east offset is applied equirectangularly about it.
- `alt_rel` = −down, the vehicle's height above the home ground.
- `alt_amsl` = the `--spawn-home` altitude field (taken as the home ground's AMSL height) +
  `alt_rel`.

No field, no `radio` key. A `--spawn-home` that is not `lat,lon[,alt[,yaw]]` — every field
a number, within ±90 / ±180 (and ±10 000 m) — leaves vehicles without radio values, and says
so at startup. A
vehicle whose position is not finite samples as unregistered, with null coordinates.

## Loading a field at startup

`--radio-spec FILE` (container: `SKYSIM_RADIO_SPEC`) loads a spec file at startup, in the same
format as `PUT /radio`. A missing or invalid file stops startup with exit code 2. The field
can still be replaced or cleared over the API afterwards.

## Cross-check vector

`tests/fixtures/radio_test_vector.json` is the model's output for a fixed case: the spec in
`tests/fixtures/radio_test_spec.json` (the one above; ground at 0 m AMSL, so
`alt_amsl = alt_rel`) at twelve positions, for both carriers. Each row is
`{lat, lon, alt_rel, carrier, rsrp, sinr, rsrq, registered}`, values rounded to 0.01; `rsrp` is
the serving tower's unrounded RSRP and the signal values are null when not registered.
Another implementation of the model computes the same case; the two files must agree within
±0.5 dB.

`test_radio` fails if the committed file is not exactly what the model computes. After a
deliberate model change, regenerate it and re-run the cross-check:

```bash
./build/test_radio --write-vector tests/fixtures/radio_test_vector.json
```

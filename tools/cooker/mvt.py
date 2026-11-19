"""Mapbox Vector Tile decoding (spec 2.1), stdlib only: just what osm_buildings.py needs.

A tile is a protobuf ``Tile`` of ``Layer`` messages; each layer carries its features, the
key and value tables their tags index into, and the extent of its integer coordinate grid.
Feature geometry is a stream of MoveTo / LineTo / ClosePath commands with zigzag-encoded
deltas; a polygon's exterior rings have a positive surveyor's-formula area in tile
coordinates (y down), its holes a negative one.
https://github.com/mapbox/vector-tile-spec/tree/master/2.1
"""

from __future__ import annotations

import struct
from collections.abc import Iterator
from dataclasses import dataclass, field

GEOM_POLYGON = 3

_CMD_MOVE_TO = 1
_CMD_LINE_TO = 2
_CMD_CLOSE_PATH = 7


@dataclass
class Feature:
    id: int | None
    type: int  # 1 point, 2 linestring, 3 polygon
    properties: dict[str, object]
    geometry: list[int]


@dataclass
class Layer:
    name: str
    extent: int = 4096
    features: list[Feature] = field(default_factory=list)


def _varint(buf: bytes, pos: int) -> tuple[int, int]:
    result = 0
    shift = 0
    while True:
        if pos >= len(buf):
            raise ValueError("truncated varint")
        byte = buf[pos]
        pos += 1
        result |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return result, pos
        shift += 7
        if shift > 63:
            raise ValueError("varint too long")


def _fields(buf: bytes) -> Iterator[tuple[int, int, int | bytes]]:
    """(field number, wire type, value) for each field of one message."""
    pos = 0
    while pos < len(buf):
        key, pos = _varint(buf, pos)
        number, wire = key >> 3, key & 7
        if wire == 0:
            value, pos = _varint(buf, pos)
            yield number, wire, value
        elif wire == 1:
            if pos + 8 > len(buf):
                raise ValueError("truncated fixed64")
            yield number, wire, buf[pos : pos + 8]
            pos += 8
        elif wire == 2:
            length, pos = _varint(buf, pos)
            if pos + length > len(buf):
                raise ValueError("truncated field")
            yield number, wire, buf[pos : pos + length]
            pos += length
        elif wire == 5:
            if pos + 4 > len(buf):
                raise ValueError("truncated fixed32")
            yield number, wire, buf[pos : pos + 4]
            pos += 4
        else:
            raise ValueError(f"unsupported wire type {wire}")


def _int(value: int | bytes) -> int:
    if not isinstance(value, int):
        raise ValueError("expected a varint field")
    return value


def _bytes(value: int | bytes) -> bytes:
    if not isinstance(value, bytes):
        raise ValueError("expected a length-delimited field")
    return value


def _uints(value: int | bytes) -> list[int]:
    """A repeated uint32 field: packed (one length-delimited run) or one varint at a time."""
    if isinstance(value, int):
        return [value]
    out = []
    pos = 0
    data = value
    while pos < len(data):
        number, pos = _varint(data, pos)
        out.append(number)
    return out


def _zigzag(n: int) -> int:
    return (n >> 1) ^ -(n & 1)


def _value(buf: bytes) -> object:
    for number, wire, raw in _fields(buf):
        if number == 1:
            return _bytes(raw).decode("utf-8")
        if number == 2 and wire == 5:
            return struct.unpack("<f", _bytes(raw))[0]
        if number == 3 and wire == 1:
            return struct.unpack("<d", _bytes(raw))[0]
        if number == 4:
            value = _int(raw)
            return value - (1 << 64) if value >= 1 << 63 else value
        if number == 5:
            return _int(raw)
        if number == 6:
            return _zigzag(_int(raw))
        if number == 7:
            return bool(_int(raw))
    return None


def _layer(buf: bytes) -> Layer:
    layer = Layer(name="")
    keys: list[str] = []
    values: list[object] = []
    raw_features: list[tuple[int | None, int, list[int], list[int]]] = []
    for number, _, raw in _fields(buf):
        if number == 1:
            layer.name = _bytes(raw).decode("utf-8")
        elif number == 2:
            fid: int | None = None
            ftype = 0
            tags: list[int] = []
            geometry: list[int] = []
            for f_number, _, f_raw in _fields(_bytes(raw)):
                if f_number == 1:
                    fid = _int(f_raw)
                elif f_number == 2:
                    tags.extend(_uints(f_raw))
                elif f_number == 3:
                    ftype = _int(f_raw)
                elif f_number == 4:
                    geometry.extend(_uints(f_raw))
            raw_features.append((fid, ftype, tags, geometry))
        elif number == 3:
            keys.append(_bytes(raw).decode("utf-8"))
        elif number == 4:
            values.append(_value(_bytes(raw)))
        elif number == 5:
            layer.extent = _int(raw)
    # Tags index the key and value tables, which may come after the features on the wire.
    for fid, ftype, tags, geometry in raw_features:
        properties: dict[str, object] = {}
        for k in range(0, len(tags) - 1, 2):
            if tags[k] < len(keys) and tags[k + 1] < len(values):
                properties[keys[tags[k]]] = values[tags[k + 1]]
        layer.features.append(Feature(fid, ftype, properties, geometry))
    return layer


def decode_layers(data: bytes) -> dict[str, Layer]:
    """Every layer of one (uncompressed) tile, by name."""
    layers = {}
    for number, _, raw in _fields(data):
        if number == 3:
            layer = _layer(_bytes(raw))
            layers[layer.name] = layer
    return layers


def polygon_rings(geometry: list[int]) -> list[list[tuple[int, int]]]:
    """A polygon feature's rings in tile coordinates, exterior and holes alike (see ring_area)."""
    rings: list[list[tuple[int, int]]] = []
    current: list[tuple[int, int]] = []
    x = y = 0
    i = 0
    while i < len(geometry):
        command, count = geometry[i] & 7, geometry[i] >> 3
        i += 1
        if command == _CMD_CLOSE_PATH:
            if len(current) >= 3:
                rings.append(current)
            current = []
            continue
        if command not in (_CMD_MOVE_TO, _CMD_LINE_TO) or i + 2 * count > len(geometry):
            raise ValueError("bad geometry command")
        for _ in range(count):
            x += _zigzag(geometry[i])
            y += _zigzag(geometry[i + 1])
            i += 2
            if command == _CMD_MOVE_TO:
                current = []
            current.append((x, y))
    return rings


def ring_area(ring: list[tuple[int, int]] | list[tuple[float, float]]) -> float:
    """Surveyor's-formula area in tile coordinates: > 0 for an exterior ring, < 0 for a hole."""
    total = 0.0
    for i, (x0, y0) in enumerate(ring):
        x1, y1 = ring[(i + 1) % len(ring)]
        total += x0 * y1 - x1 * y0
    return total / 2.0

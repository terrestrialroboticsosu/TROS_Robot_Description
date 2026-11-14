"""Plan a drivable route over a DEM and pack it small enough for a radio link.

This runs on the base station, not the rover. It needs the DEM (a GeoTIFF),
the start point and the target coordinates. It finds the route to each target
that keeps the rover's pitch and roll inside set limits, reduces it to a short
list of waypoints, and writes them as a packet of about 5 bytes per waypoint.
The rover only has to receive that packet and drive waypoint to waypoint, so
the DEM itself never crosses the radio.

    ros2 run tros_sldworks_pkg plan_route ~/Downloads/output.tin.tif
    ros2 run tros_sldworks_pkg plan_route DEM.tif --max-pitch 20 --max-roll 15

How the route is chosen
- The DEM is resampled to a 1 m grid and lightly smoothed (about the rover's
  footprint), and the ground's gradient is taken at every cell.
- Moving from a cell to a neighbour has a direction, so the gradient splits
  into the part along the move (the rover's pitch) and the part across it (its
  roll). A move is forbidden if either exceeds its limit; otherwise it costs
  its length, plus more the closer pitch and roll are to their limits. This is
  why the planner is not Nav2's: a Nav2 costmap holds one number per cell and
  cannot tell climbing a slope from driving across it.
- GPS puts the rover a metre or two from where it thinks it is, so ground
  steeper than both limits is avoided by `clearance` metres, and being near
  steep ground costs extra.
- The cheapest path is found with Dijkstra's algorithm, then cut down to
  waypoints: a run of path cells is replaced by a straight segment whenever
  that segment is itself within limits and not much costlier.

Packet format (little-endian), decoded by `decode_packet`:
    'TR', version u8, legs u8, origin latitude i32, origin longitude i32
    (1e-7 degrees), then per leg: waypoint count u8 and for each waypoint
    east i16, north i16 in decimetres from the origin; CRC-16/CCITT u16 last.
"""
import argparse
import math
import os
import struct

import numpy as np
import yaml

EARTH_RADIUS_M = 6378137.0
PACKET_VERSION = 1
# 8 neighbours: (row step, column step). Row 0 is the north edge.
MOVES = [(-1, 0), (1, 0), (0, 1), (0, -1), (-1, 1), (-1, -1), (1, 1), (1, -1)]


# ----------------------------------------------------------------- packet
def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc


def offset_m(lat, lon, lat0, lon0):
    """East/north metres from (lat0, lon0) to (lat, lon)."""
    return (math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0)),
            math.radians(lat - lat0) * EARTH_RADIUS_M)


def shift(lat0, lon0, east, north):
    """The lat/lon that is east/north metres from (lat0, lon0)."""
    return (lat0 + math.degrees(north / EARTH_RADIUS_M),
            lon0 + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(lat0)))))


def encode_packet(legs):
    """Pack legs (lists of (lat, lon)) into bytes."""
    lat0, lon0 = legs[0][0]
    lat0_i, lon0_i = round(lat0 * 1e7), round(lon0 * 1e7)
    lat0, lon0 = lat0_i / 1e7, lon0_i / 1e7
    out = struct.pack('<2sBBii', b'TR', PACKET_VERSION, len(legs), lat0_i, lon0_i)
    for leg in legs:
        out += struct.pack('<B', len(leg))
        for lat, lon in leg:
            east, north = offset_m(lat, lon, lat0, lon0)
            out += struct.pack('<hh', round(east * 10), round(north * 10))
    return out + struct.pack('<H', crc16(out))


def decode_packet(data):
    """Unpack bytes from `encode_packet` back into legs of (lat, lon)."""
    if len(data) < 14 or crc16(data[:-2]) != struct.unpack('<H', data[-2:])[0]:
        raise ValueError('route packet is damaged (checksum does not match)')
    magic, version, n_legs, lat0_i, lon0_i = struct.unpack_from('<2sBBii', data, 0)
    if magic != b'TR' or version != PACKET_VERSION:
        raise ValueError('not a route packet this version understands')
    lat0, lon0 = lat0_i / 1e7, lon0_i / 1e7
    pos, legs = 12, []
    for _ in range(n_legs):
        count = data[pos]
        pos += 1
        leg = []
        for _ in range(count):
            east, north = struct.unpack_from('<hh', data, pos)
            pos += 4
            leg.append(shift(lat0, lon0, east / 10.0, north / 10.0))
        legs.append(leg)
    return legs


# ---------------------------------------------------------------- terrain
class Terrain:
    """A 1 m height grid cut from the DEM, with its gradient."""

    def __init__(self, dem_path, points_ll, margin_m=150.0):
        from osgeo import gdal, osr
        from scipy.ndimage import uniform_filter
        gdal.UseExceptions()
        source = gdal.Open(os.path.expanduser(dem_path))
        srs = osr.SpatialReference(wkt=source.GetProjection())
        srs.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
        wgs84 = osr.SpatialReference()
        wgs84.ImportFromEPSG(4326)
        wgs84.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
        self._to_dem = osr.CoordinateTransformation(wgs84, srs)
        self._to_ll = osr.CoordinateTransformation(srs, wgs84)

        en = [self._to_dem.TransformPoint(lon, lat)[:2] for lat, lon in points_ll]
        self.west = math.floor(min(e for e, _ in en) - margin_m)
        self.north = math.ceil(max(n for _, n in en) + margin_m)
        east = math.ceil(max(e for e, _ in en) + margin_m)
        south = math.floor(min(n for _, n in en) - margin_m)
        tile = gdal.Warp('', source, format='MEM', resampleAlg='average', xRes=1.0, yRes=1.0,
                         dstNodata=-9999, outputBounds=[self.west, south, east, self.north])
        band = tile.GetRasterBand(1)
        self.missing = band.ReadAsArray() == -9999
        gdal.FillNodata(band, None, maxSearchDist=200, smoothingIterations=0)
        height = band.ReadAsArray().astype(np.float64)
        self.height = uniform_filter(height, 3, mode='nearest')
        d_row, d_col = np.gradient(self.height, 1.0)
        self.g_east, self.g_north = d_col, -d_row      # rise per metre east / north
        self.slope = np.degrees(np.arctan(np.hypot(self.g_east, self.g_north)))
        self.shape = self.height.shape

    def cell(self, lat, lon):
        e, n = self._to_dem.TransformPoint(lon, lat)[:2]
        return int(self.north - n), int(e - self.west)

    def latlon(self, row, col):
        lon, lat = self._to_ll.TransformPoint(self.west + col + 0.5, self.north - row - 0.5)[:2]
        return lat, lon


def pitch_roll(terrain, row, col, d_row, d_col):
    """Pitch and roll in degrees for a rover at a cell heading in a direction."""
    length = math.hypot(d_row, d_col)
    east, north = d_col / length, -d_row / length
    g_e, g_n = terrain.g_east[row, col], terrain.g_north[row, col]
    return (np.degrees(np.arctan(np.abs(g_e * east + g_n * north))),
            np.degrees(np.arctan(np.abs(g_e * north - g_n * east))))


class Planner:

    def __init__(self, terrain, max_pitch, max_roll, clearance):
        from scipy.ndimage import maximum_filter
        self.t = terrain
        self.max_pitch, self.max_roll = max_pitch, max_roll
        # Too steep to be on at all, whichever way the rover points.
        r = int(math.ceil(clearance))
        yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
        disc = np.hypot(yy, xx) <= clearance
        self.steep_near = maximum_filter(terrain.slope, footprint=disc, mode='nearest')
        self.blocked = self.steep_near >= max(max_pitch, max_roll)

    def move_cost(self, d_row, d_col, blocked):
        """Cost of leaving every cell in one direction (inf where forbidden)."""
        rows, cols = self.t.shape
        r, c = np.mgrid[0:rows, 0:cols]
        pitch, roll = pitch_roll(self.t, r, c, d_row, d_col)
        near = self.steep_near / max(self.max_pitch, self.max_roll)
        cost = math.hypot(d_row, d_col) * (
            1.0 + 4.0 * (pitch / self.max_pitch) ** 2 + 4.0 * (roll / self.max_roll) ** 2
            + 2.0 * near ** 2)
        cost[(pitch > self.max_pitch) | (roll > self.max_roll) | blocked] = np.inf
        return cost

    def plan(self, start, goal):
        """Cheapest list of (row, col) from start to goal, or None."""
        from scipy.sparse import coo_matrix
        from scipy.sparse.csgraph import dijkstra
        rows, cols = self.t.shape
        blocked = self.blocked.copy()
        opened = []
        for name, (r, c) in (('start', start), ('target', goal)):
            if blocked[r, c]:
                opened.append(name)
                yy, xx = np.ogrid[0:rows, 0:cols]
                blocked[np.hypot(yy - r, xx - c) <= 4.0] = False
        index = np.arange(rows * cols).reshape(rows, cols)
        src, dst, weight = [], [], []
        for d_row, d_col in MOVES:
            cost = self.move_cost(d_row, d_col, blocked)
            a = index[max(0, -d_row):rows - max(0, d_row), max(0, -d_col):cols - max(0, d_col)]
            b = index[max(0, d_row):rows + min(0, d_row), max(0, d_col):cols + min(0, d_col)]
            ca = cost.ravel()[a.ravel()]
            cb = cost.ravel()[b.ravel()]
            ok = np.isfinite(ca) & np.isfinite(cb)     # the move must be fine at both ends
            src.append(a.ravel()[ok])
            dst.append(b.ravel()[ok])
            weight.append((ca[ok] + cb[ok]) / 2)
        graph = coo_matrix((np.concatenate(weight), (np.concatenate(src), np.concatenate(dst))),
                           shape=(rows * cols, rows * cols)).tocsr()
        s, g = index[start], index[goal]
        dist, before = dijkstra(graph, indices=s, return_predecessors=True)
        if not np.isfinite(dist[g]):
            return None, opened
        path = [g]
        while path[-1] != s:
            path.append(before[path[-1]])
        return [divmod(int(i), cols) for i in reversed(path)], opened

    def segment(self, a, b):
        """(cost, max pitch, max roll, blocked?) of the straight line a -> b."""
        (r0, c0), (r1, c1) = a, b
        length = math.hypot(r1 - r0, c1 - c0)
        if length == 0:
            return 0.0, 0.0, 0.0, False
        n = max(2, int(length / 0.5) + 1)
        rr = np.clip(np.round(np.linspace(r0, r1, n)).astype(int), 0, self.t.shape[0] - 1)
        cc = np.clip(np.round(np.linspace(c0, c1, n)).astype(int), 0, self.t.shape[1] - 1)
        pitch, roll = pitch_roll(self.t, rr, cc, r1 - r0, c1 - c0)
        near = self.steep_near[rr, cc] / max(self.max_pitch, self.max_roll)
        cost = length * float(np.mean(
            1.0 + 4.0 * (pitch / self.max_pitch) ** 2 + 4.0 * (roll / self.max_roll) ** 2
            + 2.0 * near ** 2))
        return cost, float(pitch.max()), float(roll.max()), bool(self.blocked[rr, cc].any())

    def simplify(self, path, max_segment_m=20.0):
        """Reduce a cell path to waypoints joined by acceptable straight lines."""
        step = [self.segment(path[i], path[i + 1])[0] for i in range(len(path) - 1)]
        along = np.concatenate([[0.0], np.cumsum(step)])
        keep, i = [0], 0
        while i < len(path) - 1:
            best = i + 1
            for j in range(i + 2, len(path)):
                if math.hypot(path[j][0] - path[i][0], path[j][1] - path[i][1]) > max_segment_m:
                    break
                cost, pitch, roll, blocked = self.segment(path[i], path[j])
                if (not blocked and pitch <= self.max_pitch and roll <= self.max_roll
                        and cost <= 1.15 * (along[j] - along[i]) + 0.5):
                    best = j
            keep.append(best)
            i = best
        return [path[k] for k in keep]

    def describe(self, points):
        """Length and worst pitch/roll along a polyline of cells."""
        length, pitch, roll = 0.0, 0.0, 0.0
        for a, b in zip(points[:-1], points[1:]):
            _, p, r, _ = self.segment(a, b)
            length += math.hypot(b[0] - a[0], b[1] - a[1])
            pitch, roll = max(pitch, p), max(roll, r)
        return length, pitch, roll


def trim_end(path, metres):
    """Drop the last `metres` of a cell path."""
    travelled = 0.0
    for i in range(len(path) - 1, 0, -1):
        travelled += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        if travelled >= metres:
            return path[:i]
    return path[:1]


def draw_preview(terrain, planner, legs_cells, straight, targets, path):
    """Save a slope map with the planned route and the straight lines."""
    from PIL import Image, ImageDraw
    scale = 3
    s = np.clip(terrain.slope / max(planner.max_pitch, planner.max_roll), 0, 1)
    rgb = np.stack([120 + 135 * s, 200 - 150 * s, 110 - 80 * s], axis=2)
    rgb[planner.blocked] = (90, 20, 90)
    image = Image.fromarray(rgb.astype(np.uint8)).resize(
        (terrain.shape[1] * scale, terrain.shape[0] * scale), Image.NEAREST)
    draw = ImageDraw.Draw(image)
    xy = lambda cell: ((cell[1] + 0.5) * scale, (cell[0] + 0.5) * scale)   # noqa: E731
    for a, b in straight:
        draw.line([xy(a), xy(b)], fill=(255, 255, 255), width=1)
    for cells in legs_cells:
        draw.line([xy(c) for c in cells], fill=(0, 60, 255), width=3)
        for c in cells:
            x, y = xy(c)
            draw.ellipse([x - 4, y - 4, x + 4, y + 4], fill=(255, 255, 0), outline=(0, 0, 0))
    for c in targets:
        x, y = xy(c)
        draw.rectangle([x - 6, y - 6, x + 6, y + 6], outline=(0, 0, 0), width=3)
    image.save(path)


def main():
    parser = argparse.ArgumentParser(description='Plan a slope-limited route over a DEM.')
    share = None
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('tros_sldworks_pkg')
    except Exception:
        pass
    default_mission = os.path.join(share, 'config', 'mission_2027.yaml') if share else None
    parser.add_argument('dem', help='GeoTIFF DEM in a projected (metre) coordinate system')
    parser.add_argument('--mission', default=default_mission,
                        help='mission file with route_finding start and targets')
    parser.add_argument('--out', default='route.yaml', help='route file to write')
    parser.add_argument('--max-pitch', type=float, default=20.0, help='degrees')
    parser.add_argument('--max-roll', type=float, default=15.0, help='degrees')
    parser.add_argument('--clearance', type=float, default=2.0,
                        help='metres to stay away from ground steeper than both limits')
    parser.add_argument('--standoff', type=float, default=2.5,
                        help='stop this many metres short of each target')
    args = parser.parse_args()

    with open(args.mission) as f:
        route = yaml.safe_load(f)['route_finding']
    stops = [('start', route['start']['lat'], route['start']['lon'])]
    stops += [(t['model'], t['lat'], t['lon']) for t in route['targets']]

    terrain = Terrain(args.dem, [(lat, lon) for _, lat, lon in stops])
    planner = Planner(terrain, args.max_pitch, args.max_roll, args.clearance)
    print(f'DEM cut to {terrain.shape[1]} x {terrain.shape[0]} m at 1 m; '
          f'{terrain.missing.mean():.1%} had no data and was filled')
    print(f'Limits: pitch {args.max_pitch:.0f} deg, roll {args.max_roll:.0f} deg, '
          f'clearance {args.clearance:.1f} m; {planner.blocked.mean():.1%} of the area is off limits')

    legs, legs_cells, straight, report = [], [], [], []
    here = terrain.cell(stops[0][1], stops[0][2])
    for name, lat, lon in stops[1:]:
        target = terrain.cell(lat, lon)
        path, opened = planner.plan(here, target)
        if path is None:
            raise SystemExit(f'No route to {name} within the limits. Raise --max-pitch / '
                             '--max-roll or lower --clearance.')
        for end in opened:
            print(f'  note: the {end} of the leg to {name} is on or beside ground over the '
                  'limits; the last 4 m around it were allowed anyway')
        cells = planner.simplify(trim_end(path, args.standoff))
        length, pitch, roll = planner.describe(cells)
        s_length, s_pitch, s_roll = planner.describe([here, target])
        print(f'{name}: {len(cells)} waypoints, {length:.0f} m, worst pitch {pitch:.0f} deg, '
              f'roll {roll:.0f} deg   (straight line: {s_length:.0f} m, pitch {s_pitch:.0f}, '
              f'roll {s_roll:.0f})')
        leg = [terrain.latlon(r, c) for r, c in cells[1:]]   # the rover is already at cells[0]
        if not leg:
            leg = [terrain.latlon(*cells[0])]
        legs.append(leg)
        legs_cells.append(cells)
        straight.append((here, target))
        report.append({'target': name, 'length_m': round(length, 1),
                       'max_pitch_deg': round(pitch, 1), 'max_roll_deg': round(roll, 1),
                       'straight_line': {'length_m': round(s_length, 1),
                                         'max_pitch_deg': round(s_pitch, 1),
                                         'max_roll_deg': round(s_roll, 1)},
                       'waypoints': [[round(a, 7), round(b, 7)] for a, b in leg]})
        # The next leg starts at the post, not where this leg's waypoints end:
        # the tag approach drives the rover the rest of the way in, so that is
        # where it will be. Starting from the last waypoint left an unplanned
        # stretch from the post back to it.
        here = target

    packet = encode_packet(legs)
    assert len(decode_packet(packet)) == len(legs)
    with open(args.out, 'w') as f:
        f.write('# Written by plan_route. The rover drives packet_hex (what the radio would\n'
                '# carry); the legs below are the same route in readable form.\n')
        yaml.safe_dump({'limits': {'max_pitch_deg': args.max_pitch,
                                   'max_roll_deg': args.max_roll,
                                   'clearance_m': args.clearance,
                                   'standoff_m': args.standoff},
                        'packet_bytes': len(packet), 'packet_hex': packet.hex(),
                        'legs': report}, f, sort_keys=False)
    preview = os.path.splitext(args.out)[0] + '_preview.png'
    draw_preview(terrain, planner, legs_cells, straight,
                 [terrain.cell(lat, lon) for _, lat, lon in stops[1:]], preview)
    print(f'Packet: {len(packet)} bytes for {sum(len(leg) for leg in legs)} waypoints')
    print(f'Wrote {args.out} and {preview}')


if __name__ == '__main__':
    main()

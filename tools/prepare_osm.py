#!/usr/bin/env python3
"""
Підготовка даних карти для «Наві» з OpenStreetMap.

Вхід:  ukraine-latest.osm.pbf (Geofabrik, ~1 ГБ) — скрипт сам завантажить, якщо файлу немає.
Вихід: data/graph.bin  — граф доріг у компактному бінарному форматі (див. graph_bin.py)
       data/fuel.json  — усі АЗС (amenity=fuel) з брендом, назвою, видами пального

Запуск (Windows/Mac/Linux, потрібен Python 3.9+):
    pip install osmium
    python tools/prepare_osm.py                 # Київ + Київська область (типово)
    python tools/prepare_osm.py --bbox 50.2,30.2,50.65,30.95   # лише Київ (швидше, менше)

Пам'ять: ~3–5 ГБ ОЗП для файлу всієї України (використовується індекс вузлів у пам'яті).
Якщо пам'яті мало — додайте --disk-index (повільніше, але економно).
"""
import argparse, json, math, os, sys, time, urllib.request

try:
    import osmium
except ImportError:
    print("Потрібен пакет osmium:  pip install osmium"); sys.exit(1)

R = 6371008.8
D2R = math.pi / 180

# Класи доріг, якими їздить авто
HIGHWAYS = {
    'motorway', 'motorway_link', 'trunk', 'trunk_link', 'primary', 'primary_link',
    'secondary', 'secondary_link', 'tertiary', 'tertiary_link', 'unclassified',
    'residential', 'living_street', 'service', 'road',
}
SERVICE_SKIP = {'parking_aisle', 'driveway', 'drive-through', 'emergency_access'}


class Collector(osmium.SimpleHandler):
    def __init__(self, bbox, lat0, lon0):
        super().__init__()
        self.bbox = bbox  # (minlat, minlon, maxlat, maxlon)
        self.lat0, self.lon0 = lat0, lon0
        self.ways = []          # (node_ids, oneway, name)
        self.coords = {}        # node id -> (x, y)
        self.use = {}           # node id -> кількість використань
        self.fuel = []
        self.buildings = []     # (x, y, street, housenumber, outline[(x,y)...])
        self.entrances = []     # (x, y, ref, building_hint)
        self.n_ways = 0

    def inside(self, lat, lon):
        b = self.bbox
        return b[0] <= lat <= b[2] and b[1] <= lon <= b[3]

    def xy(self, lat, lon):
        return ((lon - self.lon0) * D2R * R * math.cos(lat * D2R), (lat - self.lat0) * D2R * R)

    def node(self, n):
        self.n_nodes = getattr(self, 'n_nodes', 0) + 1
        if self.n_nodes % 5000000 == 0:
            print(f'  точок прочитано: {self.n_nodes // 1000000} млн, у межах області: будинків {len(self.buildings)}, під\'їздів {len(self.entrances)}', flush=True)
        t = n.tags
        if not n.location.valid() or not self.inside(n.location.lat, n.location.lon):
            return
        if t.get('amenity') == 'fuel':
            self.fuel.append(self.fuel_rec('n%d' % n.id, t, n.location.lat, n.location.lon))
        ent = t.get('entrance')
        if ent and ent not in ('no', 'emergency', 'service', 'garage'):
            x, y = self.xy(n.location.lat, n.location.lon)
            ref = t.get('ref') or t.get('addr:unit') or ''
            self.entrances.append((round(x, 1), round(y, 1), ref[:8]))
        elif t.get('addr:housenumber') and t.get('addr:street') and t.get('building') is None:
            # адресна точка без контуру (часто в селах)
            x, y = self.xy(n.location.lat, n.location.lon)
            self.buildings.append((round(x, 1), round(y, 1), t.get('addr:street'), t.get('addr:housenumber'), []))

    def fuel_rec(self, fid, tags, lat, lon):
        x, y = self.xy(lat, lon)
        fuels = []
        for t in tags:  # pyosmium віддає об'єкти Tag з полями k, v
            k, v = (t.k, t.v) if hasattr(t, 'k') else (t, tags[t])
            if k.startswith('fuel:') and v == 'yes':
                fuels.append(k[5:])
        fuels.sort()
        return {
            'id': fid, 'x': round(x, 1), 'y': round(y, 1), 'lat': round(lat, 6), 'lon': round(lon, 6),
            'brand': tags.get('brand') or tags.get('operator') or '',
            'name': tags.get('name') or tags.get('brand') or tags.get('operator') or 'АЗС',
            'fuel': fuels, 'hours': tags.get('opening_hours', ''),
        }

    def way(self, w):
        tags = w.tags
        if tags.get('amenity') == 'fuel':
            pts = [(n.lat, n.lon) for n in w.nodes if n.location.valid()]
            if pts:
                lat = sum(p[0] for p in pts) / len(pts); lon = sum(p[1] for p in pts) / len(pts)
                if self.inside(lat, lon):
                    self.fuel.append(self.fuel_rec('w%d' % w.id, tags, lat, lon))
            return
        if tags.get('building') is not None or tags.get('addr:housenumber'):
            pts = [self.xy(n.lat, n.lon) for n in w.nodes if n.location.valid() and self.inside(n.lat, n.lon)]
            if len(pts) >= 3 or (pts and tags.get('addr:housenumber')):
                cx = sum(p[0] for p in pts) / len(pts); cy = sum(p[1] for p in pts) / len(pts)
                outline = [(round(p[0], 1), round(p[1], 1)) for p in pts] if len(pts) >= 3 else []
                if len(outline) > 1 and outline[0] == outline[-1]: outline.pop()
                if len(outline) > 24: outline = [outline[i] for i in range(0, len(outline), max(1, len(outline) // 24))]
                self.buildings.append((round(cx, 1), round(cy, 1), tags.get('addr:street', ''), tags.get('addr:housenumber', ''), outline))
            if tags.get('highway') is None:
                return
        hw = tags.get('highway')
        if hw not in HIGHWAYS:
            return
        if hw == 'service' and tags.get('service') in SERVICE_SKIP:
            return
        if tags.get('access') in ('no', 'private') and tags.get('motor_vehicle') not in ('yes', 'designated'):
            return
        if tags.get('motor_vehicle') == 'no' or tags.get('motorcar') == 'no':
            return
        ids = []
        for n in w.nodes:
            if not n.location.valid():
                continue
            if not self.inside(n.lat, n.lon):
                ids.append(None); continue
            if n.ref not in self.coords:
                self.coords[n.ref] = self.xy(n.lat, n.lon)
            ids.append(n.ref)
        # розрізати по частинах поза bbox
        parts, cur = [], []
        for i in ids:
            if i is None:
                if len(cur) > 1: parts.append(cur)
                cur = []
            else:
                cur.append(i)
        if len(cur) > 1: parts.append(cur)
        if not parts:
            return
        ow = tags.get('oneway', '')
        oneway = ow in ('yes', 'true', '1') or tags.get('junction') in ('roundabout', 'circular') or hw == 'motorway'
        reverse = ow == '-1'
        name = tags.get('name:uk') or tags.get('name') or ''
        for p in parts:
            if reverse: p = p[::-1]
            self.ways.append((p, oneway or reverse, name, hw))
            for i in p: self.use[i] = self.use.get(i, 0) + 1
            self.use[p[0]] += 1; self.use[p[-1]] += 1  # кінці завжди вузли графа
        self.n_ways += 1
        if self.n_ways % 50000 == 0:
            print(f'  доріг: {self.n_ways}, АЗС: {len(self.fuel)}, будинків: {len(self.buildings)}, під\'їздів: {len(self.entrances)}', flush=True)


def simplify(points, tol):
    """Дуглас–Пекер: лишає вузли, без яких ламана відхиляється понад tol метрів."""
    if len(points) < 3:
        return list(range(len(points)))
    keep = [0, len(points) - 1]
    stack = [(0, len(points) - 1)]
    while stack:
        a, b = stack.pop()
        ax, ay = points[a]; bx, by = points[b]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        best, bd = -1, tol
        for i in range(a + 1, b):
            px, py = points[i]
            t = 0 if L2 == 0 else max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / L2))
            d = math.hypot(px - (ax + dx * t), py - (ay + dy * t))
            if d > bd: bd, best = d, i
        if best >= 0:
            keep.append(best); stack.append((a, best)); stack.append((best, b))
    return sorted(set(keep))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pbf', default='ukraine-latest.osm.pbf')
    ap.add_argument('--bbox', default='49.17,29.25,51.56,32.20', help='minlat,minlon,maxlat,maxlon (типово: Київська область)')
    ap.add_argument('--origin', default='50.4501,30.5234', help='опорна точка lat,lon (центр Києва)')
    ap.add_argument('--out', default='data')
    ap.add_argument('--tol', type=float, default=2.5, help='спрощення геометрії, м')
    ap.add_argument('--disk-index', action='store_true', help='індекс вузлів на диску замість ОЗП')
    ap.add_argument('--name', default='Київ та область')
    ap.add_argument('--json', action='store_true', help='додатково записати graph.json (великий, для налагодження)')
    a = ap.parse_args()

    if not os.path.exists(a.pbf):
        url = 'https://download.geofabrik.de/europe/ukraine-latest.osm.pbf'
        print(f'Завантажую {url} (~1 ГБ)…')
        def hook(b, bs, total):
            if b % 200 == 0: print(f'\r  {b * bs / 1e6:.0f} / {total / 1e6:.0f} МБ', end='', flush=True)
        urllib.request.urlretrieve(url, a.pbf, hook); print()

    bbox = tuple(float(v) for v in a.bbox.split(','))
    lat0, lon0 = (float(v) for v in a.origin.split(','))
    os.makedirs(a.out, exist_ok=True)

    t0 = time.time()
    print('Читаю PBF (це займе кілька хвилин)…', flush=True)
    h = Collector(bbox, lat0, lon0)
    idx = 'sparse_file_array,nodes.cache' if a.disk_index else 'flex_mem'
    h.apply_file(a.pbf, locations=True, idx=idx)
    print(f'Прочитано за {time.time() - t0:.0f} с: доріг {len(h.ways)}, вузлів-точок {len(h.coords)}, АЗС {len(h.fuel)}, будинків {len(h.buildings)}, під\'їздів {len(h.entrances)}')

    # Побудова графа: вузли графа = перехрестя (use>=2) + кінці + точки, потрібні для геометрії
    node_index = {}
    nodes = []
    edges = []
    def gid(ref):
        i = node_index.get(ref)
        if i is None:
            x, y = h.coords[ref]; i = len(nodes); node_index[ref] = i; nodes.append([round(x, 1), round(y, 1)])
        return i
    for refs, oneway, name, hw in h.ways:
        # розбити на відрізки між перехрестями, кожен спростити
        cuts = [0] + [i for i in range(1, len(refs) - 1) if h.use[refs[i]] >= 2] + [len(refs) - 1]
        for ci in range(len(cuts) - 1):
            seg = refs[cuts[ci]:cuts[ci + 1] + 1]
            pts = [h.coords[r] for r in seg]
            keep = simplify(pts, a.tol)
            for k in range(len(keep) - 1):
                ra, rb = seg[keep[k]], seg[keep[k + 1]]
                if ra == rb: continue
                edges.append({'a': gid(ra), 'b': gid(rb), 'oneway': oneway, 'name': name, 'cls': hw.replace('_link', '')})
    # прибрати дублікати АЗС (вузол усередині полігона тієї ж станції)
    fuel, seen = [], []
    for f in sorted(h.fuel, key=lambda f: f['id'][0] != 'w'):
        if any(math.hypot(f['x'] - s['x'], f['y'] - s['y']) < 40 for s in seen): continue
        seen.append(f); fuel.append(f)

    graph = {'origin': [lat0, lon0], 'bbox': list(bbox), 'nodes': nodes, 'edges': edges,
             'meta': {'name': a.name, 'source': 'OpenStreetMap', 'built': time.strftime('%Y-%m-%d')}}
    # основний формат — компактний graph.bin (застосунок читає саме його); JSON — лише за бажанням
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from graph_bin import write_bin, write_buildings
    write_bin(graph, os.path.join(a.out, 'graph.bin'))
    nb, ne = write_buildings(h.buildings, h.entrances, os.path.join(a.out, 'buildings.bin'))
    print(f'Будинків: {nb} (з адресою {sum(1 for b in h.buildings if b[2] and b[3])}), під\'їздів: {ne} → {a.out}/buildings.bin '
          f'({os.path.getsize(os.path.join(a.out, "buildings.bin")) / 1e6:.1f} МБ)')
    if a.json:
        with open(os.path.join(a.out, 'graph.json'), 'w', encoding='utf-8') as f:
            json.dump(graph, f, ensure_ascii=False, separators=(',', ':'))
    with open(os.path.join(a.out, 'fuel.json'), 'w', encoding='utf-8') as f:
        json.dump(fuel, f, ensure_ascii=False, separators=(',', ':'))
    print(f'Готово: {len(nodes)} вузлів, {len(edges)} ребер, {len(fuel)} АЗС → {a.out}/graph.bin '
          f'({os.path.getsize(os.path.join(a.out, "graph.bin")) / 1e6:.1f} МБ), {a.out}/fuel.json')


if __name__ == '__main__':
    main()

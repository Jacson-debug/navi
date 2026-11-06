#!/usr/bin/env python3
"""
Компактний бінарний формат карти для «Наві» (graph.bin).
Використання: python tools/graph_bin.py data/graph.json data/graph.bin

Формат (little-endian):
  'NAVI' | u32 version=1 | u32 nNodes | u32 nEdges | f64 originLat | f64 originLon | u32 metaLen | meta(JSON, utf-8)
  f32 x[nNodes] | f32 y[nNodes]
  i32 a[nEdges] | i32 b[nEdges] | u8 flags[nEdges] (bit0 = oneway) | u8 cls[nEdges] | i32 nameIdx[nEdges] (-1 = без назви)
meta = {"name", "source", "built", "bbox", "names": [...], "classes": [...]}
"""
import json, struct, sys, array

CLASSES = ['service', 'living_street', 'residential', 'road', 'unclassified', 'tertiary', 'secondary', 'primary', 'trunk', 'motorway']


def write_bin(graph, out_path):
    nodes, edges = graph['nodes'], graph['edges']
    names, name_idx = [], {}
    cls_idx = {c: i for i, c in enumerate(CLASSES)}
    xa, ya = array.array('f'), array.array('f')
    for x, y in nodes:
        xa.append(x); ya.append(y)
    aa, ba, na = array.array('i'), array.array('i'), array.array('i')
    flags, cls = bytearray(), bytearray()
    for e in edges:
        aa.append(e['a']); ba.append(e['b'])
        flags.append(1 if e.get('oneway') else 0)
        cls.append(cls_idx.get(e.get('cls', 'residential'), cls_idx['residential']))
        nm = e.get('name') or ''
        if nm:
            i = name_idx.get(nm)
            if i is None:
                i = len(names); name_idx[nm] = i; names.append(nm)
            na.append(i)
        else:
            na.append(-1)
    meta = dict(graph.get('meta', {}))
    meta.update({'bbox': graph.get('bbox'), 'names': names, 'classes': CLASSES})
    mb = json.dumps(meta, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
    lat0, lon0 = graph['origin']
    with open(out_path, 'wb') as f:
        f.write(b'NAVI'); f.write(struct.pack('<IIIddI', 1, len(nodes), len(edges), lat0, lon0, len(mb))); f.write(mb)
        for arr in (xa, ya, aa, ba):
            f.write(arr.tobytes())
        f.write(bytes(flags)); f.write(bytes(cls)); f.write(na.tobytes())
    return len(nodes), len(edges), len(names)


def write_buildings(buildings, entrances, out_path):
    """
    buildings.bin (little-endian):
      'NAVB' | u32 version=1 | u32 nB | u32 nE | u32 nStreets | u32 streetsLen | streets(JSON list, utf-8)
      будинки: f32 x[nB] | f32 y[nB] | i32 street[nB] (-1 = нема) | u32 numLen | numbers(JSON list, utf-8)
               u32 outlineTotal | u16 outlineCount[nB] | f32 outlineXY[outlineTotal*2]
      під'їзди: f32 x[nE] | f32 y[nE] | u32 refsLen | refs(JSON list, utf-8)
    """
    streets, sidx = [], {}
    xa, ya, st, nums, oc, oxy = array.array('f'), array.array('f'), array.array('i'), [], array.array('H'), array.array('f')
    for x, y, street, num, outline in buildings:
        xa.append(x); ya.append(y)
        if street:
            i = sidx.get(street)
            if i is None: i = len(streets); sidx[street] = i; streets.append(street)
            st.append(i)
        else:
            st.append(-1)
        nums.append(num or '')
        oc.append(len(outline))
        for px, py in outline: oxy.append(px); oxy.append(py)
    ex, ey, refs = array.array('f'), array.array('f'), []
    for x, y, ref in entrances:
        ex.append(x); ey.append(y); refs.append(ref)
    sb = json.dumps(streets, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
    nb = json.dumps(nums, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
    rb = json.dumps(refs, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
    with open(out_path, 'wb') as f:
        f.write(b'NAVB'); f.write(struct.pack('<IIIII', 1, len(xa), len(ex), len(streets), len(sb))); f.write(sb)
        f.write(xa.tobytes()); f.write(ya.tobytes()); f.write(st.tobytes())
        f.write(struct.pack('<I', len(nb))); f.write(nb)
        f.write(struct.pack('<I', len(oxy) // 2)); f.write(oc.tobytes()); f.write(oxy.tobytes())
        f.write(ex.tobytes()); f.write(ey.tobytes()); f.write(struct.pack('<I', len(rb))); f.write(rb)
    return len(xa), len(ex)


if __name__ == '__main__':
    src = sys.argv[1] if len(sys.argv) > 1 else 'data/graph.json'
    dst = sys.argv[2] if len(sys.argv) > 2 else 'data/graph.bin'
    g = json.load(open(src, encoding='utf-8'))
    n, m, k = write_bin(g, dst)
    import os
    print(f'{dst}: {n} вузлів, {m} ребер, {k} назв, {os.path.getsize(dst) / 1e6:.1f} МБ')

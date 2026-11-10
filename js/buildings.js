// Будинки, адреси та під'їзди з buildings.bin (див. tools/graph_bin.py → write_buildings).
// Пошук адреси: нормалізація назв вулиць («вул.», «просп.», літера ґ/г, апостроф), пошук за підрядком.

export class Buildings {
  static fromBinary(buf) {
    const dv = new DataView(buf); let o = 0;
    const tag = String.fromCharCode(dv.getUint8(0), dv.getUint8(1), dv.getUint8(2), dv.getUint8(3)); o = 4;
    if (tag !== 'NAVB') throw new Error('не buildings.bin');
    const u32 = () => { const v = dv.getUint32(o, true); o += 4; return v; };
    const take = (T, count) => { const bytes = count * T.BYTES_PER_ELEMENT; const arr = new T(count); new Uint8Array(arr.buffer).set(new Uint8Array(buf, o, bytes)); o += bytes; return arr; };
    const json = (len) => { const v = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, o, len))); o += len; return v; };
    u32(); // version
    const nB = u32(), nE = u32(); u32(); const streets = json(u32());
    const b = new Buildings();
    b.streets = streets;
    b.x = take(Float32Array, nB); b.y = take(Float32Array, nB); b.street = take(Int32Array, nB);
    // 335 тис. рядків-номерів тримати в купі дорого (~15 МБ на iPhone). Різних
    // номерів лише ~7 тис., тому: словник + Uint16-індекс на кожен будинок.
    { const raw = json(u32()); b._dict = []; const map = new Map(); b._numIx = new Uint16Array(raw.length);
      for (let i = 0; i < raw.length; i++) { const v = raw[i] || ''; let ix = map.get(v); if (ix === undefined) { ix = b._dict.length; map.set(v, ix); b._dict.push(v); } b._numIx[i] = ix; } }
    const total = u32(); b.outCount = take(Uint16Array, nB); b.outXY = take(Float32Array, total * 2);
    b.outOff = new Uint32Array(nB + 1); for (let i = 0; i < nB; i++) b.outOff[i + 1] = b.outOff[i] + b.outCount[i];
    b.ex = take(Float32Array, nE); b.ey = take(Float32Array, nE); b.eref = json(u32());
    b._index();
    return b;
  }

  num(i) { return this._dict[this._numIx[i]]; }

  _index() {
    // Сітка 250 м. Замість Map<рядок, масив> — порядок індексів, відсортований за
    // ключем комірки, і бінарний пошук: у 10 разів менше пам'яті, та сама швидкість.
    const cell = this.cell = 250;
    const mk = (xs, ys) => {
      const n = xs.length, keys = new Float64Array(n), order = new Uint32Array(n);
      for (let i = 0; i < n; i++) { keys[i] = this._key(Math.floor(xs[i] / cell), Math.floor(ys[i] / cell)); order[i] = i; }
      order.sort((a, b) => keys[a] - keys[b]);
      const sortedKeys = new Float64Array(n); for (let k = 0; k < n; k++) sortedKeys[k] = keys[order[k]];
      return { order, keys: sortedKeys };
    };
    this.gridB = mk(this.x, this.y); this.gridE = mk(this.ex, this.ey);
    // нормалізовані назви вулиць для пошуку
    this.streetNorm = this.streets.map(normStreet);
    // індекс: вулиця → будинки з номером (CSR)
    const nS = this.streets.length, cnt = new Uint32Array(nS + 1);
    for (let i = 0; i < this.x.length; i++) { const s = this.street[i]; if (s >= 0 && this._numIx[i] !== this._emptyIx()) cnt[s + 1]++; }
    for (let s = 0; s < nS; s++) cnt[s + 1] += cnt[s];
    this.streetOff = cnt; this.streetList = new Uint32Array(cnt[nS]); const fill = new Uint32Array(nS);
    for (let i = 0; i < this.x.length; i++) { const s = this.street[i]; if (s >= 0 && this._numIx[i] !== this._emptyIx()) this.streetList[cnt[s] + fill[s]++] = i; }
  }
  _emptyIx() { if (this._eIx === undefined) { const k = this._dict.indexOf(''); this._eIx = k < 0 ? -1 : k; } return this._eIx; }
  _key(gx, gy) { return (gx + 1e5) * 1e6 + (gy + 1e5); }
  _houses(s) { return this.streetList.subarray(this.streetOff[s], this.streetOff[s + 1]); }
  // перший індекс у keys, де keys[k] >= key
  _lower(keys, key) { let lo = 0, hi = keys.length; while (lo < hi) { const m = (lo + hi) >> 1; if (keys[m] < key) lo = m + 1; else hi = m; } return lo; }

  get count() { return this.x.length; }
  get entranceCount() { return this.ex.length; }

  _inBox(grid, xs, ys, x0, y0, x1, y1, limit) {
    const out = [];
    const gx0 = Math.floor(x0 / this.cell), gx1 = Math.floor(x1 / this.cell), gy0 = Math.floor(y0 / this.cell), gy1 = Math.floor(y1 / this.cell);
    if ((gx1 - gx0 + 1) * (gy1 - gy0 + 1) > 400) return out;
    const { order, keys } = grid;
    for (let gx = gx0; gx <= gx1; gx++) for (let gy = gy0; gy <= gy1; gy++) {
      const key = this._key(gx, gy);
      for (let k = this._lower(keys, key); k < keys.length && keys[k] === key; k++) {
        const i = order[k];
        if (xs[i] >= x0 && xs[i] <= x1 && ys[i] >= y0 && ys[i] <= y1) { out.push(i); if (out.length >= limit) return out; }
      }
    }
    return out;
  }
  buildingsInBox(x0, y0, x1, y1, limit = 4000) { return this._inBox(this.gridB, this.x, this.y, x0, y0, x1, y1, limit); }
  entrancesInBox(x0, y0, x1, y1, limit = 1000) { return this._inBox(this.gridE, this.ex, this.ey, x0, y0, x1, y1, limit); }

  outline(i) { return this.outXY.subarray(this.outOff[i] * 2, this.outOff[i + 1] * 2); }
  label(i) { const s = this.street[i] >= 0 ? this.streets[this.street[i]] : ''; return (s ? shortStreet(s) + ', ' : '') + (this.num(i) || ''); }

  // Найближчий будинок з адресою до точки (м)
  nearest(p, r = 120) {
    let best = -1, bd = r;
    for (const i of this.buildingsInBox(p[0] - r, p[1] - r, p[0] + r, p[1] + r)) {
      if (!this.num(i)) continue;
      const d = Math.hypot(this.x[i] - p[0], this.y[i] - p[1]); if (d < bd) { bd = d; best = i; }
    }
    return best;
  }

  // Пошук «вулиця номер». Повертає до limit результатів {i, text, x, y}; near — для сортування за відстанню
  search(query, near = null, limit = 12) {
    const q = normStreet(query);
    if (q.length < 2) return [];
    // відокремити номер: останній токен із цифрою
    const toks = q.split(' ').filter(Boolean);
    let num = '', streetQ = q;
    if (toks.length > 1 && /\d/.test(toks[toks.length - 1])) { num = toks.pop(); streetQ = toks.join(' '); }
    const streetToks = streetQ.split(' ').filter(t => t.length > 1);
    if (!streetToks.length) return [];
    const res = [];
    for (let s = 0; s < this.streets.length; s++) {
      const n = this.streetNorm[s];
      if (!streetToks.every(t => n.includes(t))) continue;
      const houses = this._houses(s); if (!houses.length) continue;
      if (num) {
        const nn = normNum(num);
        for (const i of houses) { const hn = normNum(this.num(i)); if (hn === nn) res.push({ i, exact: 2 }); else if (hn.startsWith(nn)) res.push({ i, exact: 1 }); }
      } else {
        // без номера — показати вулицю як пункт (перший будинок) + кілька будинків
        res.push({ i: houses[0], exact: 0, streetOnly: true, s, count: houses.length });
      }
      if (res.length > 400) break;
    }
    const dist = (r) => near ? Math.hypot(this.x[r.i] - near[0], this.y[r.i] - near[1]) : 0;
    res.sort((a, b) => (b.exact - a.exact) || (dist(a) - dist(b)));
    return res.slice(0, limit).map(r => ({ i: r.i, x: this.x[r.i], y: this.y[r.i], streetOnly: !!r.streetOnly, count: r.count, text: r.streetOnly ? shortStreet(this.streets[r.s]) + ` (${r.count} буд.)` : this.label(r.i), d: dist(r) }));
  }
}

// Типи вулиць прибираємо з обох боків порівняння. \b у JS не працює з кирилицею,
// тому розбиваємо на слова за Unicode-класами, а не регулярками з межами слів.
const TYPE = new Set(['вулиця', 'вулица', 'вул', 'улица', 'ул', 'проспект', 'просп', 'пр', 'провулок', 'пров',
  'бульвар', 'бул', 'площа', 'пл', 'шосе', 'узвіз', 'набережна', 'наб', 'дорога', 'проїзд', 'тупик', 'алея', 'квартал', 'мікрорайон', 'мкр']);
export function normStreet(s) {
  let t = (s || '').toLowerCase().replace(/[’'`ʼ]/g, '').replace(/ґ/g, 'г').replace(/ё/g, 'е');
  t = t.replace(/пр-?т/g, 'проспект').replace(/б-?р/g, 'бульвар').replace(/м-?н/g, 'мікрорайон');
  t = t.replace(/[^\p{L}\p{N}]+/gu, ' ');
  return t.split(' ').filter(w => w && !TYPE.has(w)).join(' ').trim();
}
function normNum(n) { return (n || '').toLowerCase().replace(/\s+/g, '').replace(/[’'`ʼ]/g, ''); }
export function shortStreet(s) {
  const map = [['вулиця', 'вул.'], ['проспект', 'просп.'], ['провулок', 'пров.'], ['бульвар', 'бул.'], ['площа', 'пл.'], ['набережна', 'наб.']];
  for (const [full, short] of map) {
    s = s.replace(new RegExp('^' + full + '\\s+', 'i'), short + ' ').replace(new RegExp('\\s+' + full + '$', 'i'), ' ' + short);
  }
  return s;
}

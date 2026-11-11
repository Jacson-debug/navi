// A* по орієнтованому графу доріг. Збирається в WebAssembly: Safari на iPhone падає, коли
// JavaScript-оптимізатор (JIT) береться за цей цикл у вигляді JS, а WebAssembly компілюється
// іншим, значно простішим шляхом. Усі дані лежать у лінійній пам'яті, адреси передає JS.
typedef unsigned char u8;
enum { N, M, GEN, HCAP, MAXEXP, FROM, TO, EXPANDED, FOUND, CAPPED, PX, PY, POFF, PLIST, PB, PLEN, PSPD, PG, PPREV, PSTAMP, PCLOSED, PHK, PHV, HDIVBITS, HEAPN, RESUME, BUDGET, BESTBITS, PCLS, PTABLE };

static double *hk; static int *hv; static int hn, hcap;
static inline void push(double key, int val) {
  int i = hn++;
  while (i > 0) { int p = (i - 1) >> 1; if (hk[p] <= key) break; hk[i] = hk[p]; hv[i] = hv[p]; i = p; }
  hk[i] = key; hv[i] = val;
}
static inline int pop(void) {
  int top = hv[0]; int n = --hn;
  if (n > 0) {
    double key = hk[n]; int val = hv[n]; int i = 0;
    for (;;) {
      int l = 2 * i + 1; if (l >= n) break;
      int r = l + 1; if (r < n && hk[r] < hk[l]) l = r;
      if (hk[l] >= key) break;
      hk[i] = hk[l]; hv[i] = hv[l]; i = l;
    }
    hk[i] = key; hv[i] = val;
  }
  return top;
}

// p — адреса блоку параметрів (масив int). Повертає 1 — шлях знайдено, 0 — шляху нема (або стеля),
// 2 — ще не закінчено (викликайте знову з RESUME=1): так пошук можна ділити на короткі кроки.
int astar(int *p) {
  const float *X = (const float *)p[PX], *Y = (const float *)p[PY], *LEN = (const float *)p[PLEN], *SPD = (const float *)p[PSPD];
  const int *OFF = (const int *)p[POFF], *LIST = (const int *)p[PLIST], *B = (const int *)p[PB];
  float *G = (float *)p[PG]; int *PREV = (int *)p[PPREV], *STAMP = (int *)p[PSTAMP], *CLOSED = (int *)p[PCLOSED];
  hk = (double *)p[PHK]; hv = (int *)p[PHV]; hcap = p[HCAP];
  const int gen = p[GEN], from = p[FROM], to = p[TO], maxExp = p[MAXEXP], budget = p[BUDGET];
  union { int i; float f; } hd; hd.i = p[HDIVBITS]; const double hdiv = (double)hd.f;
  const double gx = (double)X[to], gy = (double)Y[to];
  int expanded, found = 0, capped = 0, steps = 0, more = 0;
  if (p[RESUME]) { hn = p[HEAPN]; expanded = p[EXPANDED]; }
  else {
    hn = 0; expanded = 0;
    double dx = (double)X[from] - gx, dy = (double)Y[from] - gy;
    STAMP[from] = gen; G[from] = 0.0f; PREV[from] = -1; push(__builtin_sqrt(dx * dx + dy * dy) / hdiv, from);
  }
  while (hn > 0) {
    int u = pop();
    if (u == to) { found = 1; break; }
    if (CLOSED[u] == gen) continue;
    CLOSED[u] = gen;
    if (++expanded > maxExp) { capped = 1; break; }
    const double gu = (double)G[u];
    for (int k = OFF[u], end = OFF[u + 1]; k < end; k++) {
      int e = LIST[k], nb = B[e];
      if (CLOSED[nb] == gen) continue;
      double ng = gu + (double)LEN[e] / (double)SPD[e];
      if (STAMP[nb] != gen || ng < (double)G[nb]) {
        if (hn >= hcap - 1) { capped = 1; goto done; }
        STAMP[nb] = gen; G[nb] = (float)ng; PREV[nb] = e;
        double dx = (double)X[nb] - gx, dy = (double)Y[nb] - gy;
        push(ng + __builtin_sqrt(dx * dx + dy * dy) / hdiv, nb);
      }
    }
    if (budget > 0 && ++steps >= budget) { more = 1; break; }
  }
done:
  p[EXPANDED] = expanded; p[FOUND] = found; p[CAPPED] = capped; p[HEAPN] = hn;
  return found ? 1 : (more ? 2 : 0);
}

// SPD[e] = TABLE[CLS[e]] — вільна швидкість за класом дороги для всіх ребер
void fill_spd(int *p) {
  float *SPD = (float *)p[PSPD]; const unsigned char *CLS = (const unsigned char *)p[PCLS]; const float *T = (const float *)p[PTABLE];
  for (int e = 0, m = p[M]; e < m; e++) SPD[e] = T[CLS[e] & 15];
}

// Голосові підказки через вбудований синтезатор (працює офлайн, якщо в системі є український голос).
// iOS: мовлення дозволяється лише після першої дії користувача — тому enable() викликаємо з кліку.

export class Voice {
  constructor() {
    this.enabled = JSON.parse(localStorage.getItem('navi.voice') || 'true');
    this.ok = typeof speechSynthesis !== 'undefined' && typeof SpeechSynthesisUtterance !== 'undefined';
    this.voice = null;
    this.last = { key: '', t: 0 };
    if (this.ok) { this._pick(); speechSynthesis.onvoiceschanged = () => this._pick(); }
  }
  _pick() {
    const vs = speechSynthesis.getVoices();
    this.voice = vs.find(v => /^uk/i.test(v.lang)) || vs.find(v => /ukrain/i.test(v.name)) || null;
    this.hasUk = !!this.voice;
  }
  setEnabled(v) { this.enabled = v; localStorage.setItem('navi.voice', JSON.stringify(v)); if (!v && this.ok) speechSynthesis.cancel(); }
  // «Розбудити» синтезатор з кліку (iOS)
  warmup() { if (this.ok) { const u = new SpeechSynthesisUtterance(''); speechSynthesis.speak(u); } }

  say(text, key = text, minGap = 8) {
    if (!this.ok || !this.enabled || window.NAVI_SAFE) return;
    const now = Date.now();
    if (key === this.last.key && now - this.last.t < minGap * 1000) return;
    this.last = { key, t: now };
    try {
      speechSynthesis.cancel();
      const u = new SpeechSynthesisUtterance(text);
      if (this.voice) u.voice = this.voice;
      u.lang = 'uk-UA'; u.rate = 1.0;
      speechSynthesis.speak(u);
    } catch { }
  }
}

// Текст підказки: відстань округлюємо як у навігаторах
export function turnPhrase(distM, text) {
  let d;
  if (distM < 40) d = '';
  else if (distM < 1000) d = `через ${Math.round(distM / 50) * 50} метрів `;
  else d = `через ${(distM / 1000).toFixed(1).replace('.0', '').replace('.', ' і ')} кілометра `;
  if (text === 'Прибуття') return distM < 40 ? 'Ви прибули' : `${d}прибуття`;
  return `${d}${text}`;
}

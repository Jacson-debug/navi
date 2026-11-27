// Голосові підказки через вбудований синтезатор (працює офлайн, якщо в системі є український голос).
// iOS: мовлення дозволяється лише після першої дії користувача — тому enable() викликаємо з кліку.
//
// Музика. За замовчуванням iOS вважає звук сторінки «відтворенням» і ставить чужу музику на паузу,
// а після підказки сама її не повертає. Audio Session API (Safari 16.4+) дозволяє сказати iOS інакше:
//   ambient   — у тиші звук сторінки змішується з музикою, нічого не перериває;
//   transient — коротка підказка: музика на цей час лише стишується, потім повертається.
// Тож тримаємо «ambient», а на час фрази вмикаємо «transient».

const AS = (typeof navigator !== 'undefined' && navigator.audioSession) || null;
function setSession(type) { try { if (AS && AS.type !== type) AS.type = type; } catch { } }
export function audioSessionInfo() { return AS ? AS.type : 'немає (стара iOS)'; }

export class Voice {
  constructor() {
    this.enabled = JSON.parse(localStorage.getItem('navi.voice') || 'true');
    this.ok = typeof speechSynthesis !== 'undefined' && typeof SpeechSynthesisUtterance !== 'undefined';
    this.voice = null;
    this.last = { key: '', t: 0 };
    this._restore = null;
    setSession('ambient');
    if (this.ok) { this._pick(); speechSynthesis.onvoiceschanged = () => this._pick(); }
  }
  _pick() {
    const vs = speechSynthesis.getVoices();
    this.voice = vs.find(v => /^uk/i.test(v.lang)) || vs.find(v => /ukrain/i.test(v.name)) || null;
    this.hasUk = !!this.voice;
  }
  setEnabled(v) { this.enabled = v; localStorage.setItem('navi.voice', JSON.stringify(v)); if (!v && this.ok) { speechSynthesis.cancel(); this._done(); } }
  // «Розбудити» синтезатор з кліку (iOS). Порожня фраза не повинна зупиняти музику.
  warmup() { if (this.ok) { setSession('ambient'); const u = new SpeechSynthesisUtterance(''); speechSynthesis.speak(u); } }

  // Фраза закінчилась: ще трохи тримаємо «стишення» (iOS повертає гучність із запізненням),
  // далі повертаємо сесію в «ambient» — музика грає як раніше.
  _done() {
    clearTimeout(this._restore);
    this._restore = setTimeout(() => { if (!speechSynthesis.speaking && !speechSynthesis.pending) setSession('ambient'); }, 700);
  }

  say(text, key = text, minGap = 8) {
    if (!this.ok || !this.enabled || window.NAVI_SAFE) return;
    const now = Date.now();
    if (key === this.last.key && now - this.last.t < minGap * 1000) return;
    this.last = { key, t: now };
    try {
      clearTimeout(this._restore);
      setSession('transient');            // музику не на паузу — лише стишити
      speechSynthesis.cancel();
      const u = new SpeechSynthesisUtterance(text);
      if (this.voice) u.voice = this.voice;
      u.lang = 'uk-UA'; u.rate = 1.0;
      u.onend = () => this._done();
      u.onerror = () => this._done();
      speechSynthesis.speak(u);
      // страховка: якщо iOS не надішле onend — повертаємо сесію за оцінкою тривалості фрази
      this._restore = setTimeout(() => this._done(), 1500 + text.length * 90);
    } catch { this._done(); }
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

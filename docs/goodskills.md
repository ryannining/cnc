# Good Skills & Tricks Knowledge Base

## Fixed-Point Error Diffusion for Integer Timing

**Trik:** Fixed-Point Timing Dithering dengan Error Carry-over

**Context:** Saat menghitung timing step (`tm`) dalam unit integer terbatas (tick_micros), speed transition tidak smooth karena tm harus integer (tidak bisa 1.7, 2.3, dll). Problem muncul saat speed naik 200→300→400mm/s, tm lompat dari 5→3→2 tanpa nilai intermediate.

**Pattern:**
```cpp
static uint32_t lastError = 0;

inline int stepgen() {
  if (sg != 2 || lV != V) {
    lV = V;
    realV = fast_isqrt_esp8266((V += a2));
    lastError = 0;  // Reset saat speed berubah
  }
  uint32_t fm = (globalSpeed > fixed_max_multiplier) ? fixed_max_multiplier : globalSpeed;

  // Scale 18-bit (10 original + 8 fraction)
  uint32_t tm = (uint32_t)(((uint64_t)(stepdiv2 << 18)) / (realV * fm));
  tm = (tm + lastError);
  lastError = tm & 255;  // Simpan 8-bit fractional
  tm = tm >> 8;          // Ambil integer part
  
  // ... rest of code
}
```

**Key Insight:** 
- `<< 18` beri 8-bit extra precision untuk fractional part
- `& 255` ambil error, carry ke step berikutnya via `+ lastError`
- Rata-rata timing tepat meski individual step integer
- Jangan pakai remainder `%` (expensive), cukup bit-mask `&` dan shift `>>`
- Reset `lastError = 0` saat speed berubah (acceleration/deceleration)

**Sumber:** Diskusi Feb 12 2026 dengan Ryan. Solusi simple ini lebih elegan dari 4 approach rumit (Bresenham, Phase Accumulator, Pattern Table, dll) yang saya sarankan di awal. Lesson: Don't overthink, simple is often better.

---

## Binary State Save for Power-Fail Resume

**Pattern:** Binary Format untuk Fast Persistence

**Context:** Saat mati listrik terdeteksi, sistem harus menyimpan posisi mesin secepat mungkin sebelum power hilang total. JSON terlalu lambat (parsing, allocation, fragmentation). Solusi: binary format sederhana.

**Implementation:**
```cpp
// Save - hanya 2x write operation
int32_t realSteps[3] = {shapers[0].realStep, shapers[1].realStep, shapers[2].realStep};
float jobPos[3] = {JobX, JobY, JobZ};

File f = LittleFS.open("/powerfail", "w");
f.write((uint8_t*)realSteps, 12);  // 3x int32_t
f.write((uint8_t*)jobPos, 12);      // 3x float
f.close();  // Total: ~1ms vs ~50ms untuk JSON

// Load - hanya 2x read operation
f.read((uint8_t*)realSteps, 12);
f.read((uint8_t*)jobPos, 12);

// Restore
shapers[0].realStep = realSteps[0];
// ... updateRealPos() akan konversi ke mm
```

**Key Insights:**
- Binary 24x lebih cepat daripada JSON serialize/deserialize
- Simpan step count (integer) alih-alih koordinat mm (float) untuk presisi penuh
- Tidak perlu library JSON, parsing, atau dynamic allocation
- Format: [12 bytes realSteps] + [12 bytes jobPos] = 24 bytes total
- Critical untuk power-fail: harus selesai sebelum capacitor discharge

**Sumber:** Implementasi Feb 2026 untuk fitur Power-Fail Auto-Resume. Menggunakan binary daripada JSON adalah trade-off readability untuk speed yang necessary dalam emergency save scenario.

---

## Tags
#timing #fixedpoint #isr #stepper #cnc #errordiffusion #dithering #binary #persistence #powerfail

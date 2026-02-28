# Dokumentasi Teknis CNC Controller Firmware

## 1. Gambaran Umum Sistem

Proyek ini merupakan firmware CNC Controller berbasis ESP8266/ESP32 yang mengendalikan mesin CNC 3-sumbu (X, Y, Z) dengan fitur motion planning, input shaping, dan kontrol via web. Firmware ini dioptimalkan untuk microcontroller ESP8266 dengan memori terbatas (~80KB) namun tetap mampu menghasilkan kualitas mesin yang halus melalui teknik input shaping untuk mengurangi getaran.

Sistem mendukung dua mode operasi: Laser dan Router (Spindle), dengan kontrol kecepatan tool berbasis PWM. Komunikasi dilakukan melalui WiFi (HTTP/WebSocket) dan ESP-NOW untuk real-time monitoring. Firmware juga mendukung format G-code teks standar dan format biner GX yang dikompresi untuk efisiensi penyimpanan dan parsing yang lebih cepat.

### 1.1 Fitur Baru (2026)

**Zero-Cross AC Detection** dengan interrupt eksternal untuk kontrol AC load presisi dan deteksi mati listrik.

**TOOLON Pin** terpisah dari PWM pin untuk kontrol laser yang lebih presisi - memisahkan kontrol power (analog) dari on/off (digital).

**Power-Fail Auto-Resume** - Fitur penyelamat untuk mesin tanpa endstop. Menyimpan posisi mesin saat mati listrik ke LittleFS dan mengembalikannya secara otomatis saat boot.

## 2. Arsitektur Sistem

### 2.1 Komponen Inti

Sistem terdiri dari beberapa komponen utama yang bekerja sama untuk menjalankan mesin CNC. Komponen pertama adalah **Motion Control System** yang mengelola semua gerakan stepper motor menggunakan algoritma Bresenham dengan trapezoidal velocity profiling. Sistem ini terletak di `cnc.ino` baris 494-1388 dan menggunakan ring buffer untuk Antrian gerakan dengan ukuran 32 segmen untuk ESP8266 dan 64 segmen untuk ESP32.

Komponen kedua adalah **Input Shaper** yang mereduksi getaran mesin CNC dengan memecah setiap step menjadi beberapa impulse yang tersebar waktu. Implementasi mendukung tiga tipe shaper: ZVD (Zero Vibration and Derivative), MZV (Modified Zero Vibration), dan EI (Extra-Insensitive). Shaper buffer menggunakan Struct of Arrays (SOA) layout untuk cache locality yang lebih baik pada X dan Y axis, sementara Z axis langsung step tanpa shaper.

Komponen ketiga adalah **G-code Parser** yang membaca dan mengeksekusi perintah G-code. Parser di `gcode.cpp` mendukung format teks standar (.gcode, .nc, .txt) dan format binerGX yang lebih efisien. Parser membaca file secara streaming per baris untuk menghindari penggunaan memori berlebihan.

Komponen keempat adalah **Network Stack** yang menyediakan antarmuka WiFi dengan HTTP server untuk konfigurasi dan upload file, WebSocket untuk real-time position broadcast, dan ESP-NOW untuk komunikasi dengan CNC Manager tanpa WiFi.

### 2.2 Struktur File

| File | Baris | Fungsi Utama |
|------|-------|--------------|
| `cnc.ino` | ~3100 | Main firmware, motion control, network |
| `structures.h` | 300 | Data structures, constants, config defaults |
| `gcode.cpp` | 458 | G-code parsing (text & binary GX) |
| `gcode.h` | 45 | G-code function declarations |
| `ui.cpp` | ~500+ | LCD UI dan menu system |
| `karyalcd.cpp/h` | ~400 | Driver LCD ST7565 |

## 3. Teknologi dan Teknik Optimasi

### 3.1 Direct Port Access (Fast GPIO)

Sistem menggunakan register hardware langsung untuk mengontrol GPIO alih-alih fungsi `digitalWrite()` yang lambat. Pada ESP8266, register `GPOS` (GPIO Set) dan `GPOC` (GPIO Clear) digunakan untuk mengontrol pin stepper secara langsung dengan latensi minimal. Implementasi berada di `cnc.ino` baris 58-70:

```cpp
#if defined(ESP8266)
#define GPOSet(mask) GPOS = (mask)
#define GPOClear(mask) GPOC = (mask)
#endif
```

 Teknik ini memberikan kecepatan switching yang jauh lebih tinggi karena tidak melewati Arduino framework abstraction layer. Setiap operasi GPIO hanya membutuhkan satu instruksi register write dibandingkan beberapa instruksi pada `digitalWrite()`.

### 3.2 Input Shaping Algorithm

Input shaping adalah teknik untuk mengurangi getaran struktural mesin CNC dengan cara memecah setiap step command menjadi beberapa impulse yang tersebar waktu. Ketika semua impulse dijumlahkan, hasilnya sama dengan step asli namun dengan distribusi waktu yang mengurangi eksitasi frekuensi resonansi mesin.

Sistem mendukung tiga algoritma yang dihitung secara real-time berdasarkan konfigurasi frekuensi dan damping ratio. **ZVD (Zero Vibration and Derivative)** adalah yang paling sederhana dengan 2-4 impulse yang menghilangkan getaran pada frekuensi target. **MZV (Modified ZV)** memberikan respons lebih cepat dengan pengorbanan sedikit pada vibration reduction. **EI (Extra-Insensitive)** adalah yang paling robust dengan toleransi lebih tinggi terhadap variasi frekuensi.

Perhitungan impulse dilakukan di `initShaper()` (cnc.ino:1992-2106) menggunakan rumus matematika yang melibatkan eksponensial dan akar kuadrat. Hasilnya dinormalisasi sehingga total bobot sama dengan SHAPER_SCALE (16).

### 3.3 Ring Buffer dengan Bitwise AND

Motion buffer menggunakan circular buffer dengan operasi bitwise AND untuk wrapping indeks. Ini lebih efisien daripada modulo division karena bitwise AND adalah operasi single-cycle pada microcontroller. Definisi macro di `structures.h` baris 171-172:

```cpp
#define nextbuff(x) ((x + 1) & (NUMBUFFER - 1))
#define prevbuff(x) ((x - 1) & (NUMBUFFER - 1))
```

Buffer bekerja dengan strategi producer-consumer di mana G-code parser memproduksi segmen gerakan (producer) dan ISR mengkonsumsinya (consumer). Dengan NUMBUFFER = 32 pada ESP8266, sistem memiliki cukup ruang untuk memproses beberapa segmen di depanexecution.

### 3.4 Step Event Buffer

Step event buffer menyimpan informasi step yang akan dieksekusi dalam bentuk terkompresi. Setiap event adalah 32-bit integer yang mengandung bit untuk step direction dan step enable ketiga axis, PWM value (8-bit), dan timing offset (16-bit). Encoding ini mengoptimalkan penggunaan memori karena hanya perlu satu integer per step event.

Buffer menggunakan encoding bits seperti didefinisikan di `structures.h` baris 191-206. Bits 0-5 untuk step/dir, bits 6-13 untuk PWM, dan bits 16-31 untuk waktu. Pendekatan ini memungkinkan encoding lengkap dalam satu word 32-bit dibanding menyimpan struct yang lebih besar.

### 3.5 Branch Prediction Hints

Compiler hint `__builtin_expect()` digunakan untuk mengoptimasi branch prediction pada kode critical path. Ini memberitahu compiler kondisi mana yang lebih mungkin terjadi, memungkinkan generator kode meletakkan fast path secara linear dan slow path dengan branch:

```cpp
if (__builtin_expect(sebfull, 0)) { // cold path
    log("Step event buffer full\n");
    return --Sdest > 0;
}
```

Pada ISR yang sangat ketat timing-nya, hints ini membantu processor memprediksi branch dengan lebih baik karena kondisi buffer full sangat jarang terjadi dibanding kondisi normal.

### 3.6 Fixed-Point Math

Beberapa perhitungan menggunakan fixed-point arithmetic untuk menghindari floating-point yang lambat pada ESP8266 tanpa FPU. Contohnya di `prepareramp()` (cnc.ino:904-959) menggunakan shift 12-bit untuk mensimulasikan presisi desimal:

```cpp
V = vi << 12;  // Simpan kecepatan dengan presisi 12-bit fractional
```

Konstanta seperti `CLOCKCONSTANT = 1000000 / TICK_MICROS` juga di-precompute untuk menghindari pembagian runtime.

## 4. Aliran Data (Data Flow)

### 4.1 Fase Inisialisasi

Pada saat sistem dinyalakan (`setup()` di cnc.ino:2965-3072), urutan inisialisasi dimulai dengan mounting LittleFS filesystem dan memuat konfigurasi dari `/config.json`. Jika file konfigurasi belum ada, sistem membuat default config menggunakan template di `structures.h` baris 296-297.

Setelah konfigurasi dimuat, pinout axis dikonfigurasi sesuai setting dan input shaper diinisialisasi untuk setiap axis berdasarkan tipe shaper yang dipilih (0=none, 1=ZVD, 2=MZV, 3=EI). Timer interrupt juga dikonfigurasi dengan interval `TICK_MICROS` (16 mikrodetik default) yang menentukan resolusi timing step generation.

Koneksi WiFi dicoba terlebih dahulu, dan jika gagal sistem beralih ke Access Point mode. Web server dan WebSocket dimulai, serta ESP-NOW diinisialisasi untuk komunikasi dengan CNC Manager.

### 4.2 Fase Parsing G-code

Ketika job dimulai via `startJob()` (cnc.ino:1461-1512), sistem membuka file G-code dari LittleFS dan mencoba mendeteksi format. Jika file dimulai dengan header "GX", sistem menggunakan binary parser (`readGxLine()`), sebaliknya menggunakan text parser (`readTextGCodeLine()`).

Parser text membaca baris per baris menggunakan `readStringUntil('\n')` dan memproses command G-code sederhana. Parser GX lebih efisien karena setiap command di-encode dalam 1-3 bytes dengan flag bits untuk parameter yang hadir. Setiap command G0/G1 yang ditemukan dikonversi menjadi motion segment melalui `addmove()`.

### 4.3 Fase Motion Planning

Fungsi `addmove()` (cnc.ino:1241-1313) menerima target coordinate dan mengkonversi ke step counts berdasarkan `stepsPerMm` setiap axis. Jarak total dihitung untuk menentukan feedrate, dan Bresenham algorithm digunakan untuk menentukan axis mana yang perlu di-step pada setiap iteration.

Setelah motion segment ditambahkan ke buffer, `planner()` (cnc.ino:1124-1194) dipanggil untuk menghitung kecepatan optimal menggunakan dua pass algorithm. **Backward pass** menghitung kecepatan start yang aman untuk setiap segmen berdasarkan kecepatan end segmen berikutnya. **Forward pass** memvalidasi bahwa kecepatan tidak melebihi batas akselerasi dan junction deviation.

Cornering logic menggunakan junction deviation algorithm yang menghitung kecepatan maksimal di persimpangan berdasarkan gaya sentripetal yang diizinkan. Ini mencegah机器人 dari bergerak terlalu cepat saat belok yang bisa menyebabkan lost steps atau getaran excess.

### 4.4 Fase Step Generation

Di level paling rendah, `stepgen()` (cnc.ino:849-897) menghasilkan step events berdasarkan ramp state saat ini. Setiap pemanggilan menghasilkan satu event yang berisi timing untuk step berikutnya, direction bits untuk ketiga axis, dan PWM value untuk tool.

Step events di-encode ke dalam 32-bit integer dan di-queue ke step event buffer. Buffer ini kemudian dikonsumsi oleh ISR yang berjalan pada interval `TICK_MICROS`. Jika buffer penuh, step generation berhenti sementara hingga ISR mengkosongkan beberapa event.

### 4.5 Fase ISR Execution

Timer ISR (`shaperISR()` di cnc.ino:661-763) adalah jantung sistem yang berjalan pada frekuensi tinggi (~62.5kHz pada TICK_MICROS=16). Setiap tick, ISR melakukan beberapa tugas: consume step events dari buffer jika sudah waktunya, process input shaper untuk X dan Y axis, generate step pulses ke motor, dan output PWM untuk tool.

Untuk input shaping, setiap step yang akan dihasilkan melalui shaper dipecah menjadi beberapa impulse. Impulse pertama selalu langsung, sementara impulse berikutnya di-schedule ke future ticks dalam shaper buffer. Ketika tick berjalan, ISR membaca impulse dari buffer dan mengakumulasi ke accumulator, kemudian menggunakan error diffusion (dithering) untuk menentukan apakah step harus di-generate pada tick tersebut.

### 4.6 Fase Reporting

Status dan posisi real-time di-broadcast melalui beberapa channel. Via WebSocket, posisi dikirim dalam format binary setiap 50ms ketika job running (konfigurasi). Via ESP-NOW, status dikirim ke CNC Manager setiap 1 detik dan posisi dikirim setiap 100ms jika enabled.

Data posisi dikumpulkan dalam circular buffer (`positionHistory[]`) sebelum dikirim untuk mengurangi overhead komunikasi. Format binary digunakan untuk efisiensi bandwidth dengan susunan: [header][X][Y][Z][laser][status][progress][filename].

## 5. Spesifikasi Teknis

### 5.1 Hardware yang Didukung

| Platform | Timer | Motion Buffer | Step Event Buffer | Shaper Buffer |
|----------|-------|--------------|-------------------|---------------|
| ESP8266 | 16µs | 32 segments | 1024 events | 4096 ticks |
| ESP32 | 16µs | 64 segments | 2048 events | 8192 ticks |

### 5.2 Pin Configuration Default

**ESP8266:**
- X Axis: Step=D2(D4), Dir=D1(D5)
- Y Axis: Step=D6(D12), Dir=D7(D13)
- Z Axis: Step=D5(D14), Dir=D4(D2)
- Tool PWM: D8(D15)
- Endstop: D0(D16)

**ESP32:**
- X Axis: Step=IO15, Dir=IO4
- Y Axis: Step=IO2, Dir=IO16
- Z Axis: Step=IO0, Dir=IO17
- Tool PWM: IO15

### 5.3 G-code yang Didukung

| Command | Deskripsi |
|---------|-----------|
| G0 | Rapid move (tanpa tool) |
| G1 | Linear move dengan feedrate |
| G28 | Home semua axis |
| M3 S\<power\> | Spindle/Laser on |
| M5 | Spindle/Laser off |
| G90 | Absolute mode |
| G91 | Relative mode |

### 5.4 API Endpoints

| Endpoint | Method | Deskripsi |
|---------|--------|-----------|
| `/api/config` | GET/POST | Konfigurasi sistem |
| `/api/files` | GET | List file di LittleFS |
| `/upload` | POST | Upload file G-code |
| `/api/job/start` | POST | Mulai job |
| `/api/job/preview` | POST | Preview batas job |
| WebSocket :81 | WS | Real-time status |

## 6. Saran Perbaikan dan Optimasi

### 6.1 Optimasi Performa Tinggi

**A. Dual-Buffer untuk Step Events**

Saat ini step event buffer menggunakan single circular buffer yang sama untuk semua axis. Implementasi dual-buffer bisa memisahkan producer (stepgen) dan consumer (ISR) dengan dua buffer yang di-swapped secara atomik. Ini mengeliminasi risiko race condition dan memungkinkan implementasi yang lebih sederhana dengan locking yang lebih kasar.

Buffer pertama digunakan untuk menulis sementara buffer kedua sedang dibaca oleh ISR. Ketika ISR selesai membaca buffer kedua, buffers di-swap dengan pointer exchange yang sangat cepat. Perubahan ini memerlukan modifikasi pada `stepgen()` dan `shaperISR()` dengan estimasi effort 2-3 jam.

**B. Hardware Timer dengan DMA**

Timer interrupt saat ini menggunakan software interrupt yang memerlukan context saving dan restoration. Pada ESP32, menggunakan hardware timer dengan DMA (Direct Memory Access) untuk step generation bisa mengurangi jitter secara signifikan. Ini memerlukan pengetahuan mendalam tentang ESP32 hardware dan estimasi effort 8-12 jam.

Alternatif yang lebih sederhana adalah menggunakan RMT (Remote Control) peripheral pada ESP32 yang dirancang untuk generate precise pulses tanpa CPU intervention. Namun ini memerlukan perubahan arsitektur yang signifikan.

**C. Adaptive Input Shaper**

Input shaper saat ini menggunakan konfigurasi statis yang ditentukan saat compile/boot. Implementasi adaptive shaper bisa mengukur respons frekuensi aktual mesin saat startup dan menyesuaikan parameter shaper secara otomatis. Ini melibatkan menambahkan routine excitation (misalnya step response kecil), mengukur response dengan accelerometer sederhana, dan menghitung frekuensi resonansi actual.

Dengan firmware update, sistem bisa lebih robust terhadap variasi mesin tanpa manual tuning. Estimasi effort 4-6 jam untuk implementasi dasar.

### 6.2 Optimasi Memori

**A. Pointer-Based Buffer Management**

Motion buffer menggunakan array statis dengan indeks integer. Mengganti ke pointer-based linked list atau memory pool bisa mengurangi fragmentasi memori dan memungkinkan dynamic sizing berdasarkan availability. Namun ini harus dilakukan dengan hati-hati karena microcontroller tidak memiliki MMU untuk memory protection.

Perubahan ini beresiko introducing bugs jika tidak dilakukan dengan benar. Estimasi effort 3-4 jam dengan testing yang komprehensif.

**B. Compressed Motion State**

Beberapa variabel state seperti `sg` (segment state), `status`, dan flag lainnya bisa dikemas ke dalam bit fields untuk mengurangi penggunaan RAM. Kompiler sudah melakukan beberapa optimasi, namun manual bit-packing untuk state machine yang sering diakses bisa membantu.

Contoh implementasinya adalah menggunakan bit fields dalam struct tmove untuk field-field kecil seperti `maxv` (0-63) yang hanya membutuhkan 6 bits. Estimasi effort 1-2 jam.

**C. PROGMEM untuk Constant Data**

Beberapa variabel konfigurasi yang tidak berubah saat runtime bisa dipindahkan ke PROGMEM untuk menghemat RAM. Ini termasuk default values untuk konfigurasi axis dan lookup tables untuk trigonometry. Estimasi effort 1 jam.

### 6.3 Feature Additions

**A. Arc Interpolation (G2/G3)**

Firmware saat ini hanya mendukung linear interpolation (G0/G1). Menambahkan support untuk circular arcs (G2/G3) akan meningkatkan kemampuan untuk memotong kurva dan lingkaran tanpa harus mengaproksimasi dengan banyak segmen garis kecil. Ini memerlukan implementasi midpoint circle algorithm dengan chord error tolerance.

Algoritma akan menerima center point atau radius, start/end angles, dan menghasilkan titik-titik yang di-approximasi dengan presisi yang ditentukan. Estimasi effort 6-8 jam.

**B. Canned Cycles (G81-G89)**

Menambahkan standard canned cycles seperti drilling, peck drilling, dan tapping akan mempercepat programming untuk operasi umum. Setiap cycle menerima parameter seperti depth, feed, dan retract distance.

Ini sangat berguna untuk operasi drilling yang repetitif. Estimasi effort 8-10 jam untuk implementasi lengkap.

**C. Runtime Tool Length Compensation**

Saat ini kompensasi panjang tool harus dilakukan di CAM software. Menambahkan runtime compensation позволя systems meng-offset sumbu Z berdasarkan tool yang dipilih. Ini memerlukan implementasi tool table storage dan G-code untuk memilih tool (T codes). Estimasi effort 5-6 jam.

**D. Closed-Loop Feedback dengan Encoder**

Firmware saat ini open-loop dan tidak memverifikasi bahwa stepper benar-benar bergerak. Menambahkan rotary encoder pada satu atau lebih axis untuk closed-loop feedback bisa mendeteksi dan mengkompensasi lost steps secara real-time.

Ini memerlukan hardware tambahan (encoder) dan modifikasi firmware untuk membandingkan expected position dengan actual position. Jika selisih exceeds threshold, sistem bisa stop atau mengkompensasi. Estimasi effort 8-10 jam dengan hardware modification.

**E. Advanced Homing dengan Sensorless**

Implementasi homing saat ini menggunakan physical endstop switches. Untuk beberapa aplikasi, sensorless homing menggunakan stall detection pada stepper driver bisa menghilangkan kebutuhan switches fisik. Ini memerlukan stepper driver yang support stall detection dan threshold configuration.

### 6.4 Reliability Improvements

**A. Watchdog Timer**

Menambahkan watchdog timer untuk recover dari kondisi stuck. Jika main loop atau ISR tidak berjalan dalam waktu yang ditentukan, sistem akan reset secara otomatis. Arduino framework sudah memiliki built-in watchdog yang bisa di-enable dengan `wdt_enable()`.

Estimasi effort 30 menit dengan testing.

**B. Error Recovery untuk Buffer Underflow**

Saat buffer underflow terjadi (ISR kehabisan events), sistem saat ini berhenti dengan tiba-tiba. Implementasi smooth deceleration bisa menangani ini dengan graceful manner. Ketika buffer level drops below threshold, sistem menurunkan speed secara gradual hingga buffer recover atau machine stops.

Estimasi effort 2-3 jam.

**C. Checksum untuk Konfigurasi**

Konfigurasi yang corrupt bisa menyebabkan behavior yang tidak terduga. Menambahkan CRC32 checksum pada file konfigurasi dan memvalidasi saat load bisa mendeteksi corruption early. Estimasi effort 1 jam.

### 6.5 User Experience

**A. Web Interface yang Lebih Baik**

Web interface saat ini sangat dasar. Menambahkan visualisasi real-time dari job progress, interactive jog controls, dan configuration GUI akan significantly meningkatkan usability. Ini memerlukan pengembangan frontend terpisah (HTML/CSS/JS) yang di-serving oleh ESP.

Estimasi effort 8-10 jam untuk frontend development.

**B. Job Simulation**

Menambahkan kemampuan untuk simulate job execution tanpa actual motion bisa membantu users verify G-code sebelum running di mesin. Simulator akan menjalankan parsing dan planning namun bypass step generation. Estimasi effort 3-4 jam.

**C. Audio Feedback**

Menambahkan support untuk buzzer atau speaker untuk feedback audible untuk events seperti job complete, error, atau button press. Ini memerlukan hardware tambahan dan kode untuk generate tones. Estimasi effort 1-2 jam.

### 6.6 Fitur Implemented (2026)

**A. Power-Fail Detection dan Auto-Resume**

Fitur ini telah diimplementasikan menggunakan zero-cross detection untuk mendeteksi pemadaman listrik dan menyimpan posisi mesin ke LittleFS.

Implementasi:
- Zero-cross interrupt (`zeroCrossISR`) mencatat waktu terakhir crossing
- Main loop memeriksa interval zero-cross setiap 100ms
- Saat mati PLN terdeteksi: force stop, save binary state (24 byte), hapus buffer
- Saat boot: auto-load dan restore posisi mesin

Format binary: 3x int32_t (realStep X/Y/Z) + 3x float (jobX/Y/Z)

Dokumentasi lengkap: `docs/powerfail-resume.md`

**B. Zero-Cross PWM untuk AC Control**

Mode PWM baru (pwmMode = 2) menggunakan zero-cross detection untuk kontrol AC load dengan phase control. Interrupt ZC mematikan load, kemudian shaperISR menyalakan kembali setelah delay proportional dengan power level.

**C. TOOLON Pin Terpisah**

Pin terpisah untuk kontrol on/off laser, memungkinkan pemisahan kontrol power (analog PWM) dari enable signal (digital). Pin 255 = NC (Not Connected).

## 7. Kesimpulan

Firmware CNC Controller ini merupakan implementasi yang well-optimized untuk ESP8266 dengan focus pada performance dan resource efficiency. Input shaping, direct port access, dan ring buffer management menunjukkan pemahaman yang baik tentang constraint microcontroller.

Area yang paling berdampak untuk improvement adalah: (1) dual-buffer untuk step events yang bisa reduce jitter, (2) closed-loop feedback untuk reliability, dan (3) web interface yang lebih baik untuk usability. Feature additions seperti arc interpolation dan canned cycles akan membuat sistem lebih versatile untuk berbagai aplikasi.

Prioritas rekomendasi: High priority adalah error recovery dan watchdog untuk reliability. Medium priority adalah dual-buffer dan web interface improvements. Low priority adalah advanced features seperti arc interpolation yang bisa di-add seiring waktu.

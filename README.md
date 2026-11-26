# xiaomi-touchgesture

AIDL service untuk NDT2W (double-tap-to-wake) di port HyperOS/MIUI MTK
Transsion. Menjaga `/proc/gesture_function` tetap berisi `cc1` saat layar
mati/AOD dan toggle `gesture_wakeup` aktif.

## Kenapa service sendiri

MIUI menyimpan toggle NDT2W di `Settings.System.gesture_wakeup`, tapi di port
ini tidak ada yang menerjemahkannya ke node gesture kernel. Node yang sama
juga dipakai FOD HAL untuk `fp1` saat jari menekan sensor sidik jari.

Aturan yang dipakai:

- `gesture_wakeup=1` dan layar bukan ON (`0/1/3/4`) → tulis `cc1` ke
  `/proc/gesture_function` **dan** `1` ke node FTS `fts_gesture_mode`.
  Keduanya diperlukan supaya controller mengeluarkan KEY `0xC6`.
- Layar ON → tidak menulis apa-apa, biarkan `fp1` dari FOD HAL.
- Jari menekan sensor FOD → service berhenti menulis, FOD HAL menang.
- Jari lepas dan layar masih doze → `cc1` ditulis ulang.
- Service menulis ulang `cc1` tiap 2 detik untuk mengalahkan writer lain.
- Saat KEY `0xC6` (double tap) muncul, service menyuntik `input keyevent 224`
  dengan debounce 1 detik. Timestamp disimpan di `/data/adb/fod/dt2w_last_wake`
  supaya tidak dobel setelah `getevent` restart.

Deteksi jari dan gesture memakai node touch yang dicari otomatis dari
`/sys/class/input/event*/device/name` (`mtk-tpd` / `*touch*`), bukan hardcode
`event4`.

Status layar diambil dari `debug.tracing.screen_state` via Bionic futex.
Fallback: `Reg(0xD0)` di `/sys/class/input/event4/device/device/fts_gesture_mode`
(`0` = ON, `1` = DOZE/AOD).

## AIDL

Service: `vendor.xiaomi.hardware.touchgesture.ITouchGesture/default`

```aidl
interface ITouchGesture {
    int setEnabled(int enabled);   // 0/1, override toggle Settings
    int getEnabled();
    int getState();                // 0/1 OFF, 2 ON, 3/4 DOZE
    int writeGesture(String value); // debug, mis. cc1 / fp1
}
```

## Build

Butuh NDK r27c dan target `aarch64-linux-android`.

```sh
export CARGO_TARGET_AARCH64_LINUX_ANDROID_LINKER=$NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/aarch64-linux-android33-clang
export CARGO_TARGET_AARCH64_LINUX_ANDROID_AR=$NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/llvm-ar
cargo build --release
```

## Test di device

```sh
adb push target/release/xiaomi_touchgesture /data/local/tmp/
adb shell chmod 755 /data/local/tmp/xiaomi_touchgesture
adb shell 'nohup /data/local/tmp/xiaomi_touchgesture >/data/local/tmp/touchgesture.log 2>&1 </dev/null &'
adb shell /data/local/tmp/xiaomi_touchgesture --status

# toggel dari Settings, atau lewat service:
adb shell 'service call vendor.xiaomi.hardware.touchgesture.ITouchGesture/default 1 i32 1'
```

## Pasang permanen

Butuh image rebuild: `init/xiaomi-touchgesture.rc` dan
`manifest_touchgesture.xml`. Service harus root karena node `/proc/gesture_function`
hanya writable root.

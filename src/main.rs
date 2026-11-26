#![allow(non_snake_case)]

//! xiaomi_touchgesture - keep /proc/gesture_function on `cc1` while the
//! screen is asleep/AOD and MIUI's double-tap-to-wake setting is enabled.
//!
//! The FOD HAL owns the same node while a finger is on the optical sensor and
//! writes `fp1`. This service stays out of the way during that press and
//! re-asserts `cc1` once the finger lifts.

use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int, c_uchar, c_void};
use std::io::BufRead;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicI64, AtomicU64, Ordering};
use std::sync::mpsc::{channel, RecvTimeoutError, Sender};
use std::time::{Duration, Instant};

use anyhow::anyhow;
use rsbinder::Interface;

include!(concat!(env!("OUT_DIR"), "/touchgesture.rs"));

use crate::vendor::xiaomi::hardware::touchgesture::ITouchGesture::{
    BnTouchGesture, ITouchGesture,
};

const SERVICE_NAME: &str = "vendor.xiaomi.hardware.touchgesture.ITouchGesture/default";
const GESTURE_NODE: &str = "/proc/gesture_function";
const SCREEN_STATE_PROP: &str = "debug.tracing.screen_state";
const FTS_NODE: &str =
    "/sys/devices/platform/soc/11010000.spi1/spi_master/spi1/spi1.0/fts_gesture_mode";
const WAKE_KEY: &str = "224";
const WAKE_GESTURE_KEY: u32 = 0x00c6;
const WAKE_STAMP: &str = "/data/adb/fod/dt2w_last_wake";
const WAKE_GESTURE: &str = "cc1";
const REASSERT_MS: u64 = 2_000;

const SCREEN_OFF: i32 = 0;
const SCREEN_OFF_AOD: i32 = 1;
const SCREEN_ON: i32 = 2;
const SCREEN_DOZE: i32 = 3;
const SCREEN_DOZE_SUSPEND: i32 = 4;

static ENABLED: AtomicI32 = AtomicI32::new(0);
static SCREEN_STATE: AtomicI32 = AtomicI32::new(SCREEN_ON);
static FOD_PRESSED: AtomicI32 = AtomicI32::new(0);
static TOUCH_X: AtomicI64 = AtomicI64::new(-1);
static TOUCH_Y: AtomicI64 = AtomicI64::new(-1);

const FOD_CENTER_X: i64 = 540;
const FOD_CENTER_Y: i64 = 2172;
const FOD_RADIUS: i64 = 170;

/// mtk-tpd reports 16x the panel coordinates. Only a touch inside the FOD
/// circle counts as "sensor pressed"; a plain BTN_TOUCH elsewhere must not
/// make this service yield the node to the FOD HAL.
fn in_fod_area(x_raw: i64, y_raw: i64) -> bool {
    let dx = x_raw / 16 - FOD_CENTER_X;
    let dy = y_raw / 16 - FOD_CENTER_Y;
    dx * dx + dy * dy <= FOD_RADIUS * FOD_RADIUS
}

// ------------------------------------------------------------ Bionic prop FFI

#[cfg(target_os = "android")]
extern "C" {
    fn __system_property_get(name: *const c_uchar, value: *mut c_uchar) -> c_int;
    fn __system_property_find(name: *const c_char) -> *const c_void;
    fn __system_property_read_callback(
        pi: *const c_void,
        callback: extern "C" fn(
            cookie: *mut c_void,
            name: *const c_char,
            value: *const c_char,
            serial: u32,
        ),
        cookie: *mut c_void,
    );
    fn __system_property_wait(
        pi: *const c_void,
        old_serial: u32,
        new_serial: *mut u32,
        timeout: *const c_void,
    ) -> bool;
}

#[cfg(not(target_os = "android"))]
unsafe fn __system_property_get(_name: *const c_uchar, _value: *mut c_uchar) -> c_int {
    0
}

#[cfg(not(target_os = "android"))]
unsafe fn __system_property_find(_name: *const c_char) -> *const c_void {
    std::ptr::null()
}

#[cfg(not(target_os = "android"))]
unsafe fn __system_property_read_callback(
    _pi: *const c_void,
    _cb: extern "C" fn(*mut c_void, *const c_char, *const c_char, u32),
    _cookie: *mut c_void,
) {
}

#[cfg(not(target_os = "android"))]
unsafe fn __system_property_wait(
    _pi: *const c_void,
    _old: u32,
    _new: *mut u32,
    _timeout: *const c_void,
) -> bool {
    false
}

struct PropValue {
    value: String,
    serial: u32,
}

extern "C" fn read_callback(
    cookie: *mut c_void,
    _name: *const c_char,
    value: *const c_char,
    serial: u32,
) {
    unsafe {
        let pv = &mut *(cookie as *mut PropValue);
        pv.serial = serial;
        if !value.is_null() {
            pv.value = CStr::from_ptr(value).to_string_lossy().into_owned();
        }
    }
}

fn find_prop(name: &str) -> Option<*const c_void> {
    let c_name = CString::new(name).ok()?;
    let pi = unsafe { __system_property_find(c_name.as_ptr()) };
    if pi.is_null() {
        None
    } else {
        Some(pi)
    }
}

fn read_prop(pi: *const c_void) -> PropValue {
    let mut pv = PropValue {
        value: String::new(),
        serial: 0,
    };
    unsafe {
        __system_property_read_callback(
            pi,
            read_callback,
            &mut pv as *mut PropValue as *mut c_void,
        );
    }
    pv
}

fn wait_prop(pi: *const c_void, old_serial: u32) -> Option<u32> {
    let mut new_serial = 0u32;
    if unsafe { __system_property_wait(pi, old_serial, &mut new_serial, std::ptr::null()) } {
        Some(new_serial)
    } else {
        None
    }
}

fn wait_for_prop(name: &str) -> *const c_void {
    loop {
        if let Some(pi) = find_prop(name) {
            return pi;
        }
        std::thread::sleep(Duration::from_millis(200));
    }
}

fn get_prop(key: &str) -> Option<String> {
    const MAX: usize = 92;
    let c_key = CString::new(key).ok()?;
    let mut buf = vec![0u8; MAX];
    let n = unsafe { __system_property_get(c_key.as_ptr() as *const c_uchar, buf.as_mut_ptr()) };
    if n > 0 {
        Some(unsafe { CStr::from_ptr(buf.as_ptr() as *const c_char) }
            .to_string_lossy()
            .into_owned())
    } else {
        None
    }
}

// ------------------------------------------------------------------ hardware

fn parse_screen_state(s: &str) -> Option<i32> {
    let v: i32 = s.trim().parse().ok()?;
    Some(match v {
        SCREEN_ON => SCREEN_ON,
        SCREEN_OFF | SCREEN_OFF_AOD | SCREEN_DOZE | SCREEN_DOZE_SUSPEND => v,
        _ => return None,
    })
}

/// Fallback when debug.tracing.screen_state is absent: Reg(0xD0) in the FTS
/// gesture node is 0 while awake and 1 in doze on this MTK port.
fn fts_screen_state() -> Option<i32> {
    let text = std::fs::read_to_string(FTS_NODE).ok()?;
    let flag = text.split("Reg(0xD0)=").nth(1)?.trim().chars().next()?;
    match flag {
        '0' => Some(SCREEN_ON),
        '1' => Some(SCREEN_DOZE),
        _ => None,
    }
}

/// cc1 owns the node whenever the panel is not lit for the user and no finger
/// is on the optical sensor. Screen ON and a finger-down both mean "leave the
/// node alone" (the FOD HAL writes fp1).
fn should_write_cc1(state: i32, enabled: bool, fod_pressed: bool) -> bool {
    enabled && state != SCREEN_ON && !fod_pressed
}

fn write_gesture(value: &str) -> std::io::Result<()> {
    std::fs::write(GESTURE_NODE, value.as_bytes())
}

fn current_enabled() -> bool {
    Command::new("/system/bin/settings")
        .args(["get", "system", "gesture_wakeup"])
        .output()
        .ok()
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .and_then(|s| s.trim().parse::<i32>().ok())
        .unwrap_or(0)
        != 0
}

fn apply(force: bool) {
    let state = SCREEN_STATE.load(Ordering::SeqCst);
    let enabled = ENABLED.load(Ordering::SeqCst) != 0;
    let pressed = FOD_PRESSED.load(Ordering::SeqCst) != 0;

    if !should_write_cc1(state, enabled, pressed) {
        return;
    }
    if !force && APPLIED.load(Ordering::SeqCst) {
        return;
    }
    match write_gesture(WAKE_GESTURE) {
        Ok(()) => {
            tracing::info!("[Gesture] wrote {WAKE_GESTURE} (state={state})");
            APPLIED.store(true, Ordering::SeqCst);
        }
        Err(e) => tracing::warn!("[Gesture] write {WAKE_GESTURE} failed: {e}"),
    }

    // The FTS controller also needs its own gesture mode armed, otherwise
    // KEY 0xC6 is never emitted while the panel is in doze/AOD.
    if let Err(e) = std::fs::write(FTS_NODE, b"1\n") {
        tracing::warn!("[Gesture] arm {FTS_NODE} failed: {e}");
    }
}

static APPLIED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
static AIDL_OVERRIDE: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
static LAST_WAKE_SEC: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

fn now_sec() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

fn find_touch_node() -> Option<String> {
    for entry in std::fs::read_dir("/sys/class/input").ok()?.flatten() {
        let name_path = entry.path().join("device/name");
        let name = std::fs::read_to_string(name_path).unwrap_or_default();
        let name = name.trim();
        if name == "mtk-tpd" || name.contains("touch") {
            let event = entry.file_name().to_string_lossy().into_owned();
            return Some(format!("/dev/input/{event}"));
        }
    }
    None
}

/// FocalTech emits KEY 0xC6 on a double tap while doze. Turn that into a real
/// wake key, with a 1s debounce persisted so a getevent restart cannot repeat.
fn maybe_wake() {
    let now = now_sec();
    let last = LAST_WAKE_SEC.load(Ordering::SeqCst);
    if now.saturating_sub(last) < 1 {
        return;
    }
    LAST_WAKE_SEC.store(now, Ordering::SeqCst);
    let _ = std::fs::write(WAKE_STAMP, now.to_string());
    if let Err(e) = Command::new("/system/bin/input")
        .args(["keyevent", WAKE_KEY])
        .status()
    {
        tracing::warn!("[Gesture] keyevent {WAKE_KEY} failed: {e}");
    } else {
        tracing::info!("[Gesture] wake key {WAKE_KEY} injected");
    }
}

// --------------------------------------------------------------- AIDL service

struct TouchGestureHal;

impl Interface for TouchGestureHal {}

impl ITouchGesture for TouchGestureHal {
    fn setEnabled(&self, enabled: i32) -> rsbinder::status::Result<i32> {
        AIDL_OVERRIDE.store(true, Ordering::SeqCst);
        ENABLED.store(if enabled != 0 { 1 } else { 0 }, Ordering::SeqCst);
        APPLIED.store(false, Ordering::SeqCst);
        apply(true);
        Ok(0)
    }

    fn getEnabled(&self) -> rsbinder::status::Result<i32> {
        Ok(ENABLED.load(Ordering::SeqCst))
    }

    fn getState(&self) -> rsbinder::status::Result<i32> {
        Ok(SCREEN_STATE.load(Ordering::SeqCst))
    }

    fn writeGesture(&self, value: &str) -> rsbinder::status::Result<i32> {
        match write_gesture(value) {
            Ok(()) => Ok(0),
            Err(_) => Ok(-1),
        }
    }
}

// ------------------------------------------------------------------- watchers

fn spawn_screen_watcher(tx: Sender<()>) {
    std::thread::spawn(move || {
        let pi = wait_for_prop(SCREEN_STATE_PROP);
        let initial = read_prop(pi);
        if let Some(s) = parse_screen_state(&initial.value) {
            SCREEN_STATE.store(s, Ordering::SeqCst);
        }
        let mut serial = initial.serial;
        let _ = tx.send(());

        loop {
            let Some(new_serial) = wait_prop(pi, serial) else {
                continue;
            };
            serial = new_serial;
            let pv = read_prop(pi);
            if pv.serial != 0 {
                serial = pv.serial;
            }
            if let Some(s) = parse_screen_state(&pv.value) {
                tracing::info!("[Gesture] screen_state={s}");
                SCREEN_STATE.store(s, Ordering::SeqCst);
                APPLIED.store(false, Ordering::SeqCst);
                let _ = tx.send(());
            }
        }
    });
}

fn spawn_input_watcher(tx: Sender<()>) {
    std::thread::spawn(move || loop {
        let node = match find_touch_node() {
            Some(n) => n,
            None => {
                tracing::warn!("[Input] touch node not found");
                std::thread::sleep(Duration::from_secs(2));
                continue;
            }
        };
        let child = Command::new("/system/bin/getevent")
            .args(["-q", &node])
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .spawn();
        let mut child = match child {
            Ok(c) => c,
            Err(e) => {
                tracing::warn!("[Input] getevent failed: {e}");
                std::thread::sleep(Duration::from_secs(2));
                continue;
            }
        };
        if let Some(out) = child.stdout.take() {
            for line in std::io::BufReader::new(out).lines().map_while(Result::ok) {
                let mut p: Vec<&str> = line.split_whitespace().collect();
                if p.first().map_or(false, |t| t.ends_with(':')) {
                    p.remove(0);
                }
                if p.len() < 3 {
                    continue;
                }
                let ty = u32::from_str_radix(p[0], 16).unwrap_or(0);
                let code = u32::from_str_radix(p[1], 16).unwrap_or(0);
                let val = u32::from_str_radix(p[2], 16).unwrap_or(0) as i32;
                if ty == 0x03 {
                    match code {
                        0x35 => TOUCH_X.store(val as i64, Ordering::SeqCst),
                        0x36 => TOUCH_Y.store(val as i64, Ordering::SeqCst),
                        _ => {}
                    }
                    continue;
                }
                if ty != 0x01 {
                    continue;
                }
                if ty == 0x01 && code == WAKE_GESTURE_KEY && val == 1 {
                    maybe_wake();
                    continue;
                }
                let is_fod_key = code == 0xc3 || code == 0x152;
                let is_touch_in_fod = code == 0x14a
                    && in_fod_area(
                        TOUCH_X.load(Ordering::SeqCst),
                        TOUCH_Y.load(Ordering::SeqCst),
                    );
                if !is_fod_key && !is_touch_in_fod {
                    continue;
                }
                let down = if val > 0 { 1 } else { 0 };
                if FOD_PRESSED.swap(down, Ordering::SeqCst) != down {
                    tracing::info!("[Input] FOD press={down}");
                    APPLIED.store(false, Ordering::SeqCst);
                    let _ = tx.send(());
                }
            }
        }
        let _ = child.wait();
        std::thread::sleep(Duration::from_millis(500));
    });
}

fn usage() -> ! {
    eprintln!(
        "xiaomi_touchgesture {}\n\
         usage:\n\
           xiaomi_touchgesture [--daemon]\n\
           xiaomi_touchgesture --write cc1|fp1\n\
           xiaomi_touchgesture --status",
        env!("CARGO_PKG_VERSION")
    );
    std::process::exit(2)
}

fn main() -> anyhow::Result<()> {
    let _ = tracing_log::LogTracer::init();
    let _ = tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .with_ansi(false)
        .try_init();

    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.iter().any(|a| a == "--status") {
        println!("service   : {SERVICE_NAME}");
        println!("node      : {GESTURE_NODE}");
        println!("state     : {}", SCREEN_STATE.load(Ordering::SeqCst));
        println!("enabled   : {}", ENABLED.load(Ordering::SeqCst));
        println!("fod_press : {}", FOD_PRESSED.load(Ordering::SeqCst));
        println!("settings  : gesture_wakeup={}", current_enabled());
        println!("fts       : {:?}", fts_screen_state());
        println!("prop      : {:?}", get_prop(SCREEN_STATE_PROP));
        return Ok(());
    }
    if let Some(i) = args.iter().position(|a| a == "--write") {
        let value = args.get(i + 1).map(String::as_str).unwrap_or_else(|| usage());
        write_gesture(value).map_err(|e| anyhow!("write {GESTURE_NODE}: {e}"))?;
        return Ok(());
    }

    let initial_state = get_prop(SCREEN_STATE_PROP)
        .and_then(|s| parse_screen_state(&s))
        .or_else(fts_screen_state)
        .unwrap_or(SCREEN_ON);
    SCREEN_STATE.store(initial_state, Ordering::SeqCst);
    ENABLED.store(if current_enabled() { 1 } else { 0 }, Ordering::SeqCst);
    if let Ok(stamp) = std::fs::read_to_string(WAKE_STAMP) {
        if let Ok(sec) = stamp.trim().parse::<u64>() {
            LAST_WAKE_SEC.store(sec, Ordering::SeqCst);
        }
    }

    let (tx, rx) = channel::<()>();
    spawn_screen_watcher(tx.clone());
    spawn_input_watcher(tx);

    rsbinder::ProcessState::init_default()
        .map_err(|e| anyhow!("ProcessState::init_default: {e}"))?;
    rsbinder::ProcessState::start_thread_pool();
    let binder = BnTouchGesture::new_binder(TouchGestureHal);
    rsbinder::hub::add_service(SERVICE_NAME, binder.as_binder())
        .map_err(|e| anyhow!("register {SERVICE_NAME}: {e}"))?;
    tracing::info!("[HAL] registered {SERVICE_NAME}");

    let mut last_enabled_check = Instant::now() - Duration::from_secs(1);
    let mut last_reassert = Instant::now();
    loop {
        match rx.recv_timeout(Duration::from_millis(500)) {
            Ok(()) | Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => break,
        }

        if last_enabled_check.elapsed() >= Duration::from_secs(1) {
            let setting = current_enabled();
            let current = ENABLED.load(Ordering::SeqCst) != 0;
            if setting != current && !AIDL_OVERRIDE.load(Ordering::SeqCst) {
                ENABLED.store(if setting { 1 } else { 0 }, Ordering::SeqCst);
                APPLIED.store(false, Ordering::SeqCst);
            }
            last_enabled_check = Instant::now();
        }

        if last_reassert.elapsed() >= Duration::from_millis(REASSERT_MS) {
            apply(true);
            last_reassert = Instant::now();
        } else {
            apply(false);
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cc1_only_when_asleep_and_enabled() {
        assert!(!should_write_cc1(SCREEN_ON, true, false));
        assert!(should_write_cc1(SCREEN_OFF, true, false));
        assert!(should_write_cc1(SCREEN_OFF_AOD, true, false));
        assert!(should_write_cc1(SCREEN_DOZE, true, false));
        assert!(should_write_cc1(SCREEN_DOZE_SUSPEND, true, false));
        assert!(!should_write_cc1(SCREEN_DOZE, false, false));
        assert!(!should_write_cc1(SCREEN_DOZE, true, true));
    }

    #[test]
    fn parses_screen_state() {
        assert_eq!(parse_screen_state("2"), Some(SCREEN_ON));
        assert_eq!(parse_screen_state("3"), Some(SCREEN_DOZE));
        assert_eq!(parse_screen_state("0"), Some(SCREEN_OFF));
        assert_eq!(parse_screen_state("9"), None);
        assert_eq!(parse_screen_state("abc"), None);
    }
}

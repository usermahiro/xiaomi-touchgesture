package vendor.xiaomi.hardware.touchgesture;

@VintfStability
interface ITouchGesture {
    int setEnabled(int enabled);
    int getEnabled();
    int getState();
    int writeGesture(String value);
}

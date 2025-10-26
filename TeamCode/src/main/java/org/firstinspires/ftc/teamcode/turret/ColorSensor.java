package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSensor {

    // Assume we have a NormalizedColorSensor
    private final com.qualcomm.robotcore.hardware.NormalizedColorSensor normalized;

    // Detection threshold in degrees (runtime adjustable)
    private volatile float hueThresholdDegrees = 50f;

    public ColorSensor(HardwareMap hardwareMap, String deviceName) {
        com.qualcomm.robotcore.hardware.NormalizedColorSensor n = null;
        try {
            n = hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, deviceName);
        } catch (Exception ignored) { }
        this.normalized = n; // may be null if misconfigured
    }

    /** Detection based on hue: true if hue (degrees) > threshold. */
    public boolean detection() {
        return getHueDegrees() > hueThresholdDegrees;
    }

    /** Returns brightness as the max 8-bit RGB channel (0..255). */
    public int getBrightness8bit() {
        if (normalized == null) return 0;
        com.qualcomm.robotcore.hardware.NormalizedRGBA rgba = normalized.getNormalizedColors();
        int Ri = clampTo8bit(Math.round(rgba.red * 255f));
        int Gi = clampTo8bit(Math.round(rgba.green * 255f));
        int Bi = clampTo8bit(Math.round(rgba.blue * 255f));
        return Math.max(Ri, Math.max(Gi, Bi));
    }

    /** Returns hue in degrees [0..360). */
    public float getHueDegrees() {
        if (normalized == null) return 0f;
        com.qualcomm.robotcore.hardware.NormalizedRGBA rgba = normalized.getNormalizedColors();
        int color = rgba.toColor(); // ARGB 8-bit packed int
        float[] hsv = new float[3];
        android.graphics.Color.colorToHSV(color, hsv);
        return hsv[0];
    }

    /** Gets the current hue threshold (degrees). */
    public float getHueThresholdDegrees() { return hueThresholdDegrees; }

    /** Sets the hue threshold (degrees). Values are clamped to [0, 360]. */
    public void setHueThresholdDegrees(float degrees) {
        if (Float.isNaN(degrees) || Float.isInfinite(degrees)) return;
        hueThresholdDegrees = Math.max(0f, Math.min(360f, degrees));
    }

    private static int clampTo8bit(int v) { return Math.max(0, Math.min(255, v)); }
}

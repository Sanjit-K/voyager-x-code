package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.LinkedList;
import java.util.Queue;

public class ColorSensor {

    // Assume we have a NormalizedColorSensor
    private final com.qualcomm.robotcore.hardware.NormalizedColorSensor normalized;

    private volatile float hueThresholdDegrees = 5f;

    // For delayed detection
    private volatile long delayMillis = 100; // Default 100ms delay
    private final Queue<HueSnapshot> hueHistory = new LinkedList<>();
    private static final int MAX_HISTORY_SIZE = 100; // Prevent unbounded growth

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

    /**
     * Delayed detection: stores current hue and checks against hue from delayMillis ago.
     * Call this method periodically to update the history buffer.
     * Returns true if the hue from delayMillis ago was > threshold.
     */
    public boolean delayedDetection() {
        long currentTime = System.currentTimeMillis();
        float currentHue = getHueDegrees();

        // Add current hue to history
        synchronized (hueHistory) {
            hueHistory.add(new HueSnapshot(currentTime, currentHue));

            // Limit history size
            while (hueHistory.size() > MAX_HISTORY_SIZE) {
                hueHistory.poll();
            }

            // Remove entries older than we need
            long cutoffTime = currentTime - (delayMillis * 2); // Keep 2x delay for safety
            while (!hueHistory.isEmpty() && hueHistory.peek() != null && hueHistory.peek().timestamp < cutoffTime) {
                hueHistory.poll();
            }

            // Find the hue value from approximately delayMillis ago
            long targetTime = currentTime - delayMillis;
            float delayedHue = currentHue; // Default to current if no history

            // If we don't have enough history yet, use current hue
            if (hueHistory.size() < 2) {
                return delayedHue > hueThresholdDegrees;
            }

            // Find the snapshot closest to but not after targetTime
            HueSnapshot closestSnapshot = null;
            for (HueSnapshot snapshot : hueHistory) {
                if (snapshot.timestamp <= targetTime) {
                    closestSnapshot = snapshot;
                } else {
                    // We've gone past the target time
                    break;
                }
            }

            if (closestSnapshot != null) {
                delayedHue = closestSnapshot.hue;
            }

            return delayedHue > hueThresholdDegrees;
        }
    }

    /** Gets the delay time in milliseconds for delayed detection. */
    public long getDelayMillis() { return delayMillis; }

    /** Sets the delay time in milliseconds for delayed detection. */
    public void setDelayMillis(long millis) {
        if (millis < 0) return;
        this.delayMillis = millis;
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

    /** Helper class to store hue snapshots with timestamps. */
    private static class HueSnapshot {
        final long timestamp;
        final float hue;

        HueSnapshot(long timestamp, float hue) {
            this.timestamp = timestamp;
            this.hue = hue;
        }
    }
}

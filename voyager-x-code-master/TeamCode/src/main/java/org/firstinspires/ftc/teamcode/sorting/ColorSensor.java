package org.firstinspires.ftc.teamcode.sorting;

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.LinkedList;
import java.util.Queue;

public class ColorSensor {

    // Assume we have a NormalizedColorSensor
    private final com.qualcomm.robotcore.hardware.NormalizedColorSensor normalized;

    private static final float purpleThreshold = 5f;
    private static final float greenThreshold = 80f;

    // For delayed detection
    private volatile long delayMillis = 200; // Default 200ms delay
    private final Queue<HueSnapshot> hueHistory = new LinkedList<>();
    private static final int MAX_HISTORY_SIZE = 1000; // Allow ~10 seconds at 50Hz (500 * 20ms = 10000ms)

    public ColorSensor(HardwareMap hardwareMap, String deviceName) {
        com.qualcomm.robotcore.hardware.NormalizedColorSensor n = null;
        try {
            n = hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, deviceName);
        } catch (Exception ignored) { }
        this.normalized = n; // may be null if misconfigured
    }

    /** Detection based on hue: true if hue (degrees) > threshold. */
    public char detection() {
        if (getHueDegrees() > greenThreshold) return 'G';
        else if (getHueDegrees() > purpleThreshold) return 'P';
        else return '_';
    }

    public char detection(float hue){
        if (hue > greenThreshold) return 'G';
        else if (hue > purpleThreshold) return 'P';
        else return '_';
    }


    /**
     * Delayed detection: stores current hue and checks against hue from delayMillis ago.
     * Call this method periodically to update the history buffer.
     * Returns true if the hue from delayMillis ago was > threshold.
     */
    public char delayedDetection() {
        long currentTime = System.currentTimeMillis();
        float currentHue = getHueDegrees();

        synchronized (hueHistory) {
            // Add current reading to queue
            hueHistory.add(new HueSnapshot(currentTime, currentHue));

            // Limit history size to prevent unbounded growth
            while (hueHistory.size() > MAX_HISTORY_SIZE) {
                hueHistory.poll(); // Remove oldest
            }

            // Calculate the target time we want to check (delayMillis ago)
            long targetTime = currentTime - delayMillis;

            // Remove entries that are TOO old (more than 2x the delay, to save memory)
            long cutoffTime = currentTime - (delayMillis * 2);
            while (!hueHistory.isEmpty() && hueHistory.peek().timestamp < cutoffTime) {
                hueHistory.poll();
            }

            // Now search through the queue to find the entry closest to targetTime
            if (hueHistory.isEmpty()) {
                return '_'; // No history available
            }

            HueSnapshot closestSnapshot = null;
            long closestDiff = Long.MAX_VALUE;

            for (HueSnapshot snapshot : hueHistory) {
                long diff = Math.abs(snapshot.timestamp - targetTime);
                if (diff < closestDiff) {
                    closestDiff = diff;
                    closestSnapshot = snapshot;
                }
            }

            if (closestSnapshot == null) {
                return '_'; // Shouldn't happen but safety check
            }

            float delayedHue = closestSnapshot.hue;
            return detection(delayedHue);
        }
    }

    /** Gets the delay time in milliseconds for delayed detection. */
    public long getDelayMillis() { return delayMillis; }

    /** Sets the delay time in milliseconds for delayed detection. */
    public void setDelayMillis(long millis) {
        if (millis < 0) return;
        this.delayMillis = millis;
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

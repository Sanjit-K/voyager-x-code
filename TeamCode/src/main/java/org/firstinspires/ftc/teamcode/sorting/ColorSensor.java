package org.firstinspires.ftc.teamcode.sorting;

import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.LinkedList;
import java.util.Queue;

public class ColorSensor {

    // Assume we have a NormalizedColorSensor
    private final com.qualcomm.robotcore.hardware.NormalizedColorSensor normalized;

    private static final float purpleHue = 240f;
    private static final float greenHue = 150f;
    private static final float TOLERANCE = 40f;

    public ColorSensor(HardwareMap hardwareMap, String deviceName) {
        normalized = hardwareMap.get(com.qualcomm.robotcore.hardware.NormalizedColorSensor.class, deviceName);
    }

    /** Detection based on hue: true if hue (degrees) > threshold. */
    public char detection() {
        if (Math.abs(getHueDegrees()-greenHue) < TOLERANCE) return 'G';
        else if (Math.abs(getHueDegrees()-purpleHue) < TOLERANCE) return 'P';
        else return 'X';
    }

    public char detection(float greenHue, float purpleHue){
        if (Math.abs(getHueDegrees()-greenHue) < TOLERANCE) return 'G';
        else if (Math.abs(getHueDegrees()-purpleHue) < TOLERANCE) return 'P';
        else return 'X';
    }

    public char detection(float hue){
        if (Math.abs(hue-greenHue) < TOLERANCE) return 'G';
        else if (Math.abs(hue-purpleHue) < TOLERANCE) return 'P';
        else return 'X';
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
}

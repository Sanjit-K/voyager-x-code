package org.firstinspires.ftc.teamcode.sorting;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensor {
    private final com.qualcomm.robotcore.hardware.DistanceSensor sensor;

    // Default detection threshold in centimeters
    private static final double DEFAULT_THRESHOLD_CM = 2;

    public DistanceSensor(HardwareMap hardwareMap, String name) {
        sensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, name);
    }

    /**
     * Get the current distance reading in centimeters
     */
    public double getDistance() {
        return sensor.getDistance(DistanceUnit.CM);
    }

    /**
     * Get the current distance reading in inches
     */
    public double getDistanceInches() {
        return sensor.getDistance(DistanceUnit.INCH);
    }

    /**
     * Get the current distance reading in millimeters
     */
    public double getDistanceMM() {
        return sensor.getDistance(DistanceUnit.MM);
    }

    /**
     * Check if an object is detected within the default threshold
     */
    public boolean isDetected() {
        return getDistance() < DEFAULT_THRESHOLD_CM;
    }

    /**
     * Check if an object is detected within a custom threshold (in cm)
     */
    public boolean isDetected(double thresholdCM) {
        return getDistance() < thresholdCM;
    }

    /**
     * Check if an object is within a specific range (in cm)
     */
    public boolean isInRange(double minCM, double maxCM) {
        double distance = getDistance();
        return distance >= minCM && distance <= maxCM;
    }
}

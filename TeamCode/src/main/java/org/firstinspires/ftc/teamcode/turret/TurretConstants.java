package org.firstinspires.ftc.teamcode.turret;

/**
 * Simple turret constants class for calculating bearing to goal
 */
public class TurretConstants {
    // Goal location (field coordinates in inches)
    public static final double X0 = 0.0    ; // goal X position
    public static final double Y0 = 144.0; // goal Y position

    // Robot position (updated from Pedro Pathing)
    private double robotX = 0.0;
    private double robotY = 0.0;

    // Calculated values
    private double bearingRad = 0.0;
    private double bearingDeg = 0.0;
    private double distance = 0.0;

    /**
     * Update robot position from Pedro Pathing
     */
    public void setRobotPose(double x, double y) {
        this.robotX = x;
        this.robotY = y;
    }

    /**
     * Calculate bearing and distance to goal
     */
    public void update() {
        // Calculate bearing from robot to goal
        bearingRad = Math.atan2(Y0 - robotY, X0 - robotX);
        bearingDeg = Math.toDegrees(bearingRad);

        // Calculate distance to goal
        distance = Math.sqrt(Math.pow(X0 - robotX, 2) + Math.pow(Y0 - robotY, 2));
    }

    // Getters
    public double getBearingDeg() { return bearingDeg; }
    public double getDistance() { return distance; }

    /**
     * Dummy method for target RPM - you can implement actual calculation later
     */
    public double getTargetRPM() {
        return 1000.0; // placeholder value
    }
}

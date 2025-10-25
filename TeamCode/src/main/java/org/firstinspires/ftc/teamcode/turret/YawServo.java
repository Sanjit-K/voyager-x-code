package org.firstinspires.ftc.teamcode.turret;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Discrete servo yaw control that aims at a target.
 * Uses Pedro Pathing Follower pose in center-origin inches.
 */
public class YawServo {

    private final Servo servo;
    private final Follower follower;
    // Servo calibration
    // On your robot: servo position 0.5 corresponds to the mechanical 270° of the servo horn,
    // and that orientation equals 0° yaw error (i.e., "straight ahead").
    private static final double SERVO_CENTER_POS = 0.5; // pos that points the turret forward

    // Mapping degrees to servo position: ±120° yaw corresponds to ±0.5 servo units (0.0..1.0)
    // So total 240° maps to 1.0 servo range -> 1.0 / 240 deg per unit
    private static final double DEGREES_PER_FULL_POS = 255; // deg that map to full 1.0 pos span

    // Optional safety limits on yaw command (deg). Adjust if your mechanism allows more/less.
    private static final double YAW_MIN_DEG = -127.5;
    private static final double YAW_MAX_DEG =  127.5;
    private double currAngle = -90.0;

    // Last computed values for telemetry/diagnostics


    /** Preferred: provide telemetry so this class can report useful info. */
    public YawServo(HardwareMap hardwareMap, Follower follower, String servoName, boolean reversed) {
        this.follower = follower;
        this.servo = hardwareMap.get(Servo.class, servoName);
        this.servo.setDirection(reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        this.servo.setPosition(SERVO_CENTER_POS); // center at 0° yaw error (forward)
    }

    /** Back-compat: no telemetry provided. */


    /** Aim the turret at a target point (field coordinates in inches). */
    public void aimAt(double targetX, double targetY) {
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotHeadingRad = follower.getPose().getHeading(); // radians, 0 along +X

        // Bearing from robot to target in field frame (radians)
        double bearingToTarget = Math.atan2(targetY - robotY, targetX - robotX);

        // Angle error in robot frame (how much the turret must rotate). RADIANS ONLY here.
        double angleErrorRad = normalizeRad(bearingToTarget - robotHeadingRad);

        // Convert to degrees
        double angleErrorDeg = Math.toDegrees(angleErrorRad);

        // Clip to allowed yaw limits
        currAngle = Range.clip(angleErrorDeg, YAW_MIN_DEG, YAW_MAX_DEG);


        // Map degrees -> servo position
        // 0° -> SERVO_CENTER_POS, -127.5° -> ~0.0, +127.5° -> ~1.0 (with DEGREES_PER_FULL_POS=255)
        double servoPos = SERVO_CENTER_POS + (currAngle + 90) / DEGREES_PER_FULL_POS;
        servoPos = Range.clip(servoPos, 0.0, 1.0);
        servo.setPosition(servoPos);


    }

    /** Center the servo at 0° yaw error (forward). */
    public void center() { servo.setPosition(SERVO_CENTER_POS); }

    /** Current servo position. */
    public double getPosition() { return servo.getPosition(); }

    public double getCurrAngle() { return currAngle; }


    /** Last computed yaw error in degrees (after clipping). */

    // ----- helpers -----
    /** Normalize radians to (-pi, pi]. */
    private static double normalizeRad(double a) {
        while (a > Math.PI)  a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
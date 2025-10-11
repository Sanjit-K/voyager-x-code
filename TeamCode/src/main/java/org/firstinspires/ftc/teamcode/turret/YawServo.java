package org.firstinspires.ftc.teamcode.turret;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Discrete servo yaw control that aims at a target.
 * Uses Pedro Pathing Follower pose in center-origin inches.
 */
public class YawServo {

    private final Servo servo;
    private final Follower follower;

    // Servo calibration - 0° robot angle = 0.5 servo position
    private static final double SERVO_CENTER = 0.5;  // This is 0° on your robot
    private static final double DEGREES_PER_SERVO_UNIT = 240.0;  // ±120° range mapped to ±0.5 servo units

    // Limit the servo travel to ±120°
    private static final double YAW_MIN_DEG = -120.0;
    private static final double YAW_MAX_DEG = 120.0;

    private TelemetryManager telemetryM;

    public YawServo(HardwareMap hardwareMap, Follower follower, String servoName, boolean reversed) {
        this.follower = follower;
        this.servo = hardwareMap.get(Servo.class, servoName);
        this.servo.setDirection(reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        this.servo.setPosition(SERVO_CENTER); // center at 0 degrees
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    public void aimAt(double targetX, double targetY) {
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotHeadingRad = follower.getPose().getHeading(); // radians, 0 along +X

        // Global bearing robot->target in field frame
        double bearingToTarget = Math.atan2(targetY - robotY, targetX - robotX);

        // Angle error in robot frame (how much servo needs to rotate)
        double angleError = bearingToTarget - robotHeadingRad;

        // Normalize to (-π, π]
        while (angleError > Math.PI) angleError -= 2 * Math.PI;
        while (angleError < -Math.PI) angleError += 2 * Math.PI;

        // Convert to degrees
        double angleErrorDeg = Math.toDegrees(angleError);

        // Clip to safe range
        angleErrorDeg = Range.clip(angleErrorDeg, YAW_MIN_DEG, YAW_MAX_DEG);

        // Convert degrees to servo position
        // 0° -> 0.5, -120° -> 0.0, +120° -> 1.0
        double servoPos = SERVO_CENTER + (angleErrorDeg / DEGREES_PER_SERVO_UNIT);

        servo.setPosition(Range.clip(servoPos, 0.0, 1.0));

        // Debug telemetry
        telemetryM.debug("Robot X", robotX);
        telemetryM.debug("Robot Y", robotY);
        telemetryM.debug("Robot Heading (deg)", Math.toDegrees(robotHeadingRad));
        telemetryM.debug("Bearing to Target (deg)", Math.toDegrees(bearingToTarget));
        telemetryM.debug("Angle Error (deg)", angleErrorDeg);
        telemetryM.debug("Servo Position", servo.getPosition());
    }

    /** Center the servo at 0 degrees. */
    public void center() { servo.setPosition(SERVO_CENTER); }

    /** Current servo position. */
    public double getPosition() { return servo.getPosition(); }
}
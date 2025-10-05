package org.firstinspires.ftc.teamcode.turret;

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

    // Servo calibration (tune for your hardware)
    private static final double SERVO_MIN_POS = 0.1;   // position at YAW_MIN_DEG
    private static final double SERVO_MAX_POS = 0.9;   // position at YAW_MAX_DEG
    private static final double YAW_MIN_DEG   = -120.0;
    private static final double YAW_MAX_DEG   =  120.0;

    // Field geometry
    private static final double FIELD_SIZE_IN = 144.0;
    private static final double HALF_FIELD_IN = FIELD_SIZE_IN / 2.0;

    public YawServo(HardwareMap hardwareMap, Follower follower, String servoName, boolean reversed) {
        this.follower = follower;
        this.servo = hardwareMap.get(Servo.class, servoName);
        this.servo.setDirection(reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        this.servo.setPosition(0.5); // center
    }

    /** Convert bottom-left-origin inches to follower (center-origin) inches. */
    private static double toFollowerX(double blX) { return blX - HALF_FIELD_IN; }
    private static double toFollowerY(double blY) { return blY - HALF_FIELD_IN; }

    /**
     * Aim using follower-frame coords (center-origin, inches).
     */
    public void aimAt(double targetX, double targetY) {
        follower.update(); // refresh pose
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotHeadingRad = follower.getPose().getHeading(); // radians, 0 along +X

        // Global bearing robot->target in follower frame
        double bearingToTarget = Math.atan2(targetY - robotY, targetX - robotX);

        // Error in robot frame
        double angleError = bearingToTarget - robotHeadingRad;

        // Normalize to (-π, π]
        while (angleError > Math.PI)  angleError -= 2.0 * Math.PI;
        while (angleError <= -Math.PI) angleError += 2.0 * Math.PI;

        double errorDeg = Math.toDegrees(angleError);

        // Clip to servo limits and map to 0..1
        double targetAngle = Range.clip(errorDeg, YAW_MIN_DEG, YAW_MAX_DEG);
        double normalized  = (targetAngle - YAW_MIN_DEG) / (YAW_MAX_DEG - YAW_MIN_DEG);
        double servoPos    = SERVO_MIN_POS + normalized * (SERVO_MAX_POS - SERVO_MIN_POS);

        servo.setPosition(Range.clip(servoPos, 0.0, 1.0));
    }

    /**
     * Aim using bottom-left-origin field inches (0..144).
     * Handy when your targets come from field drawings or simple constants.
     */
    public void aimAtFieldBL(double blX, double blY) {
        aimAt(toFollowerX(blX), toFollowerY(blY));
    }

    /** Center the servo. */
    public void center() { servo.setPosition(0.5); }

    /** Current servo position. */
    public double getPosition() { return servo.getPosition(); }
}
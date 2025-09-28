package org.firstinspires.ftc.teamcode.turret;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Simple discrete servo yaw control for turret that aims at a target.
 */
public class YawServo {

    private final Servo servo;
    private final Follower follower;

    // Servo calibration - adjust these for your servo's physical limits
    private static final double SERVO_MIN_POS = 0.1;   // servo position at -120° (safe minimum)
    private static final double SERVO_MAX_POS = 0.9;   // servo position at +120° (safe maximum)
    private static final double YAW_MIN_DEG = -120.0;  // leftmost angle in degrees
    private static final double YAW_MAX_DEG = 120.0;   // rightmost angle in degrees

    public YawServo(HardwareMap hardwareMap, Follower follower, String name, boolean reversed) {
        this.follower = follower;
        this.servo = hardwareMap.get(Servo.class, name);
        this.servo.setDirection(reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        this.servo.setPosition(0.5); // Start at center
    }

    /**
     * Update servo to aim at target (field coordinates)
     * @param targetX target X in field inches
     * @param targetY target Y in field inches
     */
    public void aimAt(double targetX, double targetY) {
        // Get current robot position and heading
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotHeadingRad = follower.getPose().getHeading();

        // Calculate angle to target from robot's perspective
        double bearingToTarget = Math.atan2(targetY - robotY, targetX - robotX);
        double angleError = bearingToTarget - robotHeadingRad;

        // Normalize angle to -π to π
        while (angleError > Math.PI) angleError -= 2 * Math.PI;
        while (angleError < -Math.PI) angleError += 2 * Math.PI;

        double errorDegrees = Math.toDegrees(angleError);

        // Clamp angle to servo's physical limits
        double targetAngle = Range.clip(errorDegrees, YAW_MIN_DEG, YAW_MAX_DEG);

        // Convert angle to servo position (0.0 to 1.0)
        double normalizedAngle = (targetAngle - YAW_MIN_DEG) / (YAW_MAX_DEG - YAW_MIN_DEG);
        double servoPosition = SERVO_MIN_POS + normalizedAngle * (SERVO_MAX_POS - SERVO_MIN_POS);

        // Set servo position
        servo.setPosition(Range.clip(servoPosition, 0.0, 1.0));
    }

    /**
     * Set servo to center position
     */
    public void center() {
        servo.setPosition(0.5);
    }

    /**
     * Get current servo position
     */
    public double getPosition() {
        return servo.getPosition();
    }
}

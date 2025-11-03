package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    // -------------------- Shared driving/pathing knobs --------------------
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.2)
            .forwardZeroPowerAcceleration(-30.186281715490008)
            .lateralZeroPowerAcceleration(-61.8071580)
            .useSecondaryDrivePIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(82.6)
            .yVelocity(68.92)
            .useBrakeModeInTeleOp(true);

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(0.001985)
            .strafeTicksToInches(0.001987)
            .turnTicksToInches(0.001965)
            .leftPodY(4.25)
            .rightPodY(-4.25)
            .strafePodX(0)
            .leftEncoder_HardwareMapName("leftFront")
            .rightEncoder_HardwareMapName("rightRear")
            .strafeEncoder_HardwareMapName("rightFront")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP));

    // -------------------- Alliance offsets (field-centric) --------------------
    public static final double ALLIANCE_OFFSET_BLUE = Math.toRadians(180.0);
    public static final double ALLIANCE_OFFSET_RED  = Math.toRadians(0.0);

    // -------------------- Common max-power presets (used in Auto) --------------------
    public static final double DEFAULT_MAX_POWER = 1.0;   // fast travel
    public static final double DRIVE_MAX_POWER   = 0.50;  // controlled approach / scoring
    public static final double PICKUP_MAX_POWER  = 0.15;  // slow intake sweep

    // -------------------- Auto → TeleOp pose handoff --------------------
    /** Set by Auto at the end so TeleOp can start from the true end pose. */
    public static Pose lastAutoEndPose = null;

    /** A sensible TeleOp fallback if Auto didn’t set a pose (e.g., skipped or stopped early). */
    public static final Pose DEFAULT_TELE_START_BLUE = new Pose(62, 33, Math.toRadians(180));
    public static final Pose DEFAULT_TELE_START_RED  = new Pose(62, 33, Math.toRadians(0));

    /**
     * Convenience: in TeleOp.init(), call:
     * {@code Constants.applyTeleStartPose(follower, Constants.DEFAULT_TELE_START_BLUE);}
     * or RED variant as needed.
     */
    public static void applyTeleStartPose(Follower follower, Pose fallback) {
        Pose start = (lastAutoEndPose != null) ? lastAutoEndPose : fallback;
        if (start != null && follower != null) {
            follower.setStartingPose(start);
        }
    }

    // -------------------- Follower factory --------------------
    public static Follower createFollower(HardwareMap hardwareMap){
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelIMULocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}

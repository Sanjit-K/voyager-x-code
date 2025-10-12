package org.firstinspires.ftc.teamcode.drive.opmode.teleop;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.turret.AprilTagScanner;
import org.firstinspires.ftc.teamcode.turret.YawServo;
import org.firstinspires.ftc.teamcode.turret.TurretConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class ExampleTeleOp extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(72, 9, Math.toRadians(90));
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private CRServo servo1, servo2, servo3, servo4;
    private DcMotorEx motorL, motorR;
    private final double servoPower = 1.0;
    private double turretPower = 0.5;
    private boolean turretSpinning = false;

    // Turret helpers
    private YawServo yawServo;
    private TurretConstants turretConstants;
    private AprilTagScanner aprilTagScanner;

    // Camera exposure settings (adjustable)
    private int cameraExposure = 6;  // milliseconds (1-100)
    private int cameraGain = 250;     // ISO gain (0-255)

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower.setStartingPose(startingPose);

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");
        servo3 = hardwareMap.get(CRServo.class, "servo3");
        servo4 = hardwareMap.get(CRServo.class, "servo4");

        motorL = hardwareMap.get(DcMotorEx.class, "turretL");
        motorR = hardwareMap.get(DcMotorEx.class, "turretR");

        motorL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Turret initialization
        turretConstants = new TurretConstants();
        // Assumption: yaw servo is named "yawServo" in hardware map. Change name if different.
        try {
            yawServo = new YawServo(hardwareMap, follower, "yawServo", false);
        } catch (Exception e) {
            // If the hardware map doesn't contain the servo name, yawServo will remain null; telemetry will show this.
            yawServo = null;
        }

        // Initialize AprilTag scanner
        // Change "Webcam 1" to match your webcam name in the hardware configuration
        try {
            aprilTagScanner = new AprilTagScanner(hardwareMap, "Webcam 1");
            telemetryM.debug("AprilTag Scanner", "Initialized");
        } catch (Exception e) {
            aprilTagScanner = null;
            telemetryM.debug("AprilTag Scanner", "Failed to initialize: " + e.getMessage());
        }
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();

        // Update turret constants with current robot pose (field inches)
        turretConstants.setRobotPose(follower.getPose().getX(), follower.getPose().getY());
        turretConstants.update();

        // Update yaw servo to point at configured field goal
        if (yawServo != null) {
            yawServo.aimAt(TurretConstants.X0, TurretConstants.Y0);
        }

        // AprilTag scanning and telemetry
        if (aprilTagScanner != null) {
            telemetryM.debug("AprilTags Detected", aprilTagScanner.getDetectionCount());

            // Debug: Show what IDs are actually being detected
            for (AprilTagDetection detection : aprilTagScanner.getDetections()) {
                telemetryM.debug("DEBUG Detected ID", detection.id);
                telemetryM.debug("  Decision Margin", String.format("%.1f", detection.decisionMargin));
                telemetryM.debug("  Hamming Distance", detection.hamming);
                telemetryM.debug("  Range", String.format("%.1f inches", detection.ftcPose.range));
                telemetryM.debug("  Bearing", String.format("%.0f degrees", Math.toDegrees(detection.ftcPose.bearing)));
                telemetryM.debug("  Yaw", String.format("%.0f degrees", Math.toDegrees(detection.ftcPose.yaw)));
                telemetryM.debug("  X", String.format("%.1f inches", detection.ftcPose.x));
                telemetryM.debug("  Y", String.format("%.1f inches", detection.ftcPose.y));
                telemetryM.debug("  Z", String.format("%.1f inches", detection.ftcPose.z));

                // If the tag has metadata (name), display it
                if (detection.metadata != null) {
                    telemetryM.debug("  Tag Name", detection.metadata.name);
                } else {
                    telemetryM.debug("  Tag Name", "UNKNOWN - Not in FTC library!");
                }
            }

            // Show if no tags detected
            if (aprilTagScanner.getDetectionCount() == 0) {
                telemetryM.debug("Camera Status", "No AprilTags visible");
            }
        } else {
            telemetryM.debug("AprilTag Scanner", "NOT INITIALIZED");
        }

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    false // Robot Centric
            );
        }

        //Automated PathFollowing
        if (gamepad1.bWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.aWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }

        if (gamepad1.leftBumperWasPressed()) {
            turretSpinning = !turretSpinning;
        }

        if (turretSpinning) {
            motorL.setPower(turretPower);
            motorR.setPower(turretPower);
        } else {
            motorL.setPower(0);
            motorR.setPower(0);
        }

        if (gamepad1.dpadUpWasPressed()) {
            turretPower = Math.min(turretPower + 0.05, 1.0);
        }
        if (gamepad1.dpadDownWasPressed()) {
            turretPower = Math.max(turretPower - 0.05, 0.0);
        }

        // Continuous servos - user previously wanted these to run; left as always-on. If you want them toggled, we can change.
        servo1.setPower(servoPower);
        servo2.setPower(-servoPower);
        servo3.setPower(servoPower);
        servo4.setPower(-servoPower);

        // Camera exposure and gain adjustment
        if (gamepad2.dpad_left) {
            cameraExposure = Math.max(cameraExposure - 1, 1);
        }
        if (gamepad2.dpad_right) {
            cameraExposure = Math.min(cameraExposure + 1, 100);
        }
        if (gamepad2.dpad_up) {
            cameraGain = Math.min(cameraGain + 5, 255);
        }
        if (gamepad2.dpad_down) {
            cameraGain = Math.max(cameraGain - 5, 0);
        }

        // Apply camera settings
        if (aprilTagScanner != null) {
            aprilTagScanner.setCameraExposure(cameraExposure);
            aprilTagScanner.setCameraGain(cameraGain);
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.debug("Turret Power", turretPower);
        telemetryM.debug("Goal Bearing (deg)", turretConstants.getBearingDeg());
        telemetryM.debug("Distance to Goal", turretConstants.getDistance());
        telemetryM.debug("Target RPM (est)", turretConstants.getTargetRPM());
        telemetryM.debug("Camera Exposure (ms)", cameraExposure);
        telemetryM.debug("Camera Gain (ISO)", cameraGain);
        telemetryM.debug("Camera FPS", aprilTagScanner.getCameraFPS());
        telemetryM.debug("Camera State", aprilTagScanner.getCameraState());

        // Update telemetry at the very end so all data is displayed
        telemetryM.update(telemetry);
    }
}

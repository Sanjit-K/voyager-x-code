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

import org.firstinspires.ftc.teamcode.intake.IntakeServos;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.turret.AprilTagScanner;
import org.firstinspires.ftc.teamcode.turret.LaunchMotors;
import org.firstinspires.ftc.teamcode.turret.LaunchServos;
import org.firstinspires.ftc.teamcode.turret.PivotServo;
import org.firstinspires.ftc.teamcode.turret.YawServo;
import org.firstinspires.ftc.teamcode.turret.TurretConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class LauncherTeleop extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(72, 9, Math.toRadians(90));
    private boolean automatedDrive;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private CRServo servo1, servo2, servo3, servo4;
    private DcMotorEx motorL, motorR;
    private final double servoPower = 1.0;
    private double turretPower = 0.5;
    private boolean turretSpinning = false;
    private boolean frontSpinning = false;
    private boolean backSpinning = false;

    // Additional toggle states for new controls
    private boolean bottomFrontIntakeActive = false;
    private boolean topFrontIntakeActive = false;
    private boolean launchServosActive = false;
    private boolean launchMotorsActive = false;

    // Turret helpers
    private YawServo yawServo;
    private TurretConstants turretConstants;
    private IntakeServos intakeServos;
    private PivotServo pivotServo;
    private LaunchServos launchServos;
    private LaunchMotors launchMotors;

    /**
     * Generic toggle helper function to make code cleaner
     * @param currentState The current state (true/false)
     * @param enableAction What to do when enabling (turning on)
     * @param disableAction What to do when disabling (turning off)
     * @return The new toggled state
     */
    private boolean toggle(boolean currentState, Runnable enableAction, Runnable disableAction) {
        if (!currentState) {
            enableAction.run();
        } else {
            disableAction.run();
        }
        return !currentState;
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

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

        intakeServos = new IntakeServos(hardwareMap, "servo1", "servo2", "servo3", "servo4", "servo5", "servo6", "servo7", "servo8");
        pivotServo = new PivotServo(hardwareMap, "yawServo");
        launchServos = new LaunchServos(hardwareMap, "servoL", "servoR");
        launchMotors = new LaunchMotors(hardwareMap, follower, "turretL", "turretR");
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

//        // Update yaw servo to point at configured field goal
//        if (yawServo != null) {
//            yawServo.aimAt(TurretConstants.X0, TurretConstants.Y0);
//        }

        if (gamepad1.dpadLeftWasPressed()){
            double currPos = pivotServo.getPos();
            pivotServo.set(Math.max(0.0, currPos - 0.05));
        }

        if (gamepad1.dpadRightWasPressed()){
            double currPos = pivotServo.getPos();
            pivotServo.set(Math.min(1.0, currPos + 0.05));
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
        if (gamepad1.bWasPressed()){
            frontSpinning = toggle(frontSpinning,
                    intakeServos::enableBottomFrontIntake,
                    intakeServos::disableBottomFrontIntake
            );
        }

        if (gamepad1.aWasPressed()){
            backSpinning = toggle(backSpinning,
                    intakeServos::enableTopFrontIntake,
                    intakeServos::disableTopFrontIntake
            );
        }

        // Toggle launch servos (Right Trigger - simple toggle)
        if (gamepad1.xWasPressed()) {
            launchServosActive = toggle(launchServosActive,
                    launchServos::enable,
                    launchServos::disable
            );

        }

        // Toggle launch motors (Left Trigger - simple toggle)


        if (gamepad1.leftBumperWasPressed()) {
            turretSpinning = !turretSpinning;
        }

        if (turretSpinning) {
            launchMotors.set(turretPower);
        } else {
            launchMotors.set(0);
        }

        if (gamepad1.dpadUpWasPressed()) {
            turretPower = Math.min(turretPower + 0.01, 1.0);
        }
        if (gamepad1.dpadDownWasPressed()) {
            turretPower = Math.max(turretPower - 0.01, 0.0);
        }


        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.debug("Turret Power", turretPower);
        telemetryM.debug("Goal Bearing (deg)", turretConstants.getBearingDeg());
        telemetryM.debug("Distance to Goal", turretConstants.getDistance());
        telemetryM.debug("Target RPM (est)", turretConstants.getTargetRPM());


        // Update telemetry at the very end so all data is displayed
        telemetryM.update(telemetry);
    }
}

package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.intake.IntakeServos;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.turret.ColorSensor;
import org.firstinspires.ftc.teamcode.turret.LaunchMotors;
import org.firstinspires.ftc.teamcode.turret.LaunchServos;
import org.firstinspires.ftc.teamcode.turret.RobotHeading;
import org.firstinspires.ftc.teamcode.turret.YawServo;
import org.firstinspires.ftc.teamcode.turret.TurretConstants;

@Configurable
@TeleOp
public class BlueTeleOp extends OpMode {
    private Follower follower;
    public static Pose startingPose = new Pose(62, 33, Math.toRadians(180));
    private boolean automatedDrive;
    private TelemetryManager telemetryM;
    private ColorSensor colorSensor;
    private double turretPower = 0.5;
    private boolean turretSpinning = false;

    // Turret helpers
    private YawServo yawServo;
    private TurretConstants turretConstants;
    private IntakeServos intakeServos;
    private LaunchServos launchServos;
    private LaunchMotors launchMotors;
    private RobotHeading robotHeading; // helper to aim at goal
    private boolean frontSpinning = false;
    private boolean backSpinning = false;
    private boolean launchServosActive = false;
    private boolean intakeDirection = true; // true = center/forward, false = back

    // Alliance POV offset: 180 = Blue, 0 = Red
    private final double offset = Math.toRadians(180.0);

    private boolean toggle(boolean currentState, Runnable enableAction, Runnable disableAction) {
        if (!currentState) enableAction.run(); else disableAction.run();
        return !currentState;
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower.setStartingPose(startingPose);

        intakeServos = new IntakeServos(hardwareMap, "servo1", "servo2", "servo3", "servo4", "servo5", "servo6", "servo7", "servo8");
        launchServos = new LaunchServos(hardwareMap, "servoL", "servoR");
        launchMotors = new LaunchMotors(hardwareMap, follower, "turretL", "turretR");

        // Turret initialization
        turretConstants = new TurretConstants();
        // Assumption: yaw servo is named "yawServo" in hardware map. Change name if different.
        try {
            yawServo = new YawServo(hardwareMap, follower, "yawServo", false);
        } catch (Exception e) {
            // If the hardware map doesn't contain the servo name, yawServo will remain null; telemetry will show this.
            yawServo = null;
        }

        robotHeading = new RobotHeading(follower);
        if (yawServo != null) yawServo.center();

        // Initialize color sensor and snapshot background (expects a device named "color")
        try {
            colorSensor = new ColorSensor(hardwareMap, "color");
            telemetryM.debug("Color Sensor", "initialized");
        } catch (Exception e) {
            colorSensor = null;
            telemetryM.debug("Color Sensor", "NOT FOUND: check config name 'color'");
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



        if (yawServo != null) {
            yawServo.center();
        }

        // Early exit back to teleop when automatedDrive finishes
        if (automatedDrive && !follower.isBusy()) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
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

            if (gamepad1.dpadRightWasPressed()){
                intakeDirection = toggle(intakeDirection,
                        yawServo::back,
                        yawServo::center
                );
            }

            // Toggle launch servos (Right Trigger - simple toggle)
            if (gamepad1.xWasPressed()) {
                launchServosActive = toggle(launchServosActive,
                        launchServos::enable,
                        launchServos::disable
                );

            }
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y ,
                    -gamepad1.left_stick_x ,
                    -gamepad1.right_stick_x ,
                    false,
                    offset
            );


            //Slow Mode
            if (gamepad1.rightBumperWasPressed()) {
                if (robotHeading != null) {
                    robotHeading.aimAtGoal();
                    automatedDrive = true; // let follower control until done
                }
            }

            // Turret spin toggle
            if (gamepad1.leftBumperWasPressed()) {
                turretSpinning = toggle(
                        turretSpinning,
                        () -> launchMotors.set(turretPower),
                        () -> launchMotors.set(0.0)
                );
            }

            if (gamepad1.dpadUpWasPressed()) turretPower = Math.min(turretPower + 0.05, 1.0);
            if (gamepad1.dpadDownWasPressed()) turretPower = Math.max(turretPower - 0.05, 0.0);

            // Telemetry
            telemetryM.debug("position", follower.getPose());
            telemetryM.debug("velocity", follower.getVelocity());
            telemetryM.debug("automatedDrive", automatedDrive);
            telemetryM.debug("Turret Power", turretPower);
            telemetryM.debug("Goal Bearing (deg)", turretConstants.getBearingDeg());
            telemetryM.debug("Distance to Goal", turretConstants.getDistance());
            telemetryM.debug("Target RPM (est)", turretConstants.getTargetRPM());
            telemetryM.debug("Yaw Servo Pos", yawServo != null ? yawServo.getPosition() : "N/A");


            // Minimal color sensor telemetry (brightness + detection)
            boolean detected = false;
            int brightness = 0;
            if (colorSensor != null) {
                brightness = colorSensor.getBrightness8bit();
                detected = colorSensor.detection();
            }

            // Use detection in place of purple/green logic
            if (detected && gamepad1.left_trigger < 0.5){
                launchServos.disable();
            } else {
                launchServos.enable();
            }

            if (detected && gamepad1.left_trigger > 0.5){
                launchServos.enable();
            }
            telemetryM.debug("Brightness(0-255)", brightness);
            telemetryM.debug("Detected", detected);

            telemetryM.update(telemetry);
        }
    }
}

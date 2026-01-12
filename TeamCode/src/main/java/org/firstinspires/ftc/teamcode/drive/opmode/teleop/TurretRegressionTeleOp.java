package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;
import org.firstinspires.ftc.teamcode.shooting.Turret;
import org.firstinspires.ftc.teamcode.sorting.ColorSensor;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;

@TeleOp(name = "Turret Regression Test", group = "TeleOp")
public class TurretRegressionTeleOp extends OpMode {

    private Limelight3A limelight;
    private Follower follower;
    private static final Pose startingPose = new Pose(7.5, 7.75, Math.toRadians(0));
    private BarIntake barIntake;
    private Spindexer spindexer;
    private KickerServo kickerServo;
    private Turret turret;
    private ColorSensor colorSensor;
    private LynxModule expansionHub;

    private ElapsedTime outtakeTimer;
    private static final double OUTTAKE_DELAY_MS = 500;

    // Outtake routine state
    private boolean outtakeInProgress = false;
    private int outtakeAdvanceCount = 0;
    private double lastAdvanceTime = 0;

    // RPM Control
    private double currentRPM = 2500.0;

    private static final double OFFSET = Math.toRadians(180.0);

    // Configuration for distance calculation
    // Height difference (h2-h1) is 12 inches
    private static final double HEIGHT_DIFFERENCE = 12.5;
    // Limelight mounted at 0 degrees (parallel to floor)
    private static final double MOUNT_ANGLE = 0.0;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        follower = Constants.createFollower(hardwareMap);
        barIntake = new BarIntake(hardwareMap, "barIntake", true);
        colorSensor = new ColorSensor(hardwareMap, "colorSensor");
        spindexer = new Spindexer(hardwareMap, "spindexerMotor", "spindexerAnalog", "distanceSensor", colorSensor);
        kickerServo = new KickerServo(hardwareMap, "kickerServo");
        turret = new Turret(hardwareMap, "shooter", "turret", "turretEncoder", "transferMotor", false, true, true);

        outtakeTimer = new ElapsedTime();

        follower.setStartingPose(startingPose);
        turret.setShooterRPM(currentRPM);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        turret.on();
        barIntake.spinIntake();
    }

    @Override
    public void loop() {
        expansionHub.clearBulkCache();
        follower.update();

        // Drive Control
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false,
                OFFSET);

        // Intake / Spindexer Control
        if (gamepad1.aWasPressed()) {
            barIntake.spinIntake();
        } else if (gamepad1.bWasPressed()) {
            barIntake.spinOuttake();
        }

        if (gamepad1.rightBumperWasPressed()) {
            spindexer.advanceIntake();
        } else if (gamepad1.leftBumperWasPressed()) {
            spindexer.retreatIntake();
        }

        if (gamepad1.xWasPressed()){
            spindexer.clearTracking();
            barIntake.spinIntake();
        }

        // Limelight Distance & RPM Logic
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleToGoalDegrees = MOUNT_ANGLE + ty;
            double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

            // Calculate distance using: d = (h2 - h1) / tan(a + ty)
            // Handle edge case where angle is near 0
            if (Math.abs(angleToGoalRadians) > 0.001) {
                double distance = HEIGHT_DIFFERENCE / Math.tan(angleToGoalRadians);

                // Regression: rpm = 0.001519*x^2 + 6.23011*x + 1876.19
                // where x is distance in inches
                double calculatedRPM = 0.001519 * distance * distance
                                     + 6.23011 * distance
                                     + 1876.19;

                // Update RPM
                currentRPM = calculatedRPM;
                turret.setShooterRPM(currentRPM);
                turret.on(); // Update velocity

                telemetry.addData("Limelight ty", ty);
                telemetry.addData("Calculated Distance (in)", distance);
                telemetry.addData("Calculated RPM", calculatedRPM);
            } else {
                telemetry.addData("Limelight ty", ty);
                telemetry.addData("Calculated Distance", "Infinity (angle ~0)");
            }
        } else {
             telemetry.addData("Limelight", "No Target");
        }

        if (spindexer.isFull()){
            barIntake.stop();
        }
        spindexer.update();

        // Outtake Routine Trigger
        if (gamepad1.left_trigger > 0.5 && !outtakeInProgress) {
            turret.on();
            startOuttakeRoutine();
        }

        if (outtakeInProgress) {
            handleOuttakeRoutine();
        }

        telemetry.addData("Actual RPM", turret.getShooterRPM());
        telemetry.addData("Outtake In Progress", outtakeInProgress);
        char[] filled = spindexer.getFilled();
        telemetry.addData("Filled Slots", "[" + filled[0] + ", " + filled[1] + ", " + filled[2] + "]");
        telemetry.update();
    }

    private void startOuttakeRoutine() {
        outtakeInProgress = true;
        outtakeAdvanceCount = 0;
        outtakeTimer.reset();
        lastAdvanceTime = 0;
        turret.transferOn();
        kickerServo.kick();
        spindexer.advanceIntake();
        outtakeAdvanceCount++;
        lastAdvanceTime = outtakeTimer.milliseconds();
    }

    private void handleOuttakeRoutine() {
        double currentTime = outtakeTimer.milliseconds();
        if (outtakeAdvanceCount < 3) {
            if (currentTime - lastAdvanceTime >= OUTTAKE_DELAY_MS) {
                spindexer.advanceIntake();
                outtakeAdvanceCount++;
                lastAdvanceTime = currentTime;
            }
        } else {
            if (currentTime - lastAdvanceTime >= OUTTAKE_DELAY_MS) {
                kickerServo.normal();
                spindexer.clearTracking();
                barIntake.spinIntake();
                outtakeInProgress = false;
            }
        }
    }
}


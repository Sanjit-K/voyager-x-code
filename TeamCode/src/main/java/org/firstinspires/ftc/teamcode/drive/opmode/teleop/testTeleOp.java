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

@TeleOp(name = "Test TeleOp", group = "TeleOp")
public class testTeleOp extends OpMode {
    private Follower follower;
    private static final Pose startingPose = new Pose(7.5, 7.75, Math.toRadians(0));
    private BarIntake barIntake;
    private Limelight3A limelight;
    private Spindexer spindexer;
    private KickerServo kickerServo;
    private Turret turret;
    private ColorSensor colorSensor;
    private ElapsedTime loopTimer;
    private ElapsedTime outtakeTimer;
    private LynxModule expansionHub;
    private static final double OFFSET = Math.toRadians(180.0);
    private final Pose targetPose = new Pose(144, 144, 0); // Fixed target


    // Outtake routine state
    private boolean outtakeInProgress = false;
    private boolean singleOuttakeInProgress = false;
    private boolean singleAtPosition = false;
    private int outtakeAdvanceCount = 0;
    private double lastAdvanceTime = 0;
    private static final double OUTTAKE_DELAY_MS = 500;

    private double currentRPM = 2500.0;
    private static final double HEIGHT_DIFFERENCE = 12.5;
    // Limelight mounted at 0 degrees (parallel to floor)
    private static final double MOUNT_ANGLE = 0.0;


    @Override
    public void init() {
        expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        follower = Constants.createFollower(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
        barIntake = new BarIntake(hardwareMap, "barIntake", true);
        colorSensor = new ColorSensor(hardwareMap, "colorSensor");
        spindexer = new Spindexer(hardwareMap, "spindexerMotor", "spindexerAnalog", "distanceSensor", colorSensor);
        kickerServo = new KickerServo(hardwareMap, "kickerServo");
        turret = new Turret(hardwareMap, "shooter", "turret", "turretEncoder", "transferMotor", false, false, true);
        loopTimer = new ElapsedTime();
        outtakeTimer = new ElapsedTime();

        follower.setStartingPose(startingPose);
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

        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        // Drive control
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false,
                OFFSET);

        // Intake control
        if (gamepad1.aWasPressed()) {
            barIntake.spinIntake();
        } else if (gamepad1.bWasPressed()) {
            barIntake.spinOuttake();
        }

        // Spindex control
        if (gamepad1.rightBumperWasPressed()) {
            spindexer.advanceIntake();
        } else if (gamepad1.leftBumperWasPressed()) {
            spindexer.retreatIntake();
        }


        if (gamepad1.xWasPressed()){
            spindexer.clearTracking();
            barIntake.spinIntake();
        }

        // Outtake routine trigger
        if (gamepad1.left_trigger > 0.5 && !outtakeInProgress) {
            turret.on();
            startOuttakeRoutine();
        }
        if (gamepad1.right_trigger > 0.5) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                turret.goToPosition(turret.getTurretAngle() + tx);
            }
        } else {
            turret.setTurretPower(0.0);
        }

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
                        + 1856.19;

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

        if (gamepad1.leftStickButtonWasPressed()){
            startSingleOuttake('P');
        }
        if (gamepad1.rightStickButtonWasPressed()){
            startSingleOuttake('G');
        }
        // Handle outtake routine sequence
        if (outtakeInProgress) {
            handleOuttakeRoutine();
        }
        if (singleOuttakeInProgress){
            handleSingleOuttake();
        }

        if (spindexer.isFull()){
            barIntake.stop();
        }
        // Update spindexer
        spindexer.update();


        // Spindexer diagnostic telemetry (angle, velocity, adaptive tolerance, output, etc.)

        // Telemetry
        telemetry.addData("Spindexer Index", spindexer.getIntakeIndex());
        telemetry.addData("Adaptive Tolerance", String.format(java.util.Locale.US, "%.2f", spindexer.getLastAdaptiveTol()));
        telemetry.addData("Turret RPM", String.format(java.util.Locale.US, "%.1f", turret.getShooterRPM()));
        telemetry.addData("Outtake In Progress", outtakeInProgress);
        telemetry.addData("Loop Time (ms)", String.format(java.util.Locale.US, "%.2f", loopMs));
        char[] filled = spindexer.getFilled();
        telemetry.addData("Filled Slots", "[" + filled[0] + ", " + filled[1] + ", " + filled[2] + "]");
        telemetry.update();
    }

    private void startOuttakeRoutine() {
        outtakeInProgress = true;
        outtakeAdvanceCount = 0;
        outtakeTimer.reset();
        lastAdvanceTime = 0;

        // Step 1: Turn on transfer wheel and turret wheel
        turret.transferOn();

        // Step 2: Set kicker servo to kick
        kickerServo.kick();

        // Step 3: First advanceIntake call immediately
        spindexer.advanceIntake();
        outtakeAdvanceCount++;
        lastAdvanceTime = outtakeTimer.milliseconds();
    }

    private void handleOuttakeRoutine() {
        double currentTime = outtakeTimer.milliseconds();

        // Check if it's time for the next advanceIntake call
        if (outtakeAdvanceCount < 3) {
            if (currentTime - lastAdvanceTime >= OUTTAKE_DELAY_MS) {
                spindexer.advanceIntake();
                outtakeAdvanceCount++;
                lastAdvanceTime = currentTime;
            }
        } else {
            if (currentTime - lastAdvanceTime >= OUTTAKE_DELAY_MS) {
                // All 3 advanceIntake calls completed, set kicker back to normal
                kickerServo.normal();
                spindexer.clearTracking();
                barIntake.spinIntake();
                outtakeInProgress = false;
            }
        }
    }

    private void startSingleOuttake(char color){
        int index = -1;
        char[] filled = spindexer.getFilled();
        for (int i = 0; i < 3; i++){
            if (filled[i] == color){
                index = i;
                break;
            }
        }
        if (index == -1) return;
        singleOuttakeInProgress = true;
        singleAtPosition = false;

        turret.transferOn();

        spindexer.setShootIndex(index);
    }

    private void handleSingleOuttake(){
        if (!singleAtPosition) {
            if (spindexer.isAtTarget(5.0)){
                singleAtPosition = true;
                outtakeTimer.reset();
                kickerServo.kick();
            }
        } else {
            if (outtakeTimer.milliseconds() > OUTTAKE_DELAY_MS){
                kickerServo.normal();
                spindexer.setColorAtPos('_', spindexer.getShootIndex());
                singleOuttakeInProgress = false;
                if (!spindexer.isFull()) {
                    barIntake.spinIntake();
                }
            }
        }
    }
}

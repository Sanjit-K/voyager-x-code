package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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

import java.util.List;

@TeleOp(name = "Turret RPM Test", group = "TeleOp")
public class TurretRPMTeleOp extends OpMode {

    private Limelight3A limelight; //any camera here
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
    private double targetRPM = 2500.0;

    private static final double OFFSET = Math.toRadians(180.0);

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
        turret.setShooterRPM(targetRPM);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        turret.on();
        barIntake.spinIntake();
        limelight.captureSnapshot("start");
    }

    @Override
    public void loop() {

        expansionHub.clearBulkCache();
        follower.update();

        // Drive Control (Just in case)
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false,
                OFFSET);

        // RPM Adjustment
        if (gamepad1.dpadUpWasPressed()) {
            targetRPM += 50;
            turret.setShooterRPM(targetRPM);
            turret.on(); // Update velocity
        }

        if (gamepad1.dpadDownWasPressed()) {
            targetRPM -= 50;
            turret.setShooterRPM(targetRPM);
            turret.on(); // Update velocity
        }

        if (gamepad1.dpadRightWasPressed()) {
            targetRPM += 10;
            turret.setShooterRPM(targetRPM);
            turret.on(); // Update velocity
        }

        if (gamepad1.dpadLeftWasPressed()) {
            targetRPM -= 10;
            turret.setShooterRPM(targetRPM);
            turret.on(); // Update velocity
        }


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

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                LLResultTypes.FiducialResult fr = fiducials.get(0);
                org.firstinspires.ftc.robotcore.external.navigation.Position pos = fr.getCameraPoseTargetSpace().getPosition();
                double distance = Math.sqrt(pos.x * pos.x + pos.z * pos.z);
                telemetry.addData("Limelight Distance", distance);
            }
        }


        if (spindexer.isFull()){
            barIntake.stop();
        }
        spindexer.update();

        // Outtake Routine Trigger
        if (gamepad1.left_trigger > 0.5 && !outtakeInProgress) {
            turret.on(); // Ensure shooter is on
            startOuttakeRoutine();
        }

        // Handle Outtake Routine
        if (outtakeInProgress) {
            handleOuttakeRoutine();
        }

        // Telemetry
        telemetry.addData("Target RPM", targetRPM);
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
}

package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.teleop.functions.LockMode;
import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;
import org.firstinspires.ftc.teamcode.shooting.Turret;
import org.firstinspires.ftc.teamcode.sorting.ColorSensor;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;

@TeleOp(name = "Red TeleOp", group = "TeleOp")
public class RedTeleOp extends OpMode {
    private Follower follower;
    private LockMode lockMode;
    private boolean isLocked = false;
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
    private static final double OUTTAKE_DELAY_MS = 300;

    private double currentRPM = 2500.0;


    @Override
    public void init() {
        expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        follower = Constants.createFollower(hardwareMap);
        lockMode = new LockMode(follower);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
        barIntake = new BarIntake(hardwareMap, "barIntake", true);
        colorSensor = new ColorSensor(hardwareMap, "colorSensor");
        spindexer = new Spindexer(hardwareMap, "spindexerMotor", "spindexerAnalog", "distanceSensor", colorSensor);
        kickerServo = new KickerServo(hardwareMap, "kickerServo");
        turret = new Turret(hardwareMap, "shooter", "turret", "turretEncoder", "transferMotor", false, true, true);
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

        if (isLocked) {
            lockMode.lockPosition();
        } else {
            lockMode.unlockPosition();
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false,
                    OFFSET);
        }

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
            turret.trackTarget(follower.getPose(), targetPose, 0);
        } else {
            turret.setTurretPower(0.0);
        }


        double distance = Math.sqrt((targetPose.getX() - follower.getPose().getX())
                                    * (targetPose.getX() - follower.getPose().getX())
                                    + (targetPose.getY() - follower.getPose().getY())
                                    * (targetPose.getY() - follower.getPose().getY()));

        currentRPM = 0.0151257 * distance * distance
                             + 10.03881 * distance
                             + 1382.4428;
        currentRPM = (currentRPM > 2700) ? 2700 : currentRPM;
        // Update RPM
        turret.setShooterRPM(currentRPM);
        turret.on(); // Update velocity

        if (gamepad1.right_trigger > 0.5 && Math.abs(turret.getSetShooterRPM()-turret.getShooterRPM()) < 20){
            gamepad1.rumble(100);
        }
        else {
            gamepad1.stopRumble();
        }
        telemetry.addData("Calculated Distance (in)", distance);
        telemetry.addData("Current target RPM:", currentRPM);

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
        telemetry.addData("Lock Mode Active", isLocked);
        telemetry.addData("Spindexer Index", spindexer.getIntakeIndex());
        telemetry.addData("Adaptive Tolerance", String.format(java.util.Locale.US, "%.2f", spindexer.getLastAdaptiveTol()));
        telemetry.addData("Turret RPM Error", String.format(java.util.Locale.US, "%.1f", turret.getShooterRPM() - turret.getSetShooterRPM()));
        telemetry.addData("Outtake In Progress", outtakeInProgress);
        telemetry.addData("Loop Time (ms)", String.format(java.util.Locale.US, "%.2f", loopMs));
        char[] filled = spindexer.getFilled();
        telemetry.addData("Filled Slots", "[" + filled[0] + ", " + filled[1] + ", " + filled[2] + "]");
        telemetry.update();
    }

    private void startOuttakeRoutine() {
        outtakeInProgress = true;
        isLocked = true;
        outtakeAdvanceCount = 0;
        outtakeTimer.reset();
        lastAdvanceTime = 0;
        
        
        // Step 1: Turn on transfer wheel and turret wheel
        turret.transferOn();
        
        // Step 1.5: Turn on lock mode
        
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
                isLocked = false;
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

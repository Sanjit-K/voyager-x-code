package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;
import org.firstinspires.ftc.teamcode.shooting.Turret;
import org.firstinspires.ftc.teamcode.sorting.ColorSensor;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;

import java.util.Objects;


@Autonomous(name = "Red Path Auto", group = "Autonomous")
@Configurable
public class RedPathAuto extends OpMode {

    // -------------------- Panels + Pedro --------------------
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    // -------------------- Subsystems --------------------
    private BarIntake barIntake;
    private ColorSensor colorSensor;
    private Spindexer spindexer;
    private KickerServo kickerServo;
    private Turret turret;

    // -------------------- Motifs --------------------
    private Limelight3A limelight;
    private int scannedTagId = 0;

    private static int[] getMotifForTag(int tagId) {
        switch (tagId) {
            case 21: return new int[]{2, 2, 1, 0};
            case 22: return new int[]{1, 1, 0, 2};
            case 23: return new int[]{0, 0, 2, 1};
            default: return null;
        }
    }

    private int[] order = null;
    private int currentOrderIndex = 0;
    private String currentBarIntakeState = "stop";


    // -------------------- Config --------------------
    public static double SCAN_TURRET_DEG = 110;
    public static double SHOOT_DEG = 43.5;
    public static double SHOOT_RPM = 2140;

    public static double PARK_SPEED = 1.0;

    public static double OUTTAKE_DELAY_MS =  650;
    private double targetAngle = SCAN_TURRET_DEG;

    // -------------------- State machine --------------------
    private int pathState = 0;
    private int lastState = -1;
    private final ElapsedTime stateTimer = new ElapsedTime();

    private final ElapsedTime settleTimer = new ElapsedTime();
    private boolean isSettling = false;
    private static final long SETTLE_DELAY_MS = 250;

    private void setState(int s) {
        if (s != lastState) {
            lastState = s;
            stateTimer.reset();
            isSettling = false;
        }
        pathState = s;
    }

    // -------------------- Outtake routine --------------------
    private final ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean outtakeInProgress = false;
    private int outtakeAdvanceCount = 0;
    private double lastAdvanceTime = 0.0;
    private int spinInterval = 60;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        // Start Pose from path.pp startPoint
        follower.setStartingPose(new Pose(121.396, 120.422, Math.toRadians(0)));

        // Subsystems
        barIntake = new BarIntake(hardwareMap, "barIntake", true);
        colorSensor = new ColorSensor(hardwareMap, "colorSensor");
        spindexer = new Spindexer(
                hardwareMap,
                "spindexerMotor",
                "spindexerAnalog",
                "distanceSensor",
                colorSensor
        );
        kickerServo = new KickerServo(hardwareMap, "kickerServo");
        turret = new Turret(
                hardwareMap,
                "shooter",
                "turret",
                "turretEncoder",
                "transferMotor",
                false,
                false
        );
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(100);
        limelight.start();
        spindexer.filled = new char[]{'X', 'X', 'X'};

        paths = new Paths(follower);

        kickerServo.normal();
        turret.setShooterRPM(SHOOT_RPM);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        outtakeInProgress = false;
        setState(0);

        stateTimer.reset();

        turret.on();
        turret.transferOn();
        turret.setShooterRPM(SHOOT_RPM);
        spindexer.setShootIndex(1);
    }

    @Override
    public void loop() {
        follower.update();

        turret.on();
        turret.setShooterRPM(SHOOT_RPM);
        turret.goToPosition(targetAngle);

        spindexer.update();
        if (spindexer.isFull() && !outtakeInProgress && follower.getPose().getX() < 132){
            spinInterval++;
            if ((spinInterval > 30 && spinInterval < 40))
                currentBarIntakeState = "out";
            else {
                currentBarIntakeState = "stop";
            }
        }

        // Logic for setting shoot index based on position
        if (follower.getPose().getX() < 124 && !outtakeInProgress && (pathState == 5 || pathState == 8 || pathState == 11)){
            if (order != null) spindexer.setShootIndex(order[currentOrderIndex]);
        }

        autonomousUpdate();
        PoseStorage.currentPose = follower.getPose();

        panelsTelemetry.debug("State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Outtake", outtakeInProgress);
        panelsTelemetry.debug("Balls", spindexer.getBalls());
        panelsTelemetry.debug("Scanned Tag ID", scannedTagId);

        if(currentBarIntakeState.equals("in")){
            barIntake.spinIntake();
        }else if(currentBarIntakeState.equals("out")){
            barIntake.spinIntake();
        }else{
            barIntake.stop();
        }

        panelsTelemetry.update(telemetry);
    }


    private void autonomousUpdate() {
        if (outtakeInProgress) {
            handleOuttakeRoutine();
            return;
        }

        if (scannedTagId == 0) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                scannedTagId = result.getFiducialResults().get(0).getFiducialId();
                order = getMotifForTag(scannedTagId);
            }
        }

        if (order != null && currentOrderIndex < order.length && !outtakeInProgress) {
            targetAngle = SHOOT_DEG;
        }

        switch (pathState) {
            // 0) Follow PresetShoot
            case 0:
                follower.followPath(paths.PresetShoot);
                setState(1);
                break;

            // 1) End of PresetShoot -> Shoot
            case 1:
                if (!follower.isBusy()) { // Wait for path end
                     if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(2);
                        currentOrderIndex = 1;
                    }
                }
                break;

            // 2) After Shoot -> Pickup1
            case 2:
                if (!outtakeInProgress) {
                    follower.followPath(paths.Pickup1);
                    setState(3);
                }
                break;

            // 3) After Pickup1 -> ReleaseGate
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ReleaseGate);
                    setState(4);
                }
                break;

            // 4) After ReleaseGate -> Shoot1
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot1);
                    setState(5);
                }
                break;

            // 5) End of Shoot1 -> Shoot
            case 5:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(6);
                        currentOrderIndex = 2; // Next motif index
                    }
                }
                break;

            // 6) After Outtake -> GateIntake
            case 6:
                if (!outtakeInProgress) {
                    follower.followPath(paths.GateIntake);
                    setState(7);
                }
                break;

            // 7) After GateIntake -> Wait
            case 7:
                if (!follower.isBusy()) {
                    setState(8);
                }
                break;

            case 8:
                 // Wait 3 seconds at Gate Intake before shooting
                 if (stateTimer.milliseconds() > 3000) {
                     follower.followPath(paths.ShootGateIntake);
                     setState(9);
                 }
                 break;

            // 9) End of ShootGateIntake -> Shoot
            case 9:
                 if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(10);
                        currentOrderIndex = 3;
                    }
                }
                break;

            // 10) After Outtake -> Pickup2
            case 10:
                if (!outtakeInProgress) {
                    follower.followPath(paths.Pickup2);
                    setState(11);
                }
                break;

            // 11) After Pickup2 -> Shoot2
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot2);
                    setState(12);
                }
                break;

            // 12) End of Shoot2 -> Shoot
            case 12:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(13);
                    }
                }
                break;

            // 13) After Outtake -> Park
            case 13:
                 if (!outtakeInProgress) {
                    follower.followPath(paths.Park, PARK_SPEED, false);
                    setState(14);
                }
                break;

            case 14:
                // Done
                break;
        }
    }

    private void startOuttakeRoutine() {
        outtakeInProgress = true;
        outtakeAdvanceCount = 0;
        outtakeTimer.reset();
        lastAdvanceTime = 0;

        turret.transferOn();
        currentBarIntakeState = "stop";

        kickerServo.kick();
        lastAdvanceTime = outtakeTimer.milliseconds();
    }

    private void handleOuttakeRoutine() {
        double currentTime = outtakeTimer.milliseconds();

        if (outtakeAdvanceCount < 2) {
            if (currentTime - lastAdvanceTime >= OUTTAKE_DELAY_MS) {
                spindexer.advanceShoot();
                outtakeAdvanceCount++;
                lastAdvanceTime = currentTime;
            }
        } else {
            if (currentTime - lastAdvanceTime >= OUTTAKE_DELAY_MS) {
                kickerServo.normal();
                spindexer.clearTracking();
                currentBarIntakeState = "in";
                spindexer.setIntakeIndex(0);
                spinInterval = 0;
                outtakeInProgress = false;
            }
        }
    }


    public static class Paths {
        public PathChain PresetShoot;
        public PathChain Pickup1;
        public PathChain ReleaseGate;
        public PathChain Shoot1;
        public PathChain GateIntake;
        public PathChain ShootGateIntake;
        public PathChain Pickup2;
        public PathChain Shoot2;
        public PathChain Park;

        public Paths(Follower follower) {
              PresetShoot = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(121.396, 120.422),
                    new Pose(110.000, 110.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Pickup1 = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(110.000, 110.000),
                    new Pose(88.149, 77.811),
                    new Pose(76.078, 54.808),
                    new Pose(135.307, 59.578)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            ReleaseGate = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(135.307, 59.578),
                    new Pose(117.743, 59.268),
                    new Pose(128.422, 64.146)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Shoot1 = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(128.422, 64.146),
                    new Pose(91.583, 66.836),
                    new Pose(104.346, 103.912)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            GateIntake = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(104.346, 103.912),
                    new Pose(86.727, 69.932),
                    new Pose(132.591, 58.597)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(33)).build();

            ShootGateIntake = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(132.591, 58.597),
                    new Pose(86.727, 69.932),
                    new Pose(104.346, 103.912)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(33), Math.toRadians(0)).build();

            Pickup2 = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(104.346, 103.912),
                    new Pose(95.893, 79.580),
                    new Pose(129.386, 84.373)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Shoot2 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(129.386, 84.373),
                    new Pose(104.346, 103.912)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0)).build();

            Park = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(104.346, 103.912),
                    new Pose(118.871, 84.315)
                )
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();
        }
    }
}

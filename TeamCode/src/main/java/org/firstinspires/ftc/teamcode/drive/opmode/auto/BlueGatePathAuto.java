package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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


@Autonomous(name = "Blue Gate Path Auto", group = "Autonomous")
@Configurable
public class BlueGatePathAuto extends OpMode {

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
    private String currentBarIntakeState = "stop";

    // -------------------- Config (tune in Panels) --------------------
    public static double SCAN_TURRET_DEG = 110;
    public static double SHOOT_DEG = 43.5;
    public static double SHOOT_RPM = 2125;
    public static double PARK_SPEED = 1.0;

    // Outtake cadence
    public static double OUTTAKE_DELAY_MS = 400;
    private double targetAngle = SCAN_TURRET_DEG;

    // -------------------- State machine --------------------
    private int pathState = 0;
    private int lastState = -1;
    private final ElapsedTime stateTimer = new ElapsedTime();

    private final ElapsedTime settleTimer = new ElapsedTime();
    private boolean isSettling = false;
    private static final long SETTLE_DELAY_MS = 250;
    private static final long GATE_WAIT_MS = 4500;

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

    // -----------------------------------------------------------------------------------------
    // init / start / loop
    // -----------------------------------------------------------------------------------------

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
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
        spindexer.filled = new char[]{'X', 'X', 'X'};

        // Paths
        paths = new Paths(follower);

        // Startup config
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
        currentBarIntakeState = "in";
    }

    @Override
    public void loop() {
        // 1) Always update follower first
        follower.update();

        // 2) Always keep shooter ready
        turret.on();
        turret.setShooterRPM(SHOOT_RPM);
        turret.goToPosition(targetAngle);

        // 3) Update spindexer
        spindexer.update();
        if (spindexer.isFull() && !outtakeInProgress && follower.getPose().getX() < 132) {
            spinInterval++;
            if (spinInterval > 30 && spinInterval < 40) {
                currentBarIntakeState = "out";
            } else {
                currentBarIntakeState = "stop";
            }
        }

        if (spindexer.isFull() && !outtakeInProgress) {
            spindexer.setShootIndex(1);
        }

        // 4) Run state machine
        autonomousUpdate();
        PoseStorage.currentPose = follower.getPose();

        // 5) Telemetry
        panelsTelemetry.debug("State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Outtake", outtakeInProgress);
        panelsTelemetry.debug("Balls", spindexer.getBalls());
        //panelsTelemetry.debug("Scanned Tag ID", scannedTagId);
        //
        if (currentBarIntakeState.equals("in")) {
            barIntake.spinIntake();
        } else if (currentBarIntakeState.equals("out")) {
            barIntake.spinOuttake();
        } else {
            barIntake.stop();
        }

        panelsTelemetry.update(telemetry);
    }

    // -----------------------------------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------------------------------

    private void autonomousUpdate() {
        // Outtake blocks transitions
        if (outtakeInProgress) {
            handleOuttakeRoutine();
            return;
        }

        targetAngle = SHOOT_DEG;

        switch (pathState) {
            case 0:
                follower.followPath(paths.PresetShoot);
                setState(1);
                break;

            case 1:
                if (!follower.isBusy() && stateTimer.milliseconds() > 2000) {
                    startOuttakeRoutine();
                    setState(2);
                }
                break;

            case 2:
                follower.followPath(paths.Pickup1);
                setState(3);
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.ReleaseGate);
                    setState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot1);
                    setState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(6);
                    }
                }
                break;

            case 6:
                follower.followPath(paths.GateIntake);
                setState(7);
                break;

            case 7:
                if (!follower.isBusy()) {
                    if (stateTimer.milliseconds() < GATE_WAIT_MS) {
                        return;
                    }
                    follower.followPath(paths.ShootGateIntake);
                    setState(8);
                }
                break;

            case 8:
                if (!follower.isBusy() && !Objects.equals(barIntake.getStatus(), "out")) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(9);
                    }
                }
                break;

            case 9:
                follower.followPath(paths.GateIntake);
                setState(10);
                break;

            case 10:
                if (!follower.isBusy()) {
                    if (stateTimer.milliseconds() < GATE_WAIT_MS) {
                        return;
                    }
                    follower.followPath(paths.ShootGateIntake);
                    setState(11);
                }
                break;

            case 11:
                if (!follower.isBusy() && !Objects.equals(barIntake.getStatus(), "out")) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(12);
                    }
                }
                break;

            case 12:
                follower.followPath(paths.Pickup2);
                setState(13);
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Shoot2);
                    setState(14);
                }
                break;

            case 14:
                if (!follower.isBusy() && !Objects.equals(barIntake.getStatus(), "out")) {
                    if (!isSettling) {
                        isSettling = true;
                        settleTimer.reset();
                    } else if (settleTimer.milliseconds() > SETTLE_DELAY_MS) {
                        startOuttakeRoutine();
                        setState(15);
                    }
                }
                break;

            case 15:
                follower.followPath(paths.Park, PARK_SPEED, false);
                setState(16);
                break;

            case 16:
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
            if (currentTime - lastAdvanceTime >= (outtakeAdvanceCount == 0 ? OUTTAKE_DELAY_MS / 2 : OUTTAKE_DELAY_MS)) {
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

    // -----------------------------------------------------------------------------------------
    // Paths (new geometry)
    // -----------------------------------------------------------------------------------------

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
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Pickup1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(110.000, 110.000),
                                    new Pose(88.149, 77.811),
                                    new Pose(76.078, 54.808),
                                    new Pose(135.307, 59.578)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            ReleaseGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(135.307, 59.578),
                                    new Pose(117.743, 59.268),
                                    new Pose(128.422, 64.146)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Shoot1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(128.422, 64.146),
                                    new Pose(91.583, 66.836),
                                    new Pose(104.346, 103.912)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            GateIntake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(104.346, 103.912),
                                    new Pose(86.727, 69.932),
                                    new Pose(132.591, 58.597)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(33))
                    .build();

            ShootGateIntake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(132.591, 58.597),
                                    new Pose(86.727, 69.932),
                                    new Pose(104.346, 103.912)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(33), Math.toRadians(0))
                    .build();

            Pickup2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(104.346, 103.912),
                                    new Pose(95.893, 79.580),
                                    new Pose(129.386, 84.373)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.386, 84.373),
                                    new Pose(104.346, 103.912)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(104.346, 103.912),
                                    new Pose(118.871, 84.315)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();
        }
    }
}

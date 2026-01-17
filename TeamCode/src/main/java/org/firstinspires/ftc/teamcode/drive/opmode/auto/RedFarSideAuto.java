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
import org.firstinspires.ftc.teamcode.sorting.ColorSensor;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;
import org.firstinspires.ftc.teamcode.shooting.Turret;

@Autonomous(name = "Red Far Side Auto (OpMode)", group = "Autonomous")
@Configurable
public class RedFarSideAuto extends OpMode {

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

    // -------------------- Timers --------------------
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime waitTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();

    // -------------------- State machine --------------------
    private int pathState = 0;
    private int lastState = -1;

    private void setState(int s) {
        if (s != lastState) {
            lastState = s;
            stateTimer.reset();
        }
        pathState = s;
    }

    // -------------------- Outtake routine --------------------
    private final ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean outtakeInProgress = false;
    private int outtakeAdvanceCount = 0;
    private double lastAdvanceTimeMs = 0.0;

    public static double OUTTAKE_DELAY_MS = 700;

    // -------------------- Loop timing --------------------
    public static int LOOP_WAIT_MS = 50;

    // -------------------- Dynamic destinations (MIRRORED) --------------------
    // Blue PARK_POSE: (48.400, 32.886, 180deg) -> Red: (95.600, 32.886, 0deg)
    public static final Pose PARK_POSE = new Pose(
            95.600,
            32.886,
            Math.toRadians(0)
    );

    // Blue SHOOT_POSE: (61.000, 14.000, 180deg) -> Red: (83.000, 14.000, 0deg)
    public static final Pose SHOOT_POSE = new Pose(
            83.000, 14.000, Math.toRadians(0)
    );

    // -------------------- Shooter settings --------------------
    public static double currentRPM = 3250;
    public static double targetAngleDeg = 287; // may need mirroring depending on your turret convention

    // Optional: mirrored fixed target pose (Blue target (0,144) -> Red target (144,144))
    private final Pose targetPose = new Pose(144, 144, 0);

    // -------------------- Volley / shot routine --------------------
    private int currentVolley = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // Blue startPose: (64, 8, 180deg) -> Red: (80, 8, 0deg)
        Pose startPose = new Pose(80.000, 8.000, Math.toRadians(0));
        follower.setStartingPose(startPose);

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
                true,
                true,
                52 // may need mirroring depending on your turret convention
        );

        // Build paths
        paths = new Paths(follower);

        // Startup config
        kickerServo.normal();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        matchTimer.reset();
        waitTimer.reset();
        stateTimer.reset();

        outtakeInProgress = false;
        currentVolley = 0;

        turret.on();
        barIntake.spinIntake();

        setState(0);
    }

    @Override
    public void loop() {
        follower.update();

        turret.setShooterRPM(currentRPM);
        turret.goToPosition(targetAngleDeg);
        turret.on();

        // Optional turret tracking
        // turret.trackTarget(follower.getPose(), targetPose);

        spindexer.update();

        if (spindexer.isFull()) {
            barIntake.stop();
        }

        autonomousUpdate();

        panelsTelemetry.debug("Match Time (s)", matchTimer.seconds());
        panelsTelemetry.debug("State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Target RPM", currentRPM);
        panelsTelemetry.debug("Turret Angle (deg)", targetAngleDeg);
        panelsTelemetry.debug("Outtake", outtakeInProgress);
        panelsTelemetry.debug("Full", spindexer.isFull());
        panelsTelemetry.update(telemetry);
    }

    // ------------------------------------------------------------------------
    // Outtake routine (kick + advance x3 on delay)
    // ------------------------------------------------------------------------

    private void startOuttakeRoutine() {
        outtakeInProgress = true;
        outtakeAdvanceCount = 0;
        outtakeTimer.reset();
        lastAdvanceTimeMs = 0.0;

        turret.on();
        turret.transferOn();

        kickerServo.kick();

        spindexer.advanceIntake();
        outtakeAdvanceCount++;
        lastAdvanceTimeMs = outtakeTimer.milliseconds();
    }

    private void handleOuttakeRoutine() {
        double now = outtakeTimer.milliseconds();

        if (outtakeAdvanceCount < 3) {
            if (now - lastAdvanceTimeMs >= OUTTAKE_DELAY_MS) {
                spindexer.advanceIntake();
                outtakeAdvanceCount++;
                lastAdvanceTimeMs = now;
            }
            return;
        }

        if (now - lastAdvanceTimeMs >= OUTTAKE_DELAY_MS) {
            kickerServo.normal();
            spindexer.clearTracking();
            turret.transferOff();
            barIntake.spinIntake();
            outtakeInProgress = false;
        }
    }

    // ------------------------------------------------------------------------
    // State machine (same logic)
    // ------------------------------------------------------------------------

    private void autonomousUpdate() {
        if (outtakeInProgress) {
            handleOuttakeRoutine();
            return;
        }

        switch (pathState) {

            case 0:
                double curr = turret.getTrackedTurretAngle();
                double err = Math.abs(Math.IEEEremainder(targetAngleDeg - curr, 360.0));

                if (err < 3.0 && turret.getShooterRPM() > 3300) {
                    startOuttakeRoutine();
                    setState(1);
                }
                break;

            case 1:
                if (!outtakeInProgress) {
                    follower.followPath(paths.pickup1);
                    waitTimer.reset();
                    setState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot1);
                    setState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    startOuttakeRoutine();
                    setState(4);
                }
                break;

            case 4:
                if (!outtakeInProgress) {
                    follower.followPath(paths.pickup2);
                    waitTimer.reset();
                    setState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot2);
                    setState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    startOuttakeRoutine();
                    setState(7);
                }
                break;

            case 7:
                if (!outtakeInProgress) {
                    follower.followPath(paths.path5);
                    waitTimer.reset();
                    setState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    if (matchTimer.seconds() > 25.0) {
                        paths.buildToPark(follower, follower.getPose());
                        follower.followPath(paths.toPark);
                        setState(98);
                        break;
                    }

                    if (waitTimer.milliseconds() < LOOP_WAIT_MS) return;

                    if (spindexer.getBalls() > 0) {
                        paths.buildToShoot(follower, follower.getPose());
                        follower.followPath(paths.toShoot);
                        setState(90);
                        break;
                    }

                    follower.followPath(paths.path6);
                    waitTimer.reset();
                    setState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    if (matchTimer.seconds() > 25.0) {
                        paths.buildToPark(follower, follower.getPose());
                        follower.followPath(paths.toPark);
                        setState(98);
                        break;
                    }

                    if (waitTimer.milliseconds() < LOOP_WAIT_MS) return;

                    if (spindexer.getBalls() > 0) {
                        paths.buildToShoot(follower, follower.getPose());
                        follower.followPath(paths.toShoot);
                        setState(90);
                        break;
                    }

                    follower.followPath(paths.path55);
                    waitTimer.reset();
                    setState(8);
                }
                break;

            case 90:
                if (!follower.isBusy()) {
                    startOuttakeRoutine();
                    setState(91);
                }
                break;

            case 91:
                if (!outtakeInProgress) {
                    follower.followPath(paths.path5);
                    waitTimer.reset();
                    setState(8);
                }
                break;

            case 98:
                if (!follower.isBusy()) {
                    setState(99);
                }
                break;

            case 99:
                break;
        }
    }

    // ------------------------------------------------------------------------
    // Paths class (MIRRORED GEOMETRY)
    // ------------------------------------------------------------------------

    public static class Paths {
        public PathChain pickup1, shoot1, pickup2, shoot2, path5, path55, path6;

        public PathChain toShoot;
        public PathChain toPark;

        public Paths(Follower follower) {

            // pickup1:
            // Blue: (64,8)->(63,41)->(10,35) with 180->180
            // Red : (80,8)->(81,41)->(134,35) with 0->0
            pickup1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(80.000, 8.000),
                            new Pose(81.000, 41.000),
                            new Pose(134.000, 35.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // shoot1:
            // Blue: (10,35)->(61,14)
            // Red : (134,35)->(83,14)
            shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(134.000, 35.000),
                            new Pose(83.000, 14.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // pickup2:
            // Blue: (61,14)->(13.555,4.401)->(23.76,19.89)->(10,7.746)
            // Red : (83,14)->(130.445,4.401)->(120.240,19.89)->(134,7.746)
            pickup2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(83.000, 14.000),
                            new Pose(130.445, 4.401),
                            new Pose(120.240, 19.890),
                            new Pose(134.000, 7.746)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // shoot2:
            // Blue: (10,7.746)->(61,14)
            // Red : (134,7.746)->(83,14)
            shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(134.000, 7.746),
                            new Pose(83.000, 14.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // path5:
            // Blue: (61,14)->(10,12.851)
            // Red : (83,14)->(134,12.851)
            path5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(83.000, 14.000),
                            new Pose(134.000, 12.851)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // path55:
            // Blue: (9+16,12.851)->(10,12.851) i.e. (25,12.851)->(10,12.851)
            // Red : (119,12.851)->(134,12.851)
            path55 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(119.000, 12.851),
                            new Pose(134.000, 12.851)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // path6:
            // Blue: (10,12.851)->(60.733,13.731)
            // Red : (134,12.851)->(83.267,13.731)
            path6 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(134.000, 12.851),
                            new Pose(83.267, 13.731)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }

        public void buildToShoot(Follower follower, Pose currentPose) {
            toShoot = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, SHOOT_POSE))
                    .setLinearHeadingInterpolation(currentPose.getHeading(), SHOOT_POSE.getHeading())
                    .build();
        }

        public void buildToPark(Follower follower, Pose currentPose) {
            toPark = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, PARK_POSE))
                    .addPath(new BezierLine(currentPose, PARK_POSE)) // kept as-is (you had it twice)
                    .setLinearHeadingInterpolation(currentPose.getHeading(), PARK_POSE.getHeading())
                    .build();
        }
    }
}

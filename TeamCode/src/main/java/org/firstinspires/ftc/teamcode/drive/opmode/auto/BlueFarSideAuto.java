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

@Autonomous(name = "Blue Far Side Auto (OpMode)", group = "Autonomous")
@Configurable
public class BlueFarSideAuto extends OpMode {

    // -------------------- Panels + Pedro --------------------
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    // -------------------- Subsystems (match your OpMode-based BlueAuto) --------------------
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

    // -------------------- Outtake routine (ported pattern from your OpMode BlueAuto) --------------------
    private final ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean outtakeInProgress = false;
    private int outtakeAdvanceCount = 0;
    private double lastAdvanceTimeMs = 0.0;

    // You can tune this; in your excerpt you used ~650-700ms depending on shot
    public static double OUTTAKE_DELAY_MS = 700;

    // -------------------- Loop timing --------------------
    public static int LOOP_WAIT_MS = 100;

    // -------------------- Dynamic destinations --------------------
    // Park pose (same as your original far-side code)
    public static final Pose PARK_POSE = new Pose(
            38.552567237163814,
            33.623471882640594,
            Math.toRadians(90)
    );

    // Shoot pose during loop (same as original far-side code)
    public static final Pose SHOOT_POSE = new Pose(
            61.000, 14.000, Math.toRadians(180)
    );

    // -------------------- “Auto shoot power” equivalents in this framework --------------------
    // In your OpMode auto, shooter power is “RPM” + turret angle
    public static double currentRPM = 3250;
    public static double targetAngleDeg = 286;

    // Optional: a target pose if you use turret tracking (left here for parity)
    private final Pose targetPose = new Pose(0, 144, 0);

    // -------------------- Volley / shot routine --------------------
    // In this framework, “volley” is your outtake routine (kick + advance x3 with delays).
    // If you need different orders, you’d implement spindexer indexing here.
    private int currentVolley = 0;

    // ------------------------------------------------------------------------
    // init / start / loop (OpMode style)
    // ------------------------------------------------------------------------

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // IMPORTANT: starting pose should match the first path start (pickup1 starts at 64,8)
        Pose startPose = new Pose(64.000, 8.000, Math.toRadians(180));
        follower.setStartingPose(startPose);

        // Subsystems (match your OpMode BlueAuto names/signatures)
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
                true
        );

        // Build paths
        paths = new Paths(follower);

        // Startup config
        kickerServo.normal();
        barIntake.spinIntake();

        // Spin shooter / set RPM & turret angle early
        turret.on();
        turret.transferOn();
        currentRPM = 3260;
        targetAngleDeg = 286;

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
        turret.transferOn();
        barIntake.spinIntake();

        setState(0);
    }

    @Override
    public void loop() {
        // Update follower
        follower.update();

        // Keep shooter ready
        turret.setShooterRPM(currentRPM);
        turret.goToPosition(targetAngleDeg);
        turret.on();
        turret.transferOn();

        // Optional turret tracking (commented like your excerpt)
        // turret.trackTarget(follower.getPose(), targetPose);

        // Spindexer update (TeleOp-like)
        spindexer.update();

        // If full, stop intake (matches your excerpt)
        if (spindexer.isFull()) {
            barIntake.stop();
        }

        // Run auto state machine
        autonomousUpdate();

        // Telemetry
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

        // First advance immediately
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

        // After 3 advances, wait one more delay then finish
        if (now - lastAdvanceTimeMs >= OUTTAKE_DELAY_MS) {
            kickerServo.normal();
            spindexer.clearTracking();
            barIntake.spinIntake();
            outtakeInProgress = false;
        }
    }

    // ------------------------------------------------------------------------
    // State machine (far-side strategy)
    // ------------------------------------------------------------------------

    private void autonomousUpdate() {
        // If outtaking, block path transitions
        if (outtakeInProgress) {
            handleOuttakeRoutine();
            return;
        }

        switch (pathState) {

            // ------------------------------------------------------------
            // 0: Preset Shot (shoot in place)
            // ------------------------------------------------------------
            case 0:
                double curr = turret.getTrackedTurretAngle();
                double err = Math.abs(Math.IEEEremainder(targetAngleDeg - curr, 360.0));

                if (err < 3.0) { // make sure angle is right
                    startOuttakeRoutine();
                    setState(1);
                }
                break;
            case 1:
                // After preset volley completes, go pickup1
                if (!outtakeInProgress) {
                    follower.followPath(paths.pickup1);
                    waitTimer.reset();
                    setState(2);
                }
                break;

            // ------------------------------------------------------------
            // Cycle 1: pickup1 -> short wait -> shoot1 -> volley -> pickup2
            // ------------------------------------------------------------
            case 2:
                if (!follower.isBusy()) {
                    // Small settle delay
                    if (waitTimer.milliseconds() < 200) return;

                    // Go to shoot1
                    follower.followPath(paths.shoot1);
                    setState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    // Tune for shoot1 position

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

            // ------------------------------------------------------------
            // Cycle 2: pickup2 -> wait -> shoot2 -> volley -> path5
            // ------------------------------------------------------------
            case 5:
                if (!follower.isBusy()) {
                    if (waitTimer.milliseconds() < 400) return;

                    follower.followPath(paths.shoot2);
                    setState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    // Tune for shoot2 position

                    startOuttakeRoutine();
                    setState(7);
                }
                break;

            case 7:
                if (!outtakeInProgress) {
                    // Begin loop: go to path5 first
                    follower.followPath(paths.path5);
                    waitTimer.reset();
                    setState(8);
                }
                break;

            // ------------------------------------------------------------
            // Loop: path5 <-> path6 with priority checks
            // ------------------------------------------------------------
            case 8:
                if (!follower.isBusy()) {
                    if (waitTimer.milliseconds() < LOOP_WAIT_MS) return;

                    // Priority 1: time-based exit
                    if (matchTimer.seconds() > 25.0) {
                        paths.buildToPark(follower, follower.getPose());
                        follower.followPath(paths.toPark);
                        setState(98);
                        break;
                    }

                    // Priority 2: if full, go shoot, then return to loop
                    if (spindexer.getBalls() > 0) {
                        paths.buildToShoot(follower, follower.getPose());
                        follower.followPath(paths.toShoot);
                        setState(90);
                        break;
                    }

                    // Otherwise continue loop to path6
                    follower.followPath(paths.path6);
                    waitTimer.reset();
                    setState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    if (waitTimer.milliseconds() < LOOP_WAIT_MS) return;

                    // Priority 1: time-based exit
                    if (matchTimer.seconds() > 25.0) {
                        paths.buildToPark(follower, follower.getPose());
                        follower.followPath(paths.toPark);
                        setState(98);
                        break;
                    }

                    // Priority 2: if full, go shoot, then return to loop
                    if (spindexer.getBalls() > 0) {
                        paths.buildToShoot(follower, follower.getPose());
                        follower.followPath(paths.toShoot);
                        setState(90);
                        break;
                    }

                    // Otherwise loop back to path5
                    follower.followPath(paths.path55);
                    waitTimer.reset();
                    setState(8);
                }
                break;

            // ------------------------------------------------------------
            // Go to shoot because full
            // ------------------------------------------------------------
            case 90:
                if (!follower.isBusy()) {
                    // Tune for SHOOT_POSE
                    startOuttakeRoutine();
                    setState(91);
                }
                break;

            case 91:
                if (!outtakeInProgress) {
                    // Return to loop
                    follower.followPath(paths.path5);
                    waitTimer.reset();
                    setState(8);
                }
                break;

            // ------------------------------------------------------------
            // Park because time > 25
            // ------------------------------------------------------------
            case 98:
                if (!follower.isBusy()) {
                    setState(99);
                }
                break;

            case 99:
                // End
                break;
        }
    }

    // ------------------------------------------------------------------------
    // Paths class (matches your original far-side geometry + dynamic build)
    // ------------------------------------------------------------------------

    public static class Paths {
        public PathChain pickup1, shoot1, pickup2, shoot2, path5, path55, path6;

        // Dynamic paths (built at runtime)
        public PathChain toShoot;
        public PathChain toPark;

        public Paths(Follower follower) {
            pickup1 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(64.000, 8.000),
                            new Pose(63.000, 41.000),
                            new Pose(10.000, 35.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(10.000, 35.000),
                            new Pose(61.000, 14.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            pickup2 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(61.000, 14.000),
                            new Pose(13.555, 4.401),
                            new Pose(23.76, 19.89),
                            new Pose(10.000, 7.746)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(10.000, 7.746),
                            new Pose(61.000, 14.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            path5 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(61.000, 14.000),
                            new Pose(10.000, 12.851)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            path55 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(9.000+16, 12.851),
                            new Pose(10.000, 12.851)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();



            path6 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(10.000, 12.851),
                            new Pose(60.733, 13.731)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
                    .setLinearHeadingInterpolation(currentPose.getHeading(), PARK_POSE.getHeading())
                    .build();
        }
    }
}
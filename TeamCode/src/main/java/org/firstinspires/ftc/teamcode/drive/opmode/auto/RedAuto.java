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

@Autonomous(name = "Red Auto", group = "Autonomous")
@Configurable
public class RedAuto extends OpMode {

    // Panels + Pedro
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;

    // Subsystems (match TeleOp logic)
    private BarIntake barIntake;
    private ColorSensor colorSensor;
    private Spindexer spindexer;
    private KickerServo kickerServo;
    double targetAngle = 52;
    private Turret turret;

    // State machine
    private int pathState = 0;

    // Outtake routine (ported from TeleOp)
    private final ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean outtakeInProgress = false;
    private int outtakeAdvanceCount = 0;
    private double lastAdvanceTimeMs = 0.0;
    private double OUTTAKE_DELAY_MS = 650.0;

    // RPM (ported from TeleOp distance->rpm)
    private double currentRPM = 2210;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private int lastState = -1;

    private void setState(int s) {
        if (s != lastState) {
            lastState = s;
            stateTimer.reset();
        }
        pathState = s;
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        // Use the FIRST pose of shoot1 as the starting pose (most important)
        // was: (22.55, 123.14, 180deg)
        Pose startPose = new Pose(121.45, 123.14, Math.toRadians(0));
        follower.setStartingPose(startPose);


        // Subsystems (use the same device names as TeleOp)
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
                52
        );

        // Paths
        paths = new Paths(follower);

        // Startup config similar to TeleOp behavior
        kickerServo.normal();
        //turret.on();
        //barIntake.spinIntake();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Ensure shooter/intake on at start
        turret.on();
        barIntake.spinIntake();
        turret.transferOn();
        // In start(), replace: pathState = 0;
        setState(0);
        outtakeInProgress = false;
    }

    @Override
    public void loop() {
        // Update follower first
        follower.update();

        // Keep turret aimed + RPM updated (so by the time you arrive, you're close)
        //updateShooterRPMFromDistance();
        turret.setShooterRPM(currentRPM);


        // While in a "shoot" stage, keep tracking target (safe to call always)
        //turret.trackTarget(follower.getPose(), targetPose);
        turret.goToPosition(targetAngle);
        turret.on();
        // Spindexer continuous update
        spindexer.update();
        if (!outtakeInProgress && spindexer.isFull()) {
            barIntake.stop();
        }


        // Run the autonomous state machine
        autonomousPathUpdate();

        // Telemetry
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Target RPM", currentRPM);
        panelsTelemetry.debug("Outtake", outtakeInProgress);
        panelsTelemetry.update(telemetry);
    }



    // -------------------- Outtake routine (TeleOp -> Auto) --------------------

    private void startOuttakeRoutine() {
        outtakeInProgress = true;
        outtakeAdvanceCount = 0;
        outtakeTimer.reset();
        lastAdvanceTimeMs = 0.0;

        // Kick
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

        // After the 3 advances, wait one more delay then finish
        if (now - lastAdvanceTimeMs >= OUTTAKE_DELAY_MS) {
            kickerServo.normal();
            spindexer.clearTracking();
            barIntake.spinIntake();
            outtakeInProgress = false;
        }
    }

    // -------------------- State machine --------------------

    public void autonomousPathUpdate() {
        // If we are currently outtaking, that blocks path transitions.
        if (outtakeInProgress) {
            handleOuttakeRoutine();
            return;
        }

        switch (pathState) {

            // -------- shoot1 -> outtake -> pickupPreset1 --------
            case 0:
                follower.followPath(paths.shoot1);
                setState(1);
                break;

            case 1: // end of shoot1: do outtake, then go next
                if (!follower.isBusy()) {
                    startOuttakeRoutine();
                    setState(2);
                }
                break;

            case 2: // after outtake completes, start pickupPreset1
                if (!outtakeInProgress) {
                    follower.followPath(paths.pickupPreset1);
                    currentRPM = 2580;
                    targetAngle = 52;
                    setState(3);
                }
                break;

            // -------- shoot2 -> outtake -> gateIntake1 --------
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot2);
                    setState(4);
                }
                break;

            case 4: // end of shoot2
                if (!follower.isBusy()) {
                    startOuttakeRoutine();
                    setState(5);
                }
                break;

            case 5:
                if (!outtakeInProgress) {
                    follower.followPath(paths.gateIntake1);
                    setState(6);
                }
                break;

            // -------- shoot3 -> outtake -> gateIntake2 --------
            case 6:
                if (!follower.isBusy()) {
                    if (stateTimer.milliseconds() == 0) stateTimer.reset(); // (better: use a dedicated timer)
                    if (stateTimer.milliseconds() < 3000) {
                        // wait 3 seconds
                        return;
                    }
                    follower.followPath(paths.shoot3);
                    setState(7);
                }
                break;

            case 7: // end of shoot3
                if (!follower.isBusy()) {
                    startOuttakeRoutine();
                    setState(11);
                }
                break;

//            case 8:
//                if (!outtakeInProgress) {
//                    follower.followPath(paths.gateIntake2);
//                    setState(9);
//                }
//                break;
//
//            // -------- shoot4 -> outtake -> pickupPreset2 --------
//            case 9:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.shoot4);
//                    setState(10);
//                }
//                break;
//
//            case 10: // end of shoot4
//                if (!follower.isBusy()) {
//                    startOuttakeRoutine();
//                    setState(11);
//                }
//                break;

            case 11:
                if (!outtakeInProgress) {
                    currentRPM = 2010.7;
                    targetAngle = 64;
                    follower.followPath(paths.pickupPreset2, 0.7, false);
                    setState(12);
                }
                break;

            // -------- shoot5 -> outtake -> pickupPreset3 --------
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot5);
                    setState(13);
                }
                break;

            case 13: // end of shoot5
                if (!follower.isBusy()) {
                    startOuttakeRoutine();
                    setState(14);
                }
                break;

            case 14:
                if (!outtakeInProgress) {
                    follower.followPath(paths.pickupPreset3, 0.9, false);
                    currentRPM = 3250;
                    targetAngle = 73;
                    OUTTAKE_DELAY_MS = 700;

                    setState(15);
                }
                break;

            // -------- shoot6 -> outtake -> done --------
            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(paths.shoot6);
                    setState(16);
                }
                break;

            case 16: // end of shoot6
                if (!follower.isBusy()) {
                    startOuttakeRoutine();
                    setState(17);
                }
                break;

            case 17:
                if (!outtakeInProgress) {
                    // Done. Stay here.
                    setState(18);
                }
                break;

            case 18:
                if (!follower.isBusy()) {
                    follower.followPath(paths.leave, 0.6, true);
                    setState(18);
                }
                break;

        }
    }

    // -------------------- Paths (your new pathing) --------------------

    public static class Paths {
        public PathChain shoot1;
        public PathChain pickupPreset1;
        public PathChain shoot2;
        public PathChain gateIntake1;
        public PathChain shoot3;
        public PathChain gateIntake2;
        public PathChain shoot4;
        public PathChain pickupPreset2;
        public PathChain shoot5;
        public PathChain pickupPreset3;
        public PathChain shoot6;
        public PathChain leave;

        public Paths(Follower follower) {

            // shoot1: (22.55,123.14)->(46.571,96.857)
            shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(121.45, 123.14),
                            new Pose(97.429, 96.857)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // pickupPreset1 curve:
            // (46.571,96.857)->(57.529,39.6)->(45.343,66.543)->(17,61.2)
            pickupPreset1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(97.429, 96.857),
                                    new Pose(86.471, 39.600),
                                    new Pose(98.657, 66.543),
                                    new Pose(127.000, 61.200)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // shoot2: (17,61.2)->(62.129,75.457)
            shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(127.000, 61.200),
                                    new Pose(81.871, 75.457)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // gateIntake1: (62.129,75.457)->(11.553,62.059) heading 180->147.6
            gateIntake1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(81.871, 75.457),
                            new Pose(132.447, 62.059)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(32.4)) // 180-147.6=32.4
                    .build();

            // shoot3: (11.553,62.059)->(62.129,75.457) heading 147.6->180
            shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(132.447, 62.059),
                            new Pose(81.871, 75.457)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(32.4), Math.toRadians(0))
                    .build();

            // gateIntake2: (62.129,75.457)->(12.629,61.029) heading 180->152
            gateIntake2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(81.871, 75.457),
                            new Pose(131.371, 61.029)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(28.0)) // 180-152=28
                    .build();

            // shoot4: (12.629,61.029)->(60.971,76) heading 152->180
            shoot4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(131.371, 61.029),
                            new Pose(83.029, 76.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(28.0), Math.toRadians(0))
                    .build();

            // pickupPreset2 curve: (60.971,76)->(55.47,88.7)->(13.571,86.657)
            pickupPreset2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(83.029, 76.000),
                            new Pose(88.530, 88.700),
                            new Pose(130.429, 86.657)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // shoot5: (13.571,86.657)->(31.314,101.686)
            shoot5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(130.429, 86.657),
                            new Pose(112.686, 101.686)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // pickupPreset3 curve: (31.314,101.686)->(73.057,31.586)->(10,35.714)
            pickupPreset3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(112.686, 101.686),
                            new Pose(70.943, 31.586),
                            new Pose(134.000, 35.714)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // shoot6 curve: (10,35.714)->(59.457,33.029)->(61.6,14.69)
            shoot6 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(134.000, 35.714),
                            new Pose(84.543, 33.029),
                            new Pose(82.400, 14.690)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // leave: (61.6,14.69)->(48.4,32.886)
            leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(82.400, 14.690),
                                    new Pose(95.600, 32.886)
                            )
                    ).setTangentHeadingInterpolation()
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }

}

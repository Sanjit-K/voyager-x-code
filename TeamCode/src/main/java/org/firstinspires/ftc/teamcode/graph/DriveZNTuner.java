package org.firstinspires.ftc.teamcode.graph;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Streams graph-friendly tuning signals (NAME: value) to FTControl Panels.
 *
 * This OpMode is aimed at classic Ziegler–Nichols closed-loop tuning:
 *  1) Set kI = 0, kD = 0.
 *  2) Increase kP until you get sustained oscillation (ultimate gain Ku).
 *  3) Measure oscillation period Pu (seconds).
 *  4) Use Z-N formulas to seed PID.
 *
 * This OpMode generates a repeatable heading setpoint "step" so you can see oscillations.
 */
@Configurable
@TeleOp(name = "Drive ZN Graph Tuner", group = "Tuning")
public class DriveZNTuner extends OpMode {
    // --- User-tunable parameters (live-editable in Panels) ---
    public static double STEP_DEG = 20.0;
    public static double STEP_PERIOD_S = 4.0;

    /** If true, will switch between +STEP and -STEP periodically. If false, setpoint is constant. */
    public static boolean TOGGLE_STEP = true;

    /** Target translational magnitude (inches) for the follower target pose. Keep small. */
    public static double HOLD_DISTANCE_IN = 0.0;

    /** If true, uses follower heading PIDF by commanding a target pose; otherwise just logs current pose. */
    public static boolean COMMAND_HEADING = true;

    // --- Runtime ---
    private TelemetryManager panels;
    private final ElapsedTime timer = new ElapsedTime();

    private Follower follower;

    // For rough Pu estimation: track last two zero-crossings of error.
    private double lastErr = Double.NaN;
    private double lastZeroCrossingT = Double.NaN;
    private double estHalfPeriod = Double.NaN;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new com.pedropathing.geometry.Pose());

        panels = PanelsTelemetry.INSTANCE.getTelemetry();

        timer.reset();

        panels.debug("Info", "Use Panels Graph: setpoint, measurement, error");
        panels.update(telemetry);
    }

    @Override
    public void start() {
        timer.reset();

        // Activate heading loop only (keeps this fairly safe/isolated).
        try {
            follower.deactivateAllPIDFs();
            follower.activateHeading();
        } catch (Exception ignored) {
            // Some follower versions might not expose these. We'll still provide graph signals.
        }
    }

    @Override
    public void loop() {
        follower.update();

        final double t = timer.seconds();

        final double currentHeadingRad = follower.getPose().getHeading();
        final double currentHeadingDeg = Math.toDegrees(currentHeadingRad);

        final double stepSign;
        if (!TOGGLE_STEP) {
            stepSign = 1.0;
        } else {
            // Alternate every STEP_PERIOD_S seconds: +1 for first half, -1 for second half.
            final double phase = (t % STEP_PERIOD_S) / STEP_PERIOD_S;
            stepSign = (phase < 0.5) ? 1.0 : -1.0;
        }

        final double setpointDeg = stepSign * STEP_DEG;
        final double setpointRad = Math.toRadians(setpointDeg);

        if (COMMAND_HEADING) {
            // Command a pose at the same position, with a desired heading offset from 0.
            // We assume starting pose heading ~0. If you want a different baseline, reset starting pose.
            try {
                com.pedropathing.geometry.Pose target = new com.pedropathing.geometry.Pose(HOLD_DISTANCE_IN, 0, setpointRad);
                // Many Pedro builds support a "hold point" style by following a tiny path.
                // If not supported, this will no-op and you can still read the signals.
                follower.holdPoint(target);
            } catch (Throwable ignored) {
                // Follower API variant doesn't have holdPoint.
            }
        }

        // Error (wrapped to [-180,180] for clean plots)
        double errorDeg = wrapDeg(setpointDeg - currentHeadingDeg);

        estimatePeriod(t, errorDeg);

        panels.debug("t", t);
        panels.debug("setpointDeg", setpointDeg);
        panels.debug("headingDeg", currentHeadingDeg);
        panels.debug("errorDeg", errorDeg);

        if (!Double.isNaN(estHalfPeriod)) {
            panels.debug("Pu_est_s", estHalfPeriod * 2.0);
        }

        // Helpful runtime info
        telemetry.addData("t", t);
        telemetry.addData("setpointDeg", setpointDeg);
        telemetry.addData("headingDeg", currentHeadingDeg);
        telemetry.addData("errorDeg", errorDeg);
        if (!Double.isNaN(estHalfPeriod)) {
            telemetry.addData("Pu_est_s", estHalfPeriod * 2.0);
        }

        panels.update(telemetry);
    }

    @Override
    public void stop() {
        try {
            follower.startTeleopDrive(true);
            follower.setTeleOpDrive(0, 0, 0, true);
        } catch (Exception ignored) {
        }
    }

    private void estimatePeriod(double t, double errorDeg) {
        if (Double.isNaN(lastErr)) {
            lastErr = errorDeg;
            return;
        }

        // Detect sign change across zero.
        if ((lastErr <= 0 && errorDeg > 0) || (lastErr >= 0 && errorDeg < 0)) {
            if (!Double.isNaN(lastZeroCrossingT)) {
                estHalfPeriod = t - lastZeroCrossingT;
            }
            lastZeroCrossingT = t;
        }

        lastErr = errorDeg;
    }

    private static double wrapDeg(double deg) {
        double d = deg;
        while (d > 180) d -= 360;
        while (d < -180) d += 360;
        return d;
    }
}

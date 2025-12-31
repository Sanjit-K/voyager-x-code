package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;
import java.util.Locale;

@TeleOp(name = "Spindexer TeleOp Tester", group = "Test")
public class SpindexerTeleopTester extends OpMode {
    private Spindexer spindexer;

    // previous button state used for edge detection
    private boolean prevLeftStick = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    // 0 = Kp, 1 = Ki, 2 = Kd
    private int selectedPidIndex = 0;

    @Override
    public void init() {
        // Replace these names with the ones in your robot configuration if different
        spindexer = new Spindexer(hardwareMap, "spindexerMotor", "spindexerAnalog");

        // start background polling so getLastMeasuredAngle() is kept up-to-date
        spindexer.startPolling(20);

        telemetry.addLine("Spindexer TeleOp Tester Initialized")
                .addData("Note", "Make sure motor & analog names match config");
        telemetry.update();
    }

    @Override
    public void loop() {
        final double PID_STEP = 0.001;
        // Edge-detect buttons
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;
        boolean dup = gamepad1.dpad_up;
        boolean ddown = gamepad1.dpad_down;
        boolean dleft = gamepad1.dpad_left;
        boolean dright = gamepad1.dpad_right;
        boolean leftStickBtn = gamepad1.left_stick_button;

        // cycle selected PID param when left stick button pressed
        if (leftStickBtn && !prevLeftStick) {
            selectedPidIndex = (selectedPidIndex + 1) % 3;
        }

        if (a) {
            // move to 0 deg
            spindexer.startMoveToAngle(0.0);
        }
        if (b) {
            // move to 120 deg
            spindexer.startMoveToAngle(120.0);
        }
        if (x) {
            // move to 240 deg
            spindexer.startMoveToAngle(240.0);
        }
        if (y) {
            // calibrate current reading as zero
            spindexer.calibrateSetCurrentAsZero();
        }

        if (lb) {
            // toggle reversal
            spindexer.setAngleReversed(!spindexer.isAngleReversed());
        }

        if (rb) {
            // immediate stop
            spindexer.stop();
        }

        // tuning adjustments (press once to change)
        if (dup && !prevDpadUp) {
            // increase selected PID term
            if (selectedPidIndex == 0) {
                spindexer.setPID(spindexer.getKp() + PID_STEP, spindexer.getKi(), spindexer.getKd());
            } else if (selectedPidIndex == 1) {
                spindexer.setPID(spindexer.getKp(), spindexer.getKi() + PID_STEP, spindexer.getKd());
            } else {
                spindexer.setPID(spindexer.getKp(), spindexer.getKi(), spindexer.getKd() + PID_STEP);
            }
         }
         if (ddown && !prevDpadDown) {
             // decrease Kp
            if (selectedPidIndex == 0) {
                spindexer.setPID(Math.max(0.0, spindexer.getKp() - PID_STEP), spindexer.getKi(), spindexer.getKd());
            } else if (selectedPidIndex == 1) {
                spindexer.setPID(spindexer.getKp(), Math.max(0.0, spindexer.getKi() - PID_STEP), spindexer.getKd());
            } else {
                spindexer.setPID(spindexer.getKp(), spindexer.getKi(), Math.max(0.0, spindexer.getKd() - PID_STEP));
            }
         }
         if (dright && !prevDpadRight) {
            spindexer.setPowerScaling(Math.min(1.0, Spindexer.POWER + 0.05));
        }
        if (dleft && !prevDpadLeft) {
            spindexer.setPowerScaling(Math.max(0.0, Spindexer.POWER - 0.05));
        }

        // Run PID update if busy
        if (spindexer.isBusy()) {
            spindexer.update();
        }

        // Telemetry output
        telemetry.addData("Busy", spindexer.isBusy());
        telemetry.addData("Measured Angle", String.format(Locale.US, "%.2f", spindexer.getLastMeasuredAngle()));
        telemetry.addData("Raw Voltage", String.format(Locale.US, "%.3f V", spindexer.getRawVoltage()));
        telemetry.addData("Error", String.format(Locale.US, "%.2f deg", spindexer.getLastError()));
        telemetry.addData("P/I/D", String.format(Locale.US, "%.3f / %.3f / %.4f", spindexer.getLastP(), spindexer.getLastI(), spindexer.getLastD()));
        telemetry.addData("Output", String.format(Locale.US, "%.3f", spindexer.getLastOutput()));
        telemetry.addData("Kp/Ki/Kd", String.format(Locale.US, "%.4f / %.4f / %.4f", spindexer.getKp(), spindexer.getKi(), spindexer.getKd()));
        String selectedName = selectedPidIndex == 0 ? "Kp" : selectedPidIndex == 1 ? "Ki" : "Kd";
        telemetry.addData("Selected PID", selectedName);
        telemetry.addData("PID Step", PID_STEP);
        telemetry.addData("PowerScaling", String.format(Locale.US, "%.2f", Spindexer.POWER));
        telemetry.addData("AngleReversed", spindexer.isAngleReversed());
        telemetry.addLine("Controls: A=0deg B=120 X=240 Y=cal zero LB=toggleRev RB=stop DP up/down=Kp +/- DP left/right=Power +/-");
        telemetry.update();

        prevDpadUp = dup;
        prevDpadDown = ddown;
        prevDpadLeft = dleft;
        prevDpadRight = dright;
        prevLeftStick = leftStickBtn;
    }

    @Override
    public void stop() {
        spindexer.stop();
        spindexer.stopPolling();
    }
}

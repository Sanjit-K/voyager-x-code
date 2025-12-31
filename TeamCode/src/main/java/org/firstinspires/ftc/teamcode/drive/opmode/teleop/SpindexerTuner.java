package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;

@TeleOp(name = "Spindexer Tuner", group = "Tuning")
public class SpindexerTuner extends OpMode {
    private Spindexer spindexer;

    // Toggle logic - using gamepad edge helpers (.wasPressed style) instead of manual prev flags

    // Tuning Menu State
    private int selectedParam = 0; // 0=Kp, 1=Ki, 2=Kd, 3=kStatic
    private static final String[] PARAM_NAMES = {"Kp", "Ki", "Kd", "kStatic"};
    private static final double VAL_STEP = 0.001;

    @Override
    public void init() {
        spindexer = new Spindexer(hardwareMap, "spindexerMotor", "spindexerAnalog");
        telemetry.addLine("Ready to Tune.");
        telemetry.addLine("Controls:");
        telemetry.addLine("  DPAD L/R: Select P/I/D/Static");
        telemetry.addLine("  DPAD U/D: Adjust Value");
        telemetry.addLine("  A/B/X/Y: Go to 0/90/180/270");
    }

    @Override
    public void loop() {
        // --- 1. Tuning Controls ---

        // Change Parameter Selection (use .WasPressed helpers)
        if (gamepad1.dpadRightWasPressed()) selectedParam = (selectedParam + 1) % 4;
        if (gamepad1.dpadLeftWasPressed())  selectedParam = (selectedParam - 1 + 4) % 4;

        // Adjust Value
        double change = 0.0;
        if (gamepad1.dpadUpWasPressed()) change = VAL_STEP;
        if (gamepad1.dpadDownWasPressed()) change = -VAL_STEP;

        // Apply change if any
        if (change != 0.0) {
            switch (selectedParam) {
                case 0: Spindexer.Kp += change; break;
                case 1: Spindexer.Ki += change; break;
                case 2: Spindexer.Kd += change; break;
                case 3: Spindexer.kStatic += change; break;
            }
            // Sanity check: prevent negative constants (except maybe static if you get weird)
            if (Spindexer.Kp < 0) Spindexer.Kp = 0;
            if (Spindexer.Ki < 0) Spindexer.Ki = 0;
            if (Spindexer.Kd < 0) Spindexer.Kd = 0;
            if (Spindexer.kStatic < 0) Spindexer.kStatic = 0;
        }

        // --- 2. Test Targets ---
        if (gamepad1.aWasPressed()) spindexer.startMoveToAngle(0);
        if (gamepad1.bWasPressed()) spindexer.startMoveToAngle(120);
        if (gamepad1.xWasPressed()) spindexer.startMoveToAngle(240);
        if (gamepad1.yWasPressed()) spindexer.startMoveToAngle(180);

        // Emergency Stop
        if (gamepad1.leftBumperWasPressed()) spindexer.stop();
        // Recalibrate Zero
        if (gamepad1.rightBumperWasPressed()) spindexer.calibrateSetCurrentAsZero();

        // --- 3. Run Loop ---
        spindexer.update();

        // --- 4. Telemetry ---
        telemetry.addData("Selected", PARAM_NAMES[selectedParam]);
        telemetry.addData("Value", getCurrentValue(selectedParam));
        telemetry.addData("Step Size", VAL_STEP);
        telemetry.addLine("-----------------");
        telemetry.addData("Meas Angle", "%.1f", spindexer.getCalibratedAngle());
        telemetry.addData("Error", "%.2f", spindexer.getLastError());
        telemetry.addData("Busy", spindexer.isBusy());
        telemetry.addLine("-----------------");
        telemetry.addData("Kp", "%.4f", Spindexer.Kp);
        telemetry.addData("Ki", "%.4f", Spindexer.Ki);
        telemetry.addData("Kd", "%.4f", Spindexer.Kd);
        telemetry.addData("kStatic", "%.4f", Spindexer.kStatic);
        telemetry.update();
    }

    private double getCurrentValue(int index) {
        switch (index) {
            case 0: return Spindexer.Kp;
            case 1: return Spindexer.Ki;
            case 2: return Spindexer.Kd;
            case 3: return Spindexer.kStatic;
        }
        return 0.0;
    }
}
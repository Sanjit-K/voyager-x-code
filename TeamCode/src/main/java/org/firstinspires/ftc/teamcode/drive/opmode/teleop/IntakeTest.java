package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;


@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends OpMode {
    private BarIntake barIntake;
    private Spindexer spindexer;
    private DigitalChannel distanceSensor;
    // distance sensor debounce
    private final int DISTANCE_DEBOUNCE_MS = 50;
    private final ElapsedTime distanceDebounceTimer = new ElapsedTime();
    private boolean distanceTriggered = false;

    private double currAngle = 0.0;

    @Override
    public void init(){
        barIntake = new BarIntake(hardwareMap, "barIntake", true);
        spindexer = new Spindexer(hardwareMap, "spindexerMotor", "spindexerAnalog");
        distanceSensor = hardwareMap.get(DigitalChannel.class, "distanceSensor");
        distanceSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void loop() {
        // read distance sensor with debounce: when sensor reads true for > DISTANCE_DEBOUNCE_MS,
        // perform same action as right bumper (advance angle)
        boolean sensorState = distanceSensor.getState();
        if (sensorState) {
            // start or continue timing
            if (!distanceTriggered) {
                if (distanceDebounceTimer.milliseconds() == 0) distanceDebounceTimer.reset();
                if (distanceDebounceTimer.milliseconds() > DISTANCE_DEBOUNCE_MS) {
                    // trigger the same action as right bumper
                    currAngle += 120.0;
                    currAngle %= 360.0;
                    spindexer.startMoveToAngle(currAngle);
                    distanceTriggered = true;
                }
            }
        } else {
            // reset debounce when sensor not detecting
            distanceDebounceTimer.reset();
            distanceTriggered = false;
        }

        if (gamepad1.aWasPressed()) {
            barIntake.spinIntake();
        } else if (gamepad1.bWasPressed()) {
            barIntake.spinOuttake();
        } else if (gamepad1.xWasPressed()){
            barIntake.stop();
        }

        if (gamepad1.rightBumperWasPressed()){
            currAngle += 120.0;
            currAngle %= 360.0;
            spindexer.startMoveToAngle(currAngle);
        } else if (gamepad1.leftBumperWasPressed()){
            currAngle -= 120.0;
            if (currAngle < 0.0){
                currAngle += 360.0;
            }
            spindexer.startMoveToAngle(currAngle);
        }

        if (gamepad1.yWasPressed()){
            currAngle = 0.0;
            spindexer.startMoveToAngle(0.0);
        }

        spindexer.update();
        // Telemetry: intake and spindexer status
        double intakePower = barIntake.getPower();
        String intakeMode = intakePower > 0.0 ? "INTAKE" : intakePower < 0.0 ? "OUTTAKE" : "STOPPED";
        telemetry.addData("BarIntake Power", String.format(java.util.Locale.US, "%.3f", intakePower));
        telemetry.addData("Intake Mode", intakeMode);
        telemetry.addData("Target Angle", String.format(java.util.Locale.US, "%.1f", currAngle));
        // measured calibrated angle from analog sensor
        double measured = spindexer.getCalibratedAngle();
        telemetry.addData("Measured Angle", String.format(java.util.Locale.US, "%.2f", measured));
        telemetry.update();
    }
}

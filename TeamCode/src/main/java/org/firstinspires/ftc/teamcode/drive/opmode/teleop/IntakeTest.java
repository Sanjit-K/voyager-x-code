package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.intake.BarIntake;
import org.firstinspires.ftc.teamcode.sorting.Spindexer;


@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends OpMode {
    private BarIntake barIntake;
    private Spindexer spindexer;

    @Override
    public void init(){
        barIntake = new BarIntake(hardwareMap, "barIntake", true);
        spindexer = new Spindexer(hardwareMap, "spindexerMotor", "spindexerAnalog", "distanceSensor");
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            barIntake.spinIntake();
        } else if (gamepad1.bWasPressed()) {
            barIntake.spinOuttake();
        } else if (gamepad1.xWasPressed()){
            barIntake.stop();
        }

        if (gamepad1.rightBumperWasPressed()){
            spindexer.advanceIntake();
        } else if (gamepad1.leftBumperWasPressed()){
            spindexer.retreatIntake();
        }

        if (gamepad1.yWasPressed()){
            spindexer.setIntakeIndex(0);
        }

        spindexer.update();
        // Telemetry: intake and spindexer status
        double intakePower = barIntake.getPower();
        String intakeMode = intakePower > 0.0 ? "INTAKE" : intakePower < 0.0 ? "OUTTAKE" : "STOPPED";
        telemetry.addData("BarIntake Power", String.format(java.util.Locale.US, "%.3f", intakePower));
        telemetry.addData("Intake Mode", intakeMode);

        // Spindexer state telemetry
        double measured = spindexer.getCalibratedAngle();
        telemetry.addData("Measured Angle", String.format(java.util.Locale.US, "%.2f", measured));
        telemetry.addData("Intake Index", spindexer.getIntakeIndex());
        char[] filled = spindexer.getFilled();
        telemetry.addData("Filled Slots", "[" + filled[0] + ", " + filled[1] + ", " + filled[2] + "]");
        telemetry.addData("Is Full", spindexer.isFull());
        telemetry.update();
    }
}

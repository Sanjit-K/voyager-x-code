package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.intake.BarIntake;

@TeleOp(name = "BarIntake Tester", group = "Test")
public class BarIntakeTester extends OpMode {
    private BarIntake barIntake;

    @Override
    public void init() {
        barIntake = new BarIntake(hardwareMap, "barIntake", true);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            barIntake.spinIntake();
        } else if (gamepad1.b) {
            barIntake.spinOuttake();
        } else {
            barIntake.stop();
        }

        telemetry.addData("BarIntake Power", barIntake.getPower());
        telemetry.update();
    }
}


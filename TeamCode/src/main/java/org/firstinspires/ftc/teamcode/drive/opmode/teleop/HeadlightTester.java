package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Headlight Tester", group = "TeleOp")
public class HeadlightTester extends OpMode {
    private Servo ledHeadlight;
    private boolean on = false;

    @Override
    public void init() {
        ledHeadlight = hardwareMap.get(Servo.class, "ledLight");
        ledHeadlight.setPosition(0.0);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            on = !on;
            ledHeadlight.setPosition(on ? 1.0 : 0.0);
        }

        telemetry.addData("Headlight", on ? "ON" : "OFF");
        telemetry.update();
    }
}

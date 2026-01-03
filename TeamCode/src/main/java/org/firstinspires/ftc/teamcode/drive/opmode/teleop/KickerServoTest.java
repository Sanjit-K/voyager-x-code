package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;

@TeleOp(name = "Kicker Servo Test", group = "Test")
public class KickerServoTest extends OpMode {
    private KickerServo kickerServo;
    private double currentPosition = 0.5;
    private static final double STEP = 0.01;

    @Override
    public void init() {
        kickerServo = new KickerServo(hardwareMap, "kickerServo");
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            kickerServo.kick();
            currentPosition = 0.84; // Update to match kick position
        } else if (gamepad1.bWasPressed()) {
            kickerServo.normal();
            currentPosition = 0.5; // Update to match normal position
        }

        telemetry.addData("Current Position", String.format("%.2f", currentPosition));
        telemetry.addData("Controls", "A: Kick, B: Normal, Dpad Up/Down: Adjust");
        telemetry.update();
    }
}


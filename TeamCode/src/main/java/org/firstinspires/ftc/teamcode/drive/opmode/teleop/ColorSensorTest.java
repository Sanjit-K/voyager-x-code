package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.sorting.ColorSensor;

@TeleOp(name = "Color Sensor Test", group = "Test")
public class ColorSensorTest extends OpMode {

    private ColorSensor colorSensor;

    @Override
    public void init() {
        colorSensor = new ColorSensor(hardwareMap, "colorSensor");
    }

    @Override
    public void loop() {
        float hue = colorSensor.getHueDegrees();
        char detection = colorSensor.detection();

        telemetry.addData("Hue", hue);
        telemetry.addData("Detected", detection);

        telemetry.update();
    }
}


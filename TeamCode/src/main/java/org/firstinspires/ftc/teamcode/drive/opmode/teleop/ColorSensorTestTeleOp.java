package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.turret.ColorSensor;
import java.util.Locale;

@TeleOp(name = "Color Sensor Test", group = "Test")
public class ColorSensorTestTeleOp extends OpMode {
    private ColorSensor colorSensor;
    private static final String DEVICE_NAME = "color"; // change this if your device is named differently

    @Override
    public void init() {
        try {
            colorSensor = new ColorSensor(hardwareMap, DEVICE_NAME);
            telemetry.addData("Status", "Initialized: %s", DEVICE_NAME);
        } catch (Exception e) {
            colorSensor = null;
            telemetry.addData("Status", "Color sensor NOT FOUND: %s", DEVICE_NAME);
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        int brightness = 0; // 0..255 (max channel)
        float hue = 0f;     // degrees [0..360)
        boolean detected = false;

        if (colorSensor != null) {
            // Read values
            brightness = colorSensor.getBrightness8bit();
            hue = colorSensor.getHueDegrees();
            detected = colorSensor.detection(); // true if hue > 50
        } else {
            telemetry.addData("Status", "Sensor not found: %s", DEVICE_NAME);
        }

        telemetry.addData("Hue (deg)", String.format(Locale.US, "%.1f", hue));
        telemetry.addData("Hue threshold (deg)", String.format(Locale.US, "%.1f", colorSensor != null ? colorSensor.getHueThresholdDegrees() : 0f));
        telemetry.addData("Brightness (0-255)", brightness);
        telemetry.addData("Detected (hue>50)", detected);
        telemetry.update();
    }
}

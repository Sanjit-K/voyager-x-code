package org.firstinspires.ftc.teamcode.turret;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**


 Discrete servo yaw control that aims at a target.

 Uses Pedro Pathing Follower pose in center-origin inches.*/public class PivotServo {

    private final Servo servo;
    // Servo calibration - 0° robot angle = 0.5 servo position
    private static final double SERVO_CENTER = 0.5;  // This is 0° on your robot
    private static final double DEGREES_PER_SERVO_UNIT = 240.0;  // ±120° range mapped to ±0.5 servo units

    // Limit the servo travel to ±120°
    private static final double YAW_MIN_DEG = -120.0;
    private static final double YAW_MAX_DEG = 120.0;

    private TelemetryManager telemetryM;

    public PivotServo(HardwareMap hardwareMap, String servoName) {
        this.servo = hardwareMap.get(Servo.class, servoName);
    }

    public void set(double position){
        servo.setPosition(position);
    }
    public double getPos(){
        return servo.getPosition();
    }


}
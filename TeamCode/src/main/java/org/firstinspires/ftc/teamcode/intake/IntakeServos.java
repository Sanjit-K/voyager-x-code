package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class IntakeServos {
    private final CRServo barFront;
    private final CRServo leftFront;
    private final CRServo rightFront;
    private final CRServo barBack;
    private final CRServo leftBack;
    private final CRServo rightBack;
    private static final double SLOW_WHEEL_POWER = 0.10;



    public IntakeServos(
            HardwareMap hardwareMap,
            String leftFrontName,
            String barFrontName,
            String rightFrontName,
            String leftBackName,
            String barBackName,
            String rightBackName
    ) {
        this.barFront = hardwareMap.get(CRServo.class, barFrontName);
        this.leftFront = hardwareMap.get(CRServo.class, leftFrontName);

        this.rightFront = hardwareMap.get(CRServo.class, rightFrontName);

        this.barBack = hardwareMap.get(CRServo.class, barBackName);
        barBack.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftBack = hardwareMap.get(CRServo.class, leftBackName);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        this.rightBack = hardwareMap.get(CRServo.class, rightBackName);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void enableFrontWheelsSlow() {
        // Keep same directions as enableFrontWheels(), just small magnitude
        leftFront.setPower(-SLOW_WHEEL_POWER);
        rightFront.setPower( SLOW_WHEEL_POWER);
    }

    public void enableBackWheelsSlow() {
        // Keep same directions as enableBackWheels(), just small magnitude
        leftBack.setPower( SLOW_WHEEL_POWER);
        rightBack.setPower(-SLOW_WHEEL_POWER);
    }

    public void enableFrontIntake() {
        enableFrontBar();
        enableFrontWheels();
    }
    public void enableFrontBar() {
        barFront.setPower(-1.0);
    }

    public void enableFrontWheels(){
        leftFront.setPower(-1.0);
        rightFront.setPower(1.0);
    }

    public void disableFrontBar(){
        barFront.setPower(0);
    }

    public void disableFrontWheels(){
        leftFront.setPower(0);
        rightFront.setPower(0);
    }
    public void disableFrontIntake() {
        disableFrontBar();
        disableFrontWheels();
    }

    public void enableBackBar() {
        barBack.setPower(1.0);
    }

    public void enableBackWheels(){
        leftBack.setPower(1.0);
        rightBack.setPower(-1.0);
    }

    public void disableBackBar(){
        barBack.setPower(0);
    }

    public void disableBackWheels(){
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void enableBackIntake() {
        enableBackBar();
        enableBackWheels();
    }

    public void disableBackIntake() {
        disableBackWheels();
        disableBackBar();
    }

    public void enableFrontReject() {
        barFront.setPower(1.0);
        leftFront.setPower(1.0);
        rightFront.setPower(-1.0);
    }

    public void enableBackReject() {
        barBack.setPower(-1.0);
        leftBack.setPower(-1.0);
        rightBack.setPower(1.0);
    }

    public void disableAll() {
        disableFrontIntake();
        disableBackIntake();
    }

    public void enableAllIntake() {
        enableFrontIntake();
        enableBackIntake();
    }

    public void enableAllReject() {
        enableFrontReject();
        enableBackReject();
    }
}
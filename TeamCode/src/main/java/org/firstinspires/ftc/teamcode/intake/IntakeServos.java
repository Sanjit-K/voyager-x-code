package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class IntakeServos {
    private final CRServo leftBottomFront;
    private final CRServo leftTopFront;

    private final CRServo rightBottomFront;
    private final CRServo rightTopFront;


    private final CRServo leftBottomBack;
    private final CRServo leftTopBack;

    private final CRServo rightBottomBack;
    private final CRServo rightTopBack;


    public IntakeServos(
            HardwareMap hardwareMap,
            String leftBottomFrontName,
            String leftTopFrontName,
            String rightBottomFrontName,
            String rightTopFrontName,
            String leftBottomBackName,
            String leftTopBackName,
            String rightBottomBackName,
            String rightTopBackName
    ) {
        this.leftBottomFront = hardwareMap.get(CRServo.class, leftBottomFrontName);
        this.leftTopFront = hardwareMap.get(CRServo.class, leftTopFrontName);
        this.rightBottomFront = hardwareMap.get(CRServo.class, rightBottomFrontName);
        this.rightTopFront = hardwareMap.get(CRServo.class, rightTopFrontName);

        this.leftBottomBack = hardwareMap.get(CRServo.class, leftBottomBackName);
        this.leftTopBack = hardwareMap.get(CRServo.class, leftTopBackName);
        this.rightBottomBack = hardwareMap.get(CRServo.class, rightBottomBackName);
        this.rightTopBack = hardwareMap.get(CRServo.class, rightTopBackName);
    }

    public void enableFrontIntake() {
        leftBottomFront.setPower(-1.0);
        leftTopFront.setPower(-1.0);
        rightBottomFront.setPower(1.0);
        rightTopFront.setPower(1.0);
    }

    public void disableFrontIntake() {
        leftBottomFront.setPower(0);
        leftTopFront.setPower(0);
        rightBottomFront.setPower(0);
        rightTopFront.setPower(0);
    }

    public void enableBackIntake() {
        leftBottomBack.setPower(1.0);
        leftTopBack.setPower(1.0);
        rightBottomBack.setPower(1.0);
        rightTopBack.setPower(1.0);
    }

    public void disableBackIntake() {
        leftBottomBack.setPower(0);
        leftTopBack.setPower(0);
        rightBottomBack.setPower(0);
        rightTopBack.setPower(0);
    }

    public void enableFrontReject() {
        leftBottomFront.setPower(-1.0);
        leftTopFront.setPower(-1.0);
        rightBottomFront.setPower(-1.0);
        rightTopFront.setPower(-1.0);
    }

    public void enableBackReject() {
        leftBottomBack.setPower(-1.0);
        leftTopBack.setPower(-1.0);
        rightBottomBack.setPower(-1.0);
        rightTopBack.setPower(-1.0);
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
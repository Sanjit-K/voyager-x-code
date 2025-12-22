package org.firstinspires.ftc.teamcode.sorting;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

//0 is too right.  1 is too left.  2 is too right.    outtake: 0 is too left, 1 is good, 2 is too right.

public class Spindexer {
    private final DcMotorEx spindexerMotor;

    public static int POSITION_1 = 0;
    public static int POSITION_2 = 100;
    public static int POSITION_3 = 200;
    public static double POWER = 0.5;


    public Spindexer(HardwareMap hardwareMap, String servoName) {
        this.spindexerMotor = hardwareMap.get(DcMotorEx.class, servoName);
        spindexerMotor.setTargetPosition(0);
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void goToPosition1() {
        spindexerMotor.setTargetPosition(POSITION_1);
    }

    public void goToPosition2() {
        spindexerMotor.setTargetPosition(POSITION_2);

    }

    public void goToPosition3() {
        spindexerMotor.setTargetPosition(POSITION_3);
    }

    public int getPosition() {
        return spindexerMotor.getCurrentPosition();
    }
}

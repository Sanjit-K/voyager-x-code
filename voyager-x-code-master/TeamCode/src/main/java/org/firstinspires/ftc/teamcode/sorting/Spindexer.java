package org.firstinspires.ftc.teamcode.sorting;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Spindexer {
    private final Servo spindexerServo;
    private char[] tracking = {'_', '_', '_'}; // '_' = empty, 'P' = purple, 'G' = green
    private int intakeIndex = 0; // Index of the current position (0, 1, or 2)

    // values below are placeholders
    private double shoot0 = 0.5; // Position for index 0
    private double shoot1 = 0.8333; // Position for index 1
    private double shoot2 = 0.1667; // Position for index 2
    private double intake0 = 0.0;  // Position for shooting index 0 ball
    private double intake1 = 0.3333;  // Position for shooting index 1 ball
    private double intake2 = 0.6667;  // Position for shooting index 2 ball


    public Spindexer(HardwareMap hardwareMap, String servoName) {
        this.spindexerServo = hardwareMap.get(Servo.class, servoName);
    }

    public void setPosition(double position) {
        spindexerServo.setPosition(position);
    }

    public void setIntakeIndex(int index){
        intakeIndex = index % 3;
        switch (intakeIndex) {
            case 0:
                setPosition(intake0);
                break;
            case 1:
                setPosition(intake1);
                break;
            case 2:
                setPosition(intake2);
                break;
        }
    }

    public void advanceIntake() {
        intakeIndex = (intakeIndex + 1) % 3;
        setIntakeIndex(intakeIndex);
    }

    public void retreatIntake() {
        intakeIndex = (intakeIndex + 2) % 3; // equivalent to (index - 1 + 3) % 3
        setIntakeIndex(intakeIndex);
    }

    public void setShootIndex(int index){
        index %= 3;
        switch (index) {
            case 0:
                setPosition(shoot0);
                break;
            case 1:
                setPosition(shoot1);
                break;
            case 2:
                setPosition(shoot2);
                break;
        }
    }

    public int getIntakeIndex(){
        return intakeIndex;
    }

    public double getPosition() {
        return spindexerServo.getPosition();
    }

    public void incrementPosition(double amount) {
        double newPosition = Math.min(1.0, spindexerServo.getPosition() + amount);
        setPosition(newPosition);
    }

    public void decrementPosition(double amount) {
        double newPosition = Math.max(0.0, spindexerServo.getPosition() - amount);
        setPosition(newPosition);
    }

}

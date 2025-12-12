package org.firstinspires.ftc.teamcode.sorting;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.shooting.KickerServo;

//0 is too right.  1 is too left.  2 is too right.    outtake: 0 is too left, 1 is good, 2 is too right.

public class Spindexer {
    private final Servo spindexerServo;
    private char[] filled = {'_', '_', '_'}; // '_' = empty, 'P' = purple, 'G' = green
    private int intakeIndex = 0; // Index of the current position (0, 1, or 2)

    // values below are placeholders
    private final double shoot0 = 0.5; // Position for index 0
    private final double shoot1 = 0.833; // Position for index 1
    private final double shoot2 = 0.167; // Position for index 2
    private final double intake0 = 0.0;  // Position for shooting index 0 ball
    private final double intake1 = 0.333;  // Position for shooting index 1 ball
    private final double intake2 = 0.667;  // Position for shooting index 2 ball


    public Spindexer(HardwareMap hardwareMap, String servoName) {
        this.spindexerServo = hardwareMap.get(Servo.class, servoName);
    }

    public void setPosition(double position) {
        spindexerServo.setPosition(position);
    }

    public void fillUp(){
        for(int i = 0; i<3;i++){
            filled[i] = '#';
        }
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

    public void setColorAtPos(char color, int index) {
        filled[index] = color;
    }
    public void setColorAtPos(char color){
        filled[intakeIndex] = color;
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

    public int getShootIndex() {
        double pos = spindexerServo.getPosition();

        // Compare servo's current position to the known shoot positions
        double d0 = Math.abs(pos - shoot0);
        double d1 = Math.abs(pos - shoot1);
        double d2 = Math.abs(pos - shoot2);

        // Return whichever index the position is closest to
        if (d0 < d1 && d0 < d2) return 0;
        if (d1 < d0 && d1 < d2) return 1;
        return 2;
    }

    public boolean isFull(){
        for (char c : filled){
            if (c == '_'){
                return false;
            }
        }
        return true;
    }

    public void clearTracking(){
        filled[0] = '_';
        filled[1] = '_';
        filled[2] = '_';
    }


    public int getIntakeIndex(){
        return intakeIndex;
    }

    public double getPosition() {
        return spindexerServo.getPosition();
    }

    public char[] getFilled(){
        return filled;
    }


    /* Debugging */

    public void incrementPosition(double amount) {
        double newPosition = Math.min(1.0, spindexerServo.getPosition() + amount);
        setPosition(newPosition);
    }

    public void decrementPosition(double amount) {
        double newPosition = Math.max(0.0, spindexerServo.getPosition() - amount);
        setPosition(newPosition);
    }



}

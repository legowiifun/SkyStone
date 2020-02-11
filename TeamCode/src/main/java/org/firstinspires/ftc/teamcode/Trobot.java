package org.firstinspires.ftc.teamcode;

//imports

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//class Trobot
public class Trobot {
    //define motors - if motor is added, add it here, right-click, click Generate, and select Getter and Setter
    private DcMotor leftDriveFront = null;
    private DcMotor rightDriveFront = null;
    private DcMotor leftDriveBack = null;
    private DcMotor rightDriveBack = null;

    private DcMotor elevator = null;
   //getters and setters for each motor
    public DcMotor getElevator() {
        return elevator;
    }

    public void setElevator(DcMotor elevator) {
        this.elevator = elevator;
    }

    public DcMotor getClaw() {
        return claw;
    }

    public void setClaw(DcMotor claw) {
        this.claw = claw;
    }

    private DcMotor claw = null;

    private Servo rightServo = null;
    private Servo leftServo = null;

    private Servo AutoServo = null;

    public Servo getAutoServo() {
        return AutoServo;
    }

    public void setAutoServo(Servo AutoServo) {
        this.AutoServo = AutoServo;
    }


    public DcMotor getLeftDriveFront() {
        return leftDriveFront;
    }

    public void setLeftDriveFront(DcMotor leftDriveFront) {
        this.leftDriveFront = leftDriveFront;
    }

    public DcMotor getRightDriveFront() {
        return rightDriveFront;
    }

    public void setRightDriveFront(DcMotor rightDriveFront) {
        this.rightDriveFront = rightDriveFront;
    }

    public DcMotor getLeftDriveBack() {
        return leftDriveBack;
    }

    public void setLeftDriveBack(DcMotor leftDriveBack) {
        this.leftDriveBack = leftDriveBack;
    }

    public DcMotor getRightDriveBack() {
        return rightDriveBack;
    }

    public void setRightDriveBack(DcMotor rightDriveBack) {
        this.rightDriveBack = rightDriveBack;
    }

    public Servo getRightServo() {
        return rightServo;
    }

    public void setRightServo(Servo rightServo) {
        this.rightServo = rightServo;
    }

    public Servo getLeftServo() {
        return leftServo;
    }

    public void setLeftServo(Servo leftServo) {
        this.leftServo = leftServo;
    }
}

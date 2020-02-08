package org.firstinspires.ftc.teamcode;

// imports

import com.qualcomm.robotcore.hardware.DcMotor;
import java.lang.Math;

public class Drive {

    private Trobot trobot = null;

    Drive(Trobot trobot) {
        this.trobot = trobot;
    }
    public int getTime() {
        return time;
    }
    private int time = 0;

    //set drive.driving - moving wheels
    public void driving(double leftPower, double rightPower,double speed) {
        trobot.getLeftDriveFront().setPower(speed*leftPower);
        trobot.getRightDriveFront().setPower(speed*rightPower);
        trobot.getLeftDriveBack().setPower(speed*leftPower);
        trobot.getRightDriveBack().setPower(speed*rightPower);
    }
//drive.straightDrive - drive straight
    public void straightDrive(int direction) {
        trobot.getLeftDriveFront().setPower(0.5*direction);
        trobot.getRightDriveFront().setPower(0.5*direction);
        trobot.getLeftDriveBack().setPower(0.5*direction);
        trobot.getRightDriveBack().setPower(0.5*direction);
    }
// drive.encoderDrive - I don't know what this is for
    public void encoderDrive(double speed, double distance) {

        double threadsPerCentimeter = ((1120*2)/(10*3.1415));

        trobot.getLeftDriveFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trobot.getRightDriveFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trobot.getLeftDriveBack().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trobot.getRightDriveBack().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        trobot.getLeftDriveFront().setTargetPosition((int)(distance * threadsPerCentimeter));
        trobot.getRightDriveFront().setTargetPosition((int)(distance * threadsPerCentimeter));
        trobot.getLeftDriveBack().setTargetPosition((int)(distance * threadsPerCentimeter));
        trobot.getRightDriveBack().setTargetPosition((int)(distance * threadsPerCentimeter));

        trobot.getLeftDriveFront().setPower(speed);
        trobot.getRightDriveFront().setPower(speed);
        trobot.getLeftDriveBack().setPower(speed);
        trobot.getRightDriveBack().setPower(speed);

        while (trobot.getLeftDriveFront().isBusy() && trobot.getRightDriveBack().isBusy()) {
        }

        trobot.getLeftDriveFront().setPower(0);
        trobot.getRightDriveFront().setPower(0);
        trobot.getLeftDriveBack().setPower(0);
        trobot.getRightDriveBack().setPower(0);
    }
    //drive.skyClaw - controls the Autonomous claw
    public void skyClaw(boolean down) {
        if(down) {
            trobot.getAutoServo().setPosition(0.5);
        } else {
            trobot.getAutoServo().setPosition(0);
        }
    }
//drive.move - moves the robot
    public void move(double speed, double distance) {
        if (speed > 1) {
            speed = 1;
        }
        if (distance > 0) {
            trobot.getLeftDriveFront().setPower(speed);
            trobot.getRightDriveFront().setPower(speed);
            trobot.getLeftDriveBack().setPower(speed);
            trobot.getRightDriveBack().setPower(speed);
        } else if (distance < 0) {
            trobot.getLeftDriveFront().setPower(-speed);
            trobot.getRightDriveFront().setPower(-speed);
            trobot.getLeftDriveBack().setPower(-speed);
            trobot.getRightDriveBack().setPower(-speed);
        }


        time = Math.abs((int)((distance/(72.5*speed))*1000));

    }

//drive.turnRight and drive.turnLeft - These are empty
    public void turnRight(double degrees) {

    }

    public void turnLeft(double degrees) {

    }
//drive.strafe - makes the robot strafe
    public void strafe(double power, int direction) {
        trobot.getLeftDriveFront().setPower(-power*direction);
        trobot.getRightDriveFront().setPower(power*direction);
        trobot.getLeftDriveBack().setPower(power*direction);
        trobot.getRightDriveBack().setPower(-power*direction);
    }
    //drive.elevator - moves the elevator up or down
    public void elevator (boolean up) {
        if (up) {
            trobot.getElevator().setPower(1);

        }
        else {
            trobot.getElevator().setPower(-1);
        }
        trobot.getElevator().setPower(0);
    }
    //drive.claw - controls the elevator claw
    public void claw (boolean open){
        if (open) {
            trobot.getClaw().setPower(1);
        }
        else {
            trobot.getClaw().setPower(-1);
        }
        trobot.getClaw().setPower(0);
    }
// drive.stop - stops the drive motors
    public void stop() {
        trobot.getLeftDriveFront().setPower(0);
        trobot.getRightDriveFront().setPower(0);
        trobot.getLeftDriveBack().setPower(0);
        trobot.getRightDriveBack().setPower(0);
    }
// drive.latch and drive.unlatch - latches and unlatches the hook servos
    public void latch() {
        trobot.getLeftServo().setPosition(0.5);
        trobot.getRightServo().setPosition(0.5);
    }

    public void unlatch() {
        trobot.getLeftServo().setPosition(0);
        trobot.getRightServo().setPosition(1);
    }
    /*to create new ones
    public void NAMEHERE(ParameterType parameter) {
        code here
    }

    to change motors - trobot.getMotor().setPower(power);
    to change servos - trobot.getServo().setPosition(position);
     */

}
package org.firstinspires.ftc.teamcode;

// imports

import com.qualcomm.robotcore.hardware.DcMotor;
import java.lang.Math;

public class Drive {

    // set new private variable trobot, with variable type Trobot to null

    private Trobot trobot = null;

    // run drive with a new trobot variable, with type Trobot
    Drive(Trobot trobot) {
        this.trobot = trobot;
    }
    //set variable getTime to time
    public int getTime() {
        return time;
    }
    // set time to 0
    private int time = 0;

    //set drive.driving - drive with gamepads
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

    public void skyClaw(boolean down) {
        if(down) {
            trobot.getAutoServo().setPosition(0.5);
        } else {
            trobot.getAutoServo().setPosition(0);
        }
    }

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


    public void turnRight(double degrees) {

    }

    public void turnLeft(double degrees) {

    }

    public void strafe(double power, int direction) {
        trobot.getLeftDriveFront().setPower(-power*direction);
        trobot.getRightDriveFront().setPower(power*direction);
        trobot.getLeftDriveBack().setPower(power*direction);
        trobot.getRightDriveBack().setPower(-power*direction);
    }
    public void elevator (boolean up) {
        if (up) {
            trobot.getElevator().setPower(1);

        }
        else {
            trobot.getElevator().setPower(-1);
        }
        trobot.getElevator().setPower(0);
    }
    public void claw (boolean open){
        if (open) {
            trobot.getClaw().setPower(1);
        }
        else {
            trobot.getClaw().setPower(-1);
        }
        trobot.getClaw().setPower(0);
    }

    public void stop() {
        trobot.getLeftDriveFront().setPower(0);
        trobot.getRightDriveFront().setPower(0);
        trobot.getLeftDriveBack().setPower(0);
        trobot.getRightDriveBack().setPower(0);
    }

    public void latch() {
        trobot.getLeftServo().setPosition(0.5);
        trobot.getRightServo().setPosition(0.5);
    }

    public void unlatch() {
        trobot.getLeftServo().setPosition(0);
        trobot.getRightServo().setPosition(1);
    }

    /*public void collector(boolean in, boolean stop) {
        if(in) {
            trobot.getIntakeLeft().setPower(-0.5);
            trobot.getIntakeRight().setPower(-0.5);
        }
        else if(!in && !stop) {
            trobot.getIntakeLeft().setPower(0.2);
            trobot.getIntakeRight().setPower(0.2);
        } else if(stop) {
            trobot.getIntakeLeft().setPower(0);
            trobot.getIntakeRight().setPower(0);
        }
    }*/
}
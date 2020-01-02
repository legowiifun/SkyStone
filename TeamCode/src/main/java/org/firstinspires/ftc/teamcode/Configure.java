package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Configure {
    private Trobot trobot = new Trobot();
    private HardwareMap hardwareMap = null;

    Configure(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public Trobot getTrobot() {
        return trobot;
}

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public void setHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void initialize() {
        initializeDrive();
        initializeServo();
        //initializeCollector();
    }

    public void initializeDrive() {
        trobot.setLeftDriveFront(hardwareMap.get(DcMotor.class, "front_left"));
        trobot.setRightDriveFront(hardwareMap.get(DcMotor.class, "front_right"));
        trobot.setLeftDriveBack(hardwareMap.get(DcMotor.class, "rear_left"));
        trobot.setRightDriveBack(hardwareMap.get(DcMotor.class, "rear_right"));

        trobot.setElevator(hardwareMap.get(DcMotor.class, "elevator"));
        trobot.setClaw(hardwareMap.get(DcMotor.class,"claw"));

        trobot.getLeftDriveFront().setDirection(DcMotor.Direction.FORWARD);
        trobot.getRightDriveFront().setDirection(DcMotor.Direction.FORWARD);
        trobot.getLeftDriveBack().setDirection(DcMotor.Direction.REVERSE);
        trobot.getRightDriveBack().setDirection(DcMotor.Direction.REVERSE);
    }

    public void initializeServo() {
        trobot.setAutoServo(hardwareMap.get(Servo.class, "auto_servo"));
        trobot.setRightServo(hardwareMap.get(Servo.class,"right_servo"));
        trobot.setLeftServo(hardwareMap.get(Servo.class, "left_servo"));
    }

    //public void initializeCollector() {
      //  trobot.setIntakeLeft(hardwareMap.get(DcMotor.class, "left intake"));
        //trobot.setIntakeRight(hardwareMap.get(DcMotor.class, "right intake"));

        //trobot.getIntakeLeft().setDirection(DcMotor.Direction.FORWARD);
        //trobot.getIntakeRight().setDirection(DcMotor.Direction.REVERSE);
    //}

}
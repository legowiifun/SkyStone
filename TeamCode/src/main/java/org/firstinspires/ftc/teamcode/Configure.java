package org.firstinspires.ftc.teamcode;

//imports
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//class configure
public class Configure {
    private Trobot trobot = new Trobot();
    private HardwareMap hardwareMap = null;
//Configure.Configure -  Configure hardware map
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
//Configure.initialize - initialize the motors
    public void initialize() {
        initializeDrive();
        initializeServo();
        //initializeCollector();
    }
//initializeDrive - iniitialize the drive motors and the elevator and claw motors - if you change the names or add motors, change them here
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
//initializeServo - iniitialize the Servos - if you change the names or add motors, change them here as well
    public void initializeServo() {
        trobot.setAutoServo(hardwareMap.get(Servo.class, "auto_servo"));
        trobot.setRightServo(hardwareMap.get(Servo.class,"right_servo"));
        trobot.setLeftServo(hardwareMap.get(Servo.class, "left_servo"));
    }

}
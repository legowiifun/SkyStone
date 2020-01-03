package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="move forward",group = "Autonomous")

public class AutomonousMoveForward extends LinearOpMode {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive2 = null;
    private DcMotor elevatorMotor = null;
    private ElapsedTime     runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.5;

    public void runOpMode() {
        leftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        rightDrive = hardwareMap.get(DcMotor.class, "front_right");
        rightDrive2 = hardwareMap.get(DcMotor.class, "rear_right");
        leftDrive2 = hardwareMap.get(DcMotor.class, "rear_left");
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        telemetry.addData("Path0",  "Starting at %7d :%7d");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        leftDrive.setPower(1);
        rightDrive.setPower(1);
        leftDrive2.setPower(1);
        rightDrive2.setPower(1);
        sleep(100);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive2.setPower(0);
        rightDrive2.setPower(0);
    }
}

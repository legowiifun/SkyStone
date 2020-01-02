// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.

package org.firstinspires.ftc.teamcode;

//imports

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="test autonomous", group="Autonomous")
//@Disabled

public class FoundationTest extends LinearOpMode {

    // private variable called runtime that is equal to new ElapsedTime
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFront, rightFront, leftRear, rightRear;
    Servo leftServo, rightServo;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .40, correction;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

//set motors
        leftFront = hardwareMap.dcMotor.get("front_left");
        rightFront = hardwareMap.dcMotor.get("front_right");
        leftRear = hardwareMap.dcMotor.get("rear_left");
        rightRear = hardwareMap.dcMotor.get("rear_right");

        leftServo = hardwareMap.servo.get("left_servo");
        rightServo = hardwareMap.servo.get("right_servo");
//directions of motors
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setPower(0.5);
        leftRear.setPower(-0.5);
        rightFront.setPower(-0.5);
        rightRear.setPower(0.5);
        sleep (1000);
        leftFront.setPower(-0.5);
        leftRear.setPower(-0.5);
        rightFront.setPower(-0.5);
        rightRear.setPower(-0.5);
        sleep(1000);
        leftFront.setPower(0.5);
        leftRear.setPower(0.5);
        rightFront.setPower(0.5);
        rightRear.setPower(0.5);
        sleep(2000);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

    }
}
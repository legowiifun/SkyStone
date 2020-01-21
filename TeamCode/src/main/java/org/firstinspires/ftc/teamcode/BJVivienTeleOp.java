package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;

@TeleOp(name="BJ & Vivien TeleOp", group="Linear Opmode")

public class BJVivienTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive2 = null;
    private DcMotor elevatorMotor = null;
    private DcMotor clawMotor = null;

    private double mode = 1;
    private double mode2 = 1;

    private Servo autoServo = null;
    private Servo leftServo = null;
    private Servo rightServo = null;

    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        rightDrive = hardwareMap.get(DcMotor.class, "front_right");
        rightDrive2 = hardwareMap.get(DcMotor.class, "rear_right");
        leftDrive2 = hardwareMap.get(DcMotor.class, "rear_left");
        autoServo = hardwareMap.get(Servo.class, "auto_servo");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator");
        clawMotor = hardwareMap.get(DcMotor.class, "claw");
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Path0",  "Starting at %7d",
                elevatorMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        autoServo.setPosition(0.5);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Setup a variable for each drive wheel to save power level for telemetry
            boolean DPadLeft = gamepad1.dpad_left;
            boolean DPadRight = gamepad1.dpad_right;
            boolean DPadUp = gamepad1.dpad_up;
            boolean DPadDown = gamepad1.dpad_down;

            double triggerL = gamepad1.left_trigger;
            double triggerR = gamepad1.right_trigger;

            boolean bumperL = gamepad1.left_bumper;
            boolean bumperR = gamepad1.right_bumper;

            boolean buttonA = gamepad1.a;
            boolean buttonB = gamepad1.b;
            boolean buttonX = gamepad1.x;
            boolean buttonY = gamepad1.y;

            double leftStick = -gamepad1.left_stick_y;
            double rightStick = gamepad1.right_stick_y;

            boolean DPadLeft2 = gamepad2.dpad_left;
            boolean DPadRight2 = gamepad2.dpad_right;
            boolean DPadUp2 = gamepad2.dpad_up;
            boolean DPadDown2 = gamepad2.dpad_down;

            double triggerL2 = gamepad2.left_trigger;
            double triggerR2 = gamepad2.right_trigger;

            boolean bumperL2 = gamepad2.left_bumper;
            boolean bumperR2 = gamepad2.right_bumper;

            boolean buttonA2 = gamepad2.a;
            boolean buttonB2 = gamepad2.b;
            boolean buttonX2 = gamepad2.x;
            boolean buttonY2 = gamepad2.y;

            double leftStick2 = -gamepad2.left_stick_y;
            double rightStick2 = gamepad2.right_stick_y;



            //Set mode
            if (buttonA) {
                mode = 1;
            }
            else if (buttonA2) {
                mode = 1;
            }
            if (buttonB){
                mode = 0.5;
            }
            else if (buttonB2) {
                mode = 0.5;
            }



            // Send calculated power to wheels
            if ((Math.abs(leftStick) >= 0.1)  || (Math.abs(rightStick) >= 0.1)) {
                leftDrive.setPower(leftStick * mode);
                rightDrive.setPower(rightStick * mode);
                leftDrive2.setPower(leftStick * mode);
                rightDrive2.setPower(rightStick * mode);
            }
            else if (Math.abs(leftStick2) >=0.1 || Math.abs(rightStick2) >= 0.1) {
                leftDrive.setPower(leftStick * mode2);
                rightDrive.setPower(rightStick * mode2);
                leftDrive2.setPower(leftStick * mode2);
                rightDrive2.setPower(rightStick * mode2);
            }
            else {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                leftDrive2.setPower(0);
                rightDrive2.setPower(0);
            }

            //Elevator Claw
            if (bumperL) {
                clawMotor.setPower(-1);
            }
            else if (bumperL2) {
                clawMotor.setPower(-1);
            }
            else{
                clawMotor.setPower(0);
            }

            if (bumperR) {
                clawMotor.setPower(1);
            }
            else if (bumperR2) {
                clawMotor.setPower(1);
            }
            else{
                clawMotor.setPower(0);
            }




            //Elevator
            if(triggerL >= 0.1){
                elevatorMotor.setPower(-triggerL);
            }
            else if (triggerL2 >= 0.1) {
                elevatorMotor.setPower(-triggerL2);
            }
            else{
                elevatorMotor.setPower(0);
            }

            if(triggerR >= 0.1){
                elevatorMotor.setPower(triggerR);
            }
            else if (triggerR2 >= 0.1) {
                elevatorMotor.setPower(triggerR2);
            }
            else {
                elevatorMotor.setPower(0);
            }




            //elevator lift
            if (buttonX){
                leftServo.setPosition(0);
                rightServo.setPosition(0);
            }
            else if (buttonX2) {
                leftServo.setPosition(0);
                rightServo.setPosition(0);
            }
            if (buttonY){
                leftServo.setPosition(1);
                rightServo.setPosition(1);
            }
            else if (buttonY2) {
                leftServo.setPosition(1);
                rightServo.setPosition(1);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftStick, rightStick);
            telemetry.update();

        }
    }
}
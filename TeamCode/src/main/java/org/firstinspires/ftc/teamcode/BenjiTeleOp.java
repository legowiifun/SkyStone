package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Benji TeleOp", group="Linear Opmode")

public class BenjiTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    boolean servoCheck = false;
    String servoStatus = "Unlatched";

    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Configure configure = new Configure(hardwareMap);
        configure.initialize();
        Trobot trobot = configure.getTrobot();
        Drive drive = new Drive(trobot);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

double speed = 1;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            drive.driving(gamepad1.left_stick_y, gamepad1.right_stick_y, speed); // Send calculated power to wheels

            boolean DPadLeft = gamepad1.dpad_left;
            boolean DPadLeft2 = gamepad2.dpad_left;
            boolean DPadRight = gamepad1.dpad_right;
            boolean DPadRight2 = gamepad2.dpad_right;
            boolean DPadUp = gamepad1.dpad_up;
            boolean DPadUp2 = gamepad2.dpad_up;
            boolean DPadDown = gamepad1.dpad_down;
            boolean DPadDown2 = gamepad2.dpad_down;
            boolean bumperL = gamepad1.left_bumper;
            boolean bumperL2 = gamepad2.left_bumper;
            boolean bumperR = gamepad1.right_bumper;
            boolean bumperR2 = gamepad2.right_bumper;
            double triggerL = gamepad1.left_trigger;
            double triggerL2 = gamepad2.left_trigger;
            double triggerR = gamepad1.right_trigger;
            double triggerR2 = gamepad2.right_trigger;
            boolean A = gamepad1.a;
            boolean A2 = gamepad2.a;
            boolean Y = gamepad1.y;
            boolean Y2 = gamepad2.y;
            boolean B = gamepad1.b;
            boolean B2 = gamepad2.b;
            boolean X = gamepad1.x;
            boolean X2 = gamepad2.x;
                if (DPadLeft || DPadLeft2) {
                    //strafe to the left
                    drive.strafe(1, 1);
                }
                if (DPadRight || DPadRight2) {
                    //strafe to the right
                    drive.strafe(1, -1);
                }
                if (DPadUp || DPadUp2) {
                    //go forward
                    drive.driving(1, 1,1);
                }
                if (DPadDown || DPadDown2) {
                    //go backward
                    drive.driving(-1, -1,1);
                }
                if (bumperL || bumperL2) {
                    //close the claw
                    drive.claw(false);
                }
                if (bumperR || bumperR2) {
                    //open the claw
                    drive.claw(true);
                }
                if (triggerL == 1 || triggerL2 == 1) {
                    //elevator up
                    drive.elevator(true);
                }
                if (triggerR == 1 || triggerR2 == 1) {
                    //elevator down
                    drive.elevator(false);
                }
                if (A || A2) {

                    //fast mode
                    speed = 1;
                }

                if (B || B2) {
                    //slow mode
                    speed = 0.5;
                }
                if (X || X2) {

                    // attach to foundation
                    drive.latch();
                }
                if (Y || Y2) {
                    // unlatch from foundation
                    drive.unlatch();
                }
            }
        }
    }
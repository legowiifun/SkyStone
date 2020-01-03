package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Andrew TeleOp", group="Linear Opmode")

public class AndrewTeleOp extends LinearOpMode {

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

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            drive.driving(gamepad1.left_stick_y, gamepad1.right_stick_y, 0.5); // Send calculated power to wheels

            boolean DPadLeft = gamepad1.dpad_left;
            boolean DPadRight = gamepad1.dpad_right;
            boolean DPadUp = gamepad1.dpad_up;
            boolean DPadDown = gamepad1.dpad_down;
            boolean bumperL = gamepad1.left_bumper;
            boolean bumperR = gamepad1.right_bumper;
            double triggerL = gamepad1.left_trigger;
            double triggerR = gamepad1.right_trigger;
            boolean A = gamepad1.a;
            boolean Y = gamepad1.y;
            boolean B = gamepad1.b;
            boolean X = gamepad1.x;
                if (DPadLeft) {
                    //strafe to the left
                    drive.strafe(1, 1);
                }
                if (DPadRight) {
                    //strafe to the right
                    drive.strafe(1, -1);
                }
                if (DPadUp) {
                    //go forward
                    drive.driving(1, 1,1);
                }
                if (DPadDown) {
                    //go backward
                    drive.driving(-1, -1,1);
                }
                if (bumperL) {
                    //autonomous claw down
                    drive.skyClaw(true);
                }
                if (bumperR) {
                    //autonomous claw up
                    drive.skyClaw(false);
                }
                if (triggerL == 1) {
                    //grab foundation
                    drive.latch();
                }
                if (triggerR == 1) {
                    // release foundation
                    drive.unlatch();
                }
                if (A) {

                    // elevator goes to bottom
                    drive.elevator(false);
                }

                if (B) {
                    // open the claw
                    drive.claw(true);
                }
                if (X) {

                    // close the claw
                    drive.claw(false);
                }
                if (Y) {
                    // elevator goes to top
                    drive.elevator(true);
                }
            }
        }
    }
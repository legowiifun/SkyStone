package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Move Forward Autonomous wait strafe", group="Autonomous")

public class MoveForwardAutonomousStrafeWait extends LinearOpMode {
    public void runOpMode() {
        Configure configure = new Configure(hardwareMap);
        configure.initialize();
        Trobot trobot = configure.getTrobot();

        Drive drive = new Drive(trobot);

        waitForStart();
        sleep(25000);
        drive.strafe(1,1);
        drive.stop();
        drive.driving(1,1,1);
        sleep(1000);
        drive.stop();
    }
}

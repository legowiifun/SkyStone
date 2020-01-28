package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Move Forward Autonomous Waiting", group="Autonomous")

public class MoveForwardAutonomousWait extends LinearOpMode {
    public void runOpMode() {
        Configure configure = new Configure(hardwareMap);
        configure.initialize();
        Trobot trobot = configure.getTrobot();

        Drive drive = new Drive(trobot);

        waitForStart();
        sleep(25000);
        drive.driving(1,1,1);
        sleep(1000);
        drive.stop();
    }
}

//starts the same as normal
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Move Forward Autonomous", group="Autonomous")

public class TrobotixTestAutonomous extends LinearOpMode {
    public void runOpMode() {
      //put these lines in this area of the program
        Configure configure = new Configure(hardwareMap);
        configure.initialize();
        Trobot trobot = configure.getTrobot();

        Drive drive = new Drive(trobot);

        waitForStart();
        // end area of lines
        drive.driving(1,1,1);
        sleep(1000);
        drive.stop();

    }
}

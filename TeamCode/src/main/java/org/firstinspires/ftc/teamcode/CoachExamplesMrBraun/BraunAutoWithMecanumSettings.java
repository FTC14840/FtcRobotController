package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled

// Register class as Autonomous on Driver Station
@Autonomous(name = "Braun Autonomous")

    // Begin class and extend methods for LinearOpMode
    public class BraunAutoWithMecanumSettings extends LinearOpMode {

        // Create a new instance of the hardware class
        BraunMecanumSettings robot = new BraunMecanumSettings();

        // Override the method runOpMode from LinearOpMode
        @Override
        public void runOpMode() throws InterruptedException {

            // Run method from hardware class
            robot.initHardware(this);
            robot.calibrateGyro(this);

            // Wait for the drive to press play
            waitForStart();

            // Repeat this code once play is pressed until stop is pressed
            while (opModeIsActive()) {

                // Run these methods from the hardware setup to move the bot
                robot.gyroDrive(12, .20, 0, 5000);
//                robot.gyroLeft(.20, -90, 5000);
//                robot.gyroRight(.20, 0, 5000);
//                robot.gyroDrive(-12, .20, 0, 5000);


            }
        }
    }
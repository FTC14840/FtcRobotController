package org.firstinspires.ftc.teamcode.TeamLeadJonathan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.BraunMecanumSettings;


// @Disabled

// Register class as Autonomous on Driver Station
@Autonomous(name = "Jonathan Autonomous")

    // Begin class and extend methods for LinearOpMode
    public class JonathanAutoWithMecanumSettings extends LinearOpMode {

        // Create a new instance of the hardware class
        BraunMecanumSettings robot = new BraunMecanumSettings();

        // Override the method runOpMode from LinearOpMode
        @Override
        public void runOpMode() throws InterruptedException {

            // Run method from hardware class

            robot.initHardware(this);
            robot.calibrateGyro(this);
            robot.activateTfod();
            //robot.tfodTelemetry();

            while (!isStarted()) {
                robot.tfodTelemetry();
            }

            // Wait for the drive to press play
            waitForStart();

            robot.deactivedTfod();

            // Repeat this code once play is pressed until stop is pressed
            while (opModeIsActive()) {

//                if (robot.tfodTelemetry() == "single") {
//                    robot.gyroLeft(.20, -90, 5000);
//                    Thread.sleep(50000);
//                }
//
//                if (robot.tfodTelemetry() == "double") {
//                    robot.gyroRight(.20, 90, 5000);
//                    Thread.sleep(50000);
//                }
//
//                if (robot.tfodTelemetry() == "quad") {
//                    robot.gyroDrive(12, .20, 0, 5000);
//                    Thread.sleep(50000);
//                }



                // Run these methods from the hardware setup to move the bot
                robot.gyroDrive(12, .20, 0, 5000);
//                robot.gyroLeft(.20, -90, 5000);
//                robot.gyroRight(.20, 0, 5000);
//                robot.gyroDrive(-12, .20, 0, 5000);


            }
        }
    }
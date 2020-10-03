package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

// Register class as Autonomous on Driver Station - Place your name first
@Autonomous(name = "Braun Test Autonomous")

// @Disabled

    // Begin class and extend methods for LinearOpMode - Place your name first
    public class BraunAutoWithMecanumSettings extends LinearOpMode {

        // Create a new instance of the hardware class
        BraunMethods robot = new BraunMethods();

    // Override the method runOpMode from LinearOpMode
        @Override
        public void runOpMode() throws InterruptedException {

            // Run method from hardware class
            robot.initHardware(this);
            robot.initVuforia(this);
            robot.initTfod(this);
            robot.calibrateGyro(this);

            while (!isStarted()) {
                robot.tfodTelemetry(this);
            }

            // Wait for the drive to press play
            waitForStart();

            robot.stopTfod(this);


            // Repeat this code once play is pressed until stop is pressed


            if (robot.tfodDetected == "Single") {

                telemetry.log().clear();
                telemetry.addData("Detected", "Single");
                telemetry.update();
                robot.gyroDrive(12, .20, 0, 5000);
            }

            if (robot.tfodDetected == "Double") {

                telemetry.log().clear();
                telemetry.addData("Detected", "Double");
                telemetry.update();
            }

            if (robot.tfodDetected == "Quad") {

                telemetry.log().clear();
                telemetry.addData("Detected", "Quad");
                telemetry.update();



            }



                // Run these methods from the hardware setup to move the bot
//                robot.gyroDrive(12, .20, 0, 5000);
//                robot.gyroLeft(.20, -90, 5000);
//                robot.gyroRight(.20, 0, 5000);
//                robot.gyroDrive(-12, .20, 0, 5000);


            Thread.sleep(5000);

        }
    }
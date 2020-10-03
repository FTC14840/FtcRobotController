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

            // Example Movements
                // robot.gyroForward(12, .20, 0, 5000);
                // robot.gyroReverse(12, .20, 0, 5000);
                // robot.gyroStrafeLeft(12, .20, 0, 5000);
                // robot.gyroStrafeRight(12, .20, 0, 5000);
                // robot.gyroLeft(.20, -90, 5000);
                // robot.gyroRight(.20, 90, 5000);

            if (robot.getTfodDetected() == "Single") {

                telemetry.log().clear();
                telemetry.addData("Detected", "Single");
                telemetry.update();
                robot.stopTfod(this);
                // robot.driveTelemetry(this);

                robot.gyroLeft(.20, -90, 5000);

            }

            if (robot.getTfodDetected() == "Double") { //Default if nothing detected

                telemetry.log().clear();
                telemetry.addData("Detected", "Double");
                telemetry.update();
                robot.stopTfod(this);
                // robot.driveTelemetry(this);

                robot.gyroForward(12, .20, 0, 5000);
            }

            if (robot.getTfodDetected() == "Quad") {

                telemetry.log().clear();
                telemetry.addData("Detected", "Quad");
                telemetry.update();
                robot.stopTfod(this);
                // robot.driveTelemetry(this);

                robot.gyroRight(.20, 90, 5000);
            }








        }
    }
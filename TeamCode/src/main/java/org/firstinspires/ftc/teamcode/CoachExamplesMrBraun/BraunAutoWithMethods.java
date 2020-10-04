package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Register class as Autonomous on Driver Station - Place your name first
@Autonomous(name = "Braun Test Autonomous")

// @Disabled

    // Begin class and extend methods for LinearOpMode - Place your name first
    public class BraunAutoWithMethods extends LinearOpMode {

        // Create a new instance of the hardware class
        BraunMethods robot = new BraunMethods();

    // Override the method runOpMode from LinearOpMode
        @Override
        public void runOpMode() throws InterruptedException {

            // Run method from hardware class
            robot.initHardware(this);
            robot.initTfod(this);
            robot.calibrateGyro(this);

            while (!isStarted()) {
                robot.tfodTelemetry(this);
            }

            // Wait for the drive to press play
            waitForStart();

            robot.deactivateTfod();

            // Still need to research threads so we can do two things at once.

            // Example Movements
                // robot.gyroForward(12, .20, 0, 5000);
                // robot.gyroReverse(12, .20, 0, 5000);
                // robot.gyroStrafeLeft(12, .20, 0, 5000);
                // robot.gyroStrafeRight(12, .20, 0, 5000);
                // robot.gyroLeft(.20, -90, 5000);
                // robot.gyroRight(.20, 90, 5000);

            if (robot.getTfodDetected() == "None") { //Default

                telemetry.log().clear();
                telemetry.addData("Detected", "None");
                telemetry.update();
                robot.stopTfod(this);
                // robot.driveTelemetry(this);

                robot.gyroForward(12, .20, 0, 5000);
            }

            if (robot.getTfodDetected() == "Single") {

                telemetry.log().clear();
                telemetry.addData("Detected", "Single");
                telemetry.update();
                robot.stopTfod(this);
                // robot.driveTelemetry(this);

                robot.gyroLeft(.20, -90, 5000);

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
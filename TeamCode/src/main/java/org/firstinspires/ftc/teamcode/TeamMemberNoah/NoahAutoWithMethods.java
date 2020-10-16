// Package name
package org.firstinspires.ftc.teamcode.TeamMemberNoah;

// Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.BraunMethods;

// Register class as Autonomous on Driver Station - Place your name first
@Autonomous(name = "Noah Test Autonomous")

// @Disabled

// Begin class and extend methods from LinearOpMode - Place your name first
public class NoahAutoWithMethods extends LinearOpMode {

    // Create a new instance of the methods class
    BraunMethods robot = new BraunMethods();

    // Override the method runOpMode from LinearOpMode
    @Override
    public void runOpMode() throws InterruptedException {

        // Run these init methods from the hardware class
        robot.initHardware(this);
        robot.initVisionTracking(this);
        robot.activateCruiseControl();
        robot.calibrateGyro(this);

        // While waiting for the driver to press play, show TFOD telemety
        while (!isStarted()) {
            //robot.tfodTelemetry(this);
        }

        // Press play to begin
        waitForStart();

        //robot.zoomTfod();

        // Deactivate TFOD to save resources
        //robot.stopTfod(this);

        // Working: We still need to research threads so we can do two things at once.
        // Create an instance for a new thread
//        Thread braunRunnableThread = new Thread (new BraunRunnableThread());

        // Start the thread
//        braunRunnableThread.start();

        // Interrupt the second thread
//        braunRunnableThread.interrupt();

        // To run the tread again, you need a new instance.  The same instance cannot be run again.

        /** Example Movements
        // robot.gyroForward(12, .20, 0, 5000);
        // robot.gyroReverse(12, .20, 0, 5000);
        // robot.gyroStrafeLeft(12, .20, 0, 5000);
        // robot.gyroStrafeRight(12, .20, 0, 5000);
        // robot.gyroLeft(.20, -90, 5000);
        // robot.gyroRight(.20, 90, 5000);
        **/

        if (robot.getTfodDetected() == "Quad") {

            telemetry.log().clear();
            telemetry.addData("Detected", "Quad - Tfod Worked");
            telemetry.update();
            robot.stopTfod(this);

            robot.gyroRight(.20, 5, 0);

        } else if (robot.getTfodDetected() == "Single") {

            telemetry.log().clear();
            telemetry.addData("Detected", "Single");
            telemetry.update();
            robot.stopTfod(this);

            robot.gyroLeft(.20, -90, 5000);

        } else {

            telemetry.log().clear();
            telemetry.addData("Detected", "None");
            telemetry.update();
            robot.stopTfod(this);

            // robot.gyroForward(12, .20, 0, 5000);

            while (robot.targetsAreVisible()) {
                robot.cruiseControlTelemetry();
                if (robot.cruiseControl(1500)) {
                    break;
                } else {
                    robot.cruiseControl(1500);
                    robot.moveRobot();
                    robot.cruiseControlTelemetry();
                    Thread.sleep(1000);
                }
            }
        }
    }
}





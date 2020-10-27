// Package name
package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun;

// Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Register class as Autonomous on Driver Station - Place your name first
@Autonomous(name = "Braun Test Autonomous")

@Disabled

// Begin class and extend methods from LinearOpMode - Place your name first
public class BraunAutoWithMethods extends LinearOpMode {

    // Create a new instance of the methods class
    BraunMethods robot = new BraunMethods();

    // Override the method runOpMode from LinearOpMode
    @Override
    public void runOpMode() throws InterruptedException {

        // Run these init methods from the hardware class
        robot.initHardware(this);
        //robot.initTracking(this);
        //robot.activateCruiseControl(this);
        //robot.initTfod(this);
        robot.calibrateGyro(this);

        // While waiting for the driver to press play, show TFOD telemety
        while (!isStarted()) {
            //robot.tfodTelemetry(this);
        }

        // Press play to begin
        waitForStart();

        //robot.stopTfod(this);

        //robot.zoomReset();
        //Thread.sleep(30000);

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
            telemetry.addData("Running", "Quad Program");
            robot.cruiseControlTelemetry();
            telemetry.update();
            sleep(30000);

            //robot.gyroRight(.20, 5, 0);

        } else if (robot.getTfodDetected() == "Single") {

            telemetry.log().clear();
            telemetry.addData("Running", "Single Program");
            robot.cruiseControlTelemetry();
            telemetry.update();
            sleep(30000);

            //robot.gyroLeft(.20, -90, 5000);

        } else {

            telemetry.log().clear();
            telemetry.addData("Running", "Default Program");
            //robot.cruiseControlTelemetry();
            telemetry.update();
            //sleep(30000);

             robot.gyroForward(12, .20, 0, 0);
             robot.gyroReverse(12, .20, 0, 0);
             robot.gyroStrafeLeft(12, .20, 0, 0);
             robot.gyroStrafeRight(12, .20, 0, 0);
             robot.gyroLeft(.20, -90, 0);
             robot.gyroRight(.20, 90, 0);

//            while (robot.targetsAreVisible()) {
//                robot.cruiseControlTelemetry();
//                if (robot.cruiseControl(1500)) {
//                    break;
//                } else {
//                    robot.cruiseControl(1500);
//                    robot.moveRobot();
//                    robot.cruiseControlTelemetry();
//                    Thread.sleep(1000);
//                }
//            }
        }
    }
}





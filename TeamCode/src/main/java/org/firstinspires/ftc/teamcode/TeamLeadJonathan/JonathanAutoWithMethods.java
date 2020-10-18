// Package name
package org.firstinspires.ftc.teamcode.TeamLeadJonathan;

// Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.BraunMethods;

// Register class as Autonomous on Driver Station - Place your name first
@Autonomous(name = "Jonathan Test Autonomous")

@Disabled

// Begin class and extend methods from LinearOpMode - Place your name first
public class JonathanAutoWithMethods extends LinearOpMode {

    // Create a new instance of the methods class
    JonathanMethods robot = new JonathanMethods();

    // Override the method runOpMode from LinearOpMode
    @Override
    public void runOpMode() throws InterruptedException {

        // Run these init methods from the hardware class
        robot.initHardware(this);
        robot.initTfod(this);
        robot.calibrateGyro(this);

        // While waiting for the driver to press play, show TFOD telemety
        while (!isStarted()) {
            robot.tfodTelemetry(this);
        }

        // Press play to begin
        waitForStart();

        // Deactivate TFOD to save resources
        robot.deactivateTfod();

        // We still need to research threads so we can do two things at once.

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
            telemetry.addData("Detected", "Quad");
            telemetry.update();
            robot.stopTfod(this);

            robot.gyroForward(10,.5,0,0);
            robot.gyroReverse(15,.5,0,0);
            robot.gyroRight(.35,90,0);
            robot.gyroRight(.35,90,0);
            robot.gyroRight(.35,90,0);
            robot.gyroRight(.35,90,0);
            robot.gyroLeft(.35,90,0);
            robot.gyroLeft(.35,90,0);
            robot.gyroLeft(.35,90,0);
            robot.gyroLeft(.35,90,0);


        } else if (robot.getTfodDetected() == "Single") {

            telemetry.log().clear();
            telemetry.addData("Detected", "Single");
            telemetry.update();
            robot.stopTfod(this);

            robot.gyroForward(10, .3, 0,0);
            robot.gyroReverse(15,.3,0,0);
            robot.gyroForward(10,.3,0,0);

        } else {

            telemetry.log().clear();
            telemetry.addData("Detected", "None");
            telemetry.update();
            robot.stopTfod(this);

            robot.gyroLeft(.15,90,0);
            robot.gyroRight(.15,90,0);
            robot.gyroRight(.15,90,0);
            robot.gyroLeft(.15,90,0);

        }
    }
}
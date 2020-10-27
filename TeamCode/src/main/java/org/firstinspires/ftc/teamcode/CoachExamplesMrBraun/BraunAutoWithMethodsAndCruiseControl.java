// Package name
package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun;

// Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Register class as TeleOp on Driver Station - Place your name first
@Autonomous(name="Braun Cruise Control Test")

@Disabled

// Begin class and extend methods from LinearOpMode - Place your name first
public class BraunAutoWithMethodsAndCruiseControl extends LinearOpMode {

    // Create a new instance of the hardware class
    BraunMethods robot = new BraunMethods();

    // Override the method runOpMode from LinearOpMode
    @Override
    public void runOpMode() throws InterruptedException {

        // Run these init methods from the hardware class
        robot.initHardware(this);
        //robot.initAllVision(this);
        robot.activateCruiseControl();
        robot.calibrateGyro(this);

        // While waiting for the driver to press play, show TFOD telemety
        while (!isStarted()) {
            robot.tfodTelemetry(this);
        }

        // Press play to begin
        waitForStart();

        //robot.zoomReset();

        /** Example Movements
        robot.gyroForward(12, .20, 0, 5000);
        robot.gyroReverse(12, .20, 0, 5000);
        robot.gyroStrafeLeft(12, .20, 0, 5000);
        robot.gyroStrafeRight(12, .20, 0, 5000);
        robot.gyroLeft(.20, -90, 5000);
        robot.gyroRight(.20, 90, 5000);

        while (opModeIsActive() && robot.targetsAreVisible() && !robot.cruiseControl(TARGET_DISTANCE)) {
            robot.autoCruiseControl();
        }
         **/

        if (robot.getTfodDetected() == "Quad") {

            telemetry.log().clear();
            telemetry.addData("Detected", "Quad");
            telemetry.update();

            robot.gyroRight(.20, 5, 0);

        } else if (robot.getTfodDetected() == "Single") {

            telemetry.log().clear();
            telemetry.addData("Detected", "Single");
            telemetry.update();

            robot.gyroLeft(.20, -90, 5000);

        } else {

            telemetry.log().clear();
            telemetry.addData("Detected", "None");
            telemetry.update();

//            robot.gyroForward(12, .20, 0, 5000);
//
//            Thread.sleep(5000);

            while (opModeIsActive() && robot.targetsAreVisible() && !robot.cruiseControl(1500)) {
                //robot.autoCruiseControl(1500);
            }
        }
    }
}

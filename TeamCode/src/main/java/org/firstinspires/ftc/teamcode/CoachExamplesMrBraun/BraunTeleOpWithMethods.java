// Package name
package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun;

// Imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Register class as TeleOp on Driver Station - Place your name first
@TeleOp(name="Braun Test Teleop")

//@Disabled

// Begin class and extend methods from LinearOpMode - Place your name first
public class BraunTeleOpWithMethods extends LinearOpMode {

    // Hold robot's center 400 mm from target
    final double TARGET_DISTANCE =  1524.0;

    // Create a new instance of the hardware class
    BraunMethods robot = new BraunMethods();

    // Override the method runOpMode from LinearOpMode
    @Override
    public void runOpMode() throws InterruptedException {

        // Run these init methods from the hardware class
        robot.initHardware(this);
        robot.initVisionTracking(this);
        robot.activateCruiseControl();

        // Press play to begin
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Add message for cruise control
            telemetry.addData(">", "Press Left Bumper to track target");

            // logic for manual and cruise control
            if (robot.targetsAreVisible() && gamepad1.left_bumper) {
                robot.cruiseControl(TARGET_DISTANCE);
            } else {
                robot.manualDrive();
            }

            // Build telemetry messages with Navigation Information;
            robot.cruiseControlTelemetry();

            //  Move the robot according to the pre-determined axis motions
            robot.moveRobot();
            telemetry.update();
        }
    }
}

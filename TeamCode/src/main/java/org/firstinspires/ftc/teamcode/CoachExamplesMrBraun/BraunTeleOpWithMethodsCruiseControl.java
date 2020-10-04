package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FinalTeleOp;

@TeleOp(name="Cruise Control Test")

//@Disabled

public class BraunTeleOpWithMethodsCruiseControl extends LinearOpMode {

    final double TARGET_DISTANCE =  400.0;    // Hold robot's center 400 mm from target

    // Create a new instance of the hardware class
    BraunMethods robot = new BraunMethods();


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the robot and navigation
        robot.initHardware(this);
        robot.initVuforiaVision(this);

        // Activate Vuforia (this takes a few seconds)
        robot.cruiseControlTracking();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            // Prompt User
            telemetry.addData(">", "Press start");

            // Display any Nav Targets while we wait for the match to start
            robot.targetsAreVisible();
            robot.cruiseControlTelemetry();
            telemetry.update();
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData(">", "Press Left Bumper to track target");

            // auto drive or manual drive?
            // In auto drive, the robot will approach any target it can see and then press against it
            // In manual drive the robot responds to the Joystick.
            if (robot.targetsAreVisible() && gamepad1.left_bumper) {
                // Calculate automatic target approach
                robot.cruiseControl(TARGET_DISTANCE);

            } else {
                // Drive the robot using the joysticks
                robot.manualDrive();
            }

            // Build telemetry messages with Navigation Information;
            robot.cruiseControlTelemetry();

            //  Move the robot according to the pre-determined axis motions
            robot.moveRobot();
            telemetry.update();
        }

        telemetry.addData(">", "Shutting Down. Bye!");
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.CruiseControl.Robot_Navigation;
import org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.CruiseControl.Robot_OmniDrive;

@TeleOp(name="Cruise Control Test", group="main")

//@Disabled

public class BraunTeleOpWithMethodsCruiseControl extends LinearOpMode {

    final double TARGET_DISTANCE =  400.0;    // Hold robot's center 400 mm from target

    /* Declare OpMode members. */
    Robot_OmniDrive robot    = new Robot_OmniDrive();   // Use Omni-Directional drive system
    Robot_Navigation nav      = new Robot_Navigation();  // Use Image Tracking library

    @Override
    public void runOpMode() {

        // Initialize the robot and navigation
        robot.initDrive(this);
        nav.initVuforia(this, robot);

        // Activate Vuforia (this takes a few seconds)
        nav.activateTracking();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            // Prompt User
            telemetry.addData(">", "Press start");

            // Display any Nav Targets while we wait for the match to start
            nav.targetsAreVisible();
            nav.addNavTelemetry();
            telemetry.update();
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData(">", "Press Left Bumper to track target");

            // auto drive or manual drive?
            // In auto drive, the robot will approach any target it can see and then press against it
            // In manual drive the robot responds to the Joystick.
            if (nav.targetsAreVisible() && gamepad1.left_bumper) {
                // Calculate automatic target approach
                nav.cruiseControl(TARGET_DISTANCE);

            } else {
                // Drive the robot using the joysticks
                robot.manualDrive();
            }

            // Build telemetry messages with Navigation Information;
            nav.addNavTelemetry();

            //  Move the robot according to the pre-determined axis motions
            robot.moveRobot();
            telemetry.update();
        }

        telemetry.addData(">", "Shutting Down. Bye!");
        telemetry.update();
    }
}

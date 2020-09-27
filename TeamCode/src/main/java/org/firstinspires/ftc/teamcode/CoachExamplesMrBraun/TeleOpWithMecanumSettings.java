// Package name
package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun;

// Imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Register class as TeleOp on Driver Station
@TeleOp(name ="TeleOp With Mecanum Settings")

// Begin class and extend methods for LinearOpMode
public class TeleOpWithMecanumSettings extends LinearOpMode {

    // Create a new instance of the hardware class
    MecanumSettings robot = new MecanumSettings();

    // Override the method runOpMode from LinearOpMode
    @Override
    public void runOpMode() {

        // Run method from hardware class
        robot.initDrive(this);
        robot.calibrateGyro(this);

        // Do this code block until play is pressed
        while (!isStarted()) {

        }

        // Wait for the drive to press play
        waitForStart();

        // Repeat this code once play is pressed until stop is pressed
        while (opModeIsActive()) {

            // Run these methods from the hardware setup to move the bot
            robot.manualDrive();
            robot.moveRobot();
            robot.driveTelemetry(this);
        }
    }
}
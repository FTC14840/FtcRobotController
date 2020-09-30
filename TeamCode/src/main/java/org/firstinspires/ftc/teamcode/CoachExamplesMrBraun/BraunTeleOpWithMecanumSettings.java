// Package name
package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun;

// Imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// @Disabled

// Register class as TeleOp on Driver Station
@TeleOp(name ="Braun TeleOp")
//HeeHee
// Begin class and extend methods for LinearOpMode
public class BraunTeleOpWithMecanumSettings extends LinearOpMode {

    // Create a new instance of the hardware class
    BraunMecanumSettings robot = new BraunMecanumSettings();

    // Override the method runOpMode from LinearOpMode
    @Override
    public void runOpMode() throws InterruptedException {

        // Run method from hardware class
        robot.initHardware(this);
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
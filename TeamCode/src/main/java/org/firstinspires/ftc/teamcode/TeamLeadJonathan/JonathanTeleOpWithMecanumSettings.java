// Package name
package org.firstinspires.ftc.teamcode.TeamLeadJonathan;

// Imports
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.BraunMecanumSettings;
import org.firstinspires.ftc.teamcode.MecanumSettings;

@Disabled

// Register class as TeleOp on Driver Station
@TeleOp(name ="Jonathan TeleOp")
//HeeHee
// Begin class and extend methods for LinearOpMode
public class JonathanTeleOpWithMecanumSettings extends LinearOpMode {

    // Create a new instance of the hardware class
    JonathanMecanumSettings robot = new JonathanMecanumSettings();

    // Override the method runOpMode from LinearOpMode
    @Override
    public void runOpMode() throws InterruptedException {

        // Run method from hardware class
        robot.initHardware(this);
        robot.activateTfod();
        robot.calibrateGyro(this);

        // Do this code block until play is pressed
        while (!isStarted()) {
            robot.tfodTelemetry();
        }

        // Wait for the drive to press play
        waitForStart();

        robot.deactivedTfod();

        // Repeat this code once play is pressed until stop is pressed
        while (opModeIsActive()) {

            // Run these methods from the hardware setup to move the bot
            robot.manualDrive();
            robot.moveRobot();
            robot.driveTelemetry(this);

        }
    }
}
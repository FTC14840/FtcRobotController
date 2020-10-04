// Package name
package org.firstinspires.ftc.teamcode.TeamMemberBekah;

// Imports
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled

// Register class as TeleOp on Driver Station - Place your name first
@TeleOp(name ="Bekah Test TeleOp")

// Begin class and extend methods for LinearOpMode - Place your name first
public class BekahTeleOpWithMecanumSettings extends LinearOpMode {

    // Create a new instance of the hardware class
    BekahMecanumSettings robot = new BekahMecanumSettings();

    // Override the method runOpMode from LinearOpMode
    @Override
    public void runOpMode() throws InterruptedException {

        // Run method from hardware class
        robot.initHardware(this);
        robot.initVision(this);
        robot.calibrateGyro(this);

        // Do this code block until play is pressed
        while (!isStarted()) {
            robot.targetsAreVisible();
        }

        // Wait for the drive to press play
        waitForStart();

        //robot.deactiveTfod();

        // Repeat this code once play is pressed until stop is pressed
        while (opModeIsActive()) {

            if (robot.targetsAreVisible() && gamepad1.left_bumper) {
                robot.cruiseControl(robot.PRIMARYDISTANCE);

            } else if(robot.targetsAreVisible() && gamepad1.right_bumper){
                robot.cruiseControl(robot.SECONDARYDISTANCE);

            } else {
                robot.manualDrive();
            }

            robot.moveRobot();
            robot.navigationTelemetry();
        }
    }
}
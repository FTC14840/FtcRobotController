package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ShortTeleOp")

public class ShortTeleOp extends LinearOpMode {

    MechWarriorCode robot = new MechWarriorCode();

    final double TARGET_DISTANCE =  1625.6;  // 64 inches (1625.6 mm) sets at edge of white line inside launch area.
    final double LAUNCHER_SPEED = 0.80;  // Set default launcher speed
    final double DEGREES_OFFSET = 0;  // Zero places the bot on the mid-line perpendicular to the target
    final double RELATIVE_BEARING = 0;  // Adjust the orientation of the bot in relation to offset

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initHardware(this);
        robot.initVisionTracking(this);

        while (!isStarted()) {
            robot.initTelemetry();
        }

        waitForStart();

        while (opModeIsActive()) {

            robot.startBlinkinLeds();

            if (robot.targetsAreVisible() && gamepad1.left_bumper) {
                robot.cruiseControl(TARGET_DISTANCE);
            } else {
                robot.manualDrive();
            }

            robot.moveRobot();
            robot.auxilaryControls();
            robot.cruiseControlTelemetry();

        }
    }
}

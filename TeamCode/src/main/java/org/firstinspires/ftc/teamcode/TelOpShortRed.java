package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TelOpShortRed")

public class TelOpShortRed extends LinearOpMode {

    MechWarriorCode robot = new MechWarriorCode();

    final double TARGET_DISTANCE =  1650;  // 64 inches (1625.6 mm) sets at edge of white line inside launch area.
    final double LAUNCHER_SPEED = 0.80;  // Set default launcher speed
    final double POWERSHOT_DISTANCE = 1800;
    final double POWERSHOT_OFFSET = 30;  // Zero places the bot on the mid-line perpendicular to the target
    final double POWERSHOT_BEARING = -30;  // Adjust the orientation of the bot in relation to offset

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initHardware(this);
        robot.initVisionTracking(this);

        while (!isStarted()) {
            robot.initTelemetry();
        }

        waitForStart();

        robot.initAuxiliaryControls(LAUNCHER_SPEED);

        while (opModeIsActive()) {

            robot.startBlinkinRed();

            if (robot.targetsAreVisible() && gamepad1.left_bumper) {
                robot.cruiseControl(TARGET_DISTANCE);
            } else if(robot.targetsAreVisible() && gamepad1.right_bumper) {
                robot.redPowerShot(POWERSHOT_DISTANCE, POWERSHOT_OFFSET, POWERSHOT_BEARING);
            } else {
                robot.manualDrive();
            }

            robot.moveRobot();
            robot.auxiliaryControls();
            robot.cruiseControlTelemetry();

        }
    }
}

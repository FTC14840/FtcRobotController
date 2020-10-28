package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="LongTeleOp")

@Disabled

public class LongTeleOp extends LinearOpMode {

    MechWarriorCode robot = new MechWarriorCode();

    final double TARGET_DISTANCE =  1625.6;  // 64 inches (1625.6 mm) from target is back edge of white line
    final double LAUNCHER_SPEED = 0.80;
    final double DEGREES_OFF_MIDLINE = 0;
    final double RELATIVE_BEARING = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initHardware(this);
        robot.initVisionTracking(this);

        while (!isStarted()) {
            robot.initTelemetry();
        }

        waitForStart();

        while (opModeIsActive()) {

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

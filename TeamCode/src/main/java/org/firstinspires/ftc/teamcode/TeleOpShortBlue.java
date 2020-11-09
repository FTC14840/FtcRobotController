package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOpShortBlue")

public class TeleOpShortBlue extends LinearOpMode {

    MechWarriorCode robot = new MechWarriorCode();

    final double LAUNCHER_SPEED = 0.80;

    final double CRUISE_CONTROL_RANGE = 1500;
    final double CRUISE_CONTROL_OFFSET = 0;
    final double CRUISE_CONTROL_ANGLE = 0;
    final double CRUISE_CONTROL_AXIAL_GAIN = 0.0030;
    final double CRUISE_CONTROL_LATERAL_GAIN = 0;
    final double CRUISE_CONTROL_YAW_GAIN = 0.0400;

    final double POWERSHOT_RANGE = 1500;
    final double POWERSHOT_OFFSET = 0;
    final double POWERSHOT_ANGLE = -10;
    final double POWERSHOT_RANGE_AXIAL_GAIN = 0.0030;
    final double POWERSHOT_RANGE_LATERAL_GAIN = 0;
    final double POWERSHOT_RANGE_YAW_GAIN = 0.0400;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initHardware(this);
        robot.initVisionTracking(this);
        robot.calibrateGyro(this);

        while (!isStarted()) {
            robot.initTelemetry();
        }

        waitForStart();

        robot.initAuxiliaryControls(LAUNCHER_SPEED);

        while (opModeIsActive()) {

            robot.startBlinkinBlue();

            if (robot.targetsAreVisible() && gamepad1.left_bumper) {
                robot.cruiseControl(CRUISE_CONTROL_RANGE, CRUISE_CONTROL_OFFSET, CRUISE_CONTROL_ANGLE,
                        CRUISE_CONTROL_AXIAL_GAIN, CRUISE_CONTROL_LATERAL_GAIN, CRUISE_CONTROL_YAW_GAIN);
            } else if(robot.targetsAreVisible() && gamepad1.right_bumper) {
                robot.powerShot(POWERSHOT_RANGE, POWERSHOT_OFFSET, POWERSHOT_ANGLE,
                        POWERSHOT_RANGE_AXIAL_GAIN, POWERSHOT_RANGE_LATERAL_GAIN, POWERSHOT_RANGE_YAW_GAIN);
            } else {
                    robot.manualDrive();
            }
            robot.moveRobot();
            robot.auxiliaryControls();
            robot.cruiseControlTelemetry();

        }
    }
}

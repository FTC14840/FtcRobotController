package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOpBlue")

public class TeleOpBlue extends LinearOpMode {

    MechWarriorCode robot = new MechWarriorCode();

    final double CRUISE_CONTROL_RANGE = 1900;
    final double CRUISE_CONTROL_OFFSET = 0;
    final double CRUISE_CONTROL_ANGLE = 5;
    final double CRUISE_CONTROL_AXIAL_GAIN = 0.0030;
    final double CRUISE_CONTROL_LATERAL_GAIN = 0;
    final double CRUISE_CONTROL_YAW_GAIN = 0.0800;

    final double POWERSHOT_RANGE = 2100;
    final double POWERSHOT_OFFSET = 0;
    final double POWERSHOT_ANGLE = -12;
    final double POWERSHOT_RANGE_AXIAL_GAIN = 0.0030;
    final double POWERSHOT_RANGE_LATERAL_GAIN = 0;
    final double POWERSHOT_RANGE_YAW_GAIN = 0.0800;

    final double POWERSHOTL_RANGE = 2100;
    final double POWERSHOTL_OFFSET = 0;
    final double POWERSHOTL_ANGLE = -7;
    final double POWERSHOTL_RANGE_AXIAL_GAIN = 0.0030;
    final double POWERSHOTL_RANGE_LATERAL_GAIN = 0;
    final double POWERSHOTL_RANGE_YAW_GAIN = 0.0800;

    final double POWERSHOTR_RANGE = 2100;
    final double POWERSHOTR_OFFSET = 0;
    final double POWERSHOTR_ANGLE = -17;
    final double POWERSHOTR_RANGE_AXIAL_GAIN = 0.0030;
    final double POWERSHOTR_RANGE_LATERAL_GAIN = 0;
    final double POWERSHOTR_RANGE_YAW_GAIN = 0.0800;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initHardware(this);
        robot.initVisionTracking(this);
        robot.calibrateGyro(this);
        robot.magazineSetup(.02, 2000);

        while (!isStarted()) {
            robot.initTelemetry();

        }

        waitForStart();

        robot.initAuxiliaryControls();

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

            if(robot.targetsAreVisible() && gamepad1.dpad_left) {
                robot.powerShot(POWERSHOTL_RANGE, POWERSHOTL_OFFSET, POWERSHOTL_ANGLE,
                        POWERSHOTL_RANGE_AXIAL_GAIN, POWERSHOTL_RANGE_LATERAL_GAIN, POWERSHOTL_RANGE_YAW_GAIN);
            }

            if(robot.targetsAreVisible() && gamepad1.dpad_right) {
                robot.powerShot(POWERSHOTR_RANGE, POWERSHOTR_OFFSET, POWERSHOTR_ANGLE,
                        POWERSHOTR_RANGE_AXIAL_GAIN, POWERSHOTR_RANGE_LATERAL_GAIN, POWERSHOTR_RANGE_YAW_GAIN);
            }
            robot.moveRobot();
            robot.auxiliaryControls();
            robot.cruiseControlTelemetry();

        }
    }
}

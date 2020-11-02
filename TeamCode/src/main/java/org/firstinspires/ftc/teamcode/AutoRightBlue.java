package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name ="AutoRightBlue")

//@Disabled

public class AutoRightBlue extends LinearOpMode {

    MechWarriorCode robot = new MechWarriorCode();

    final double LAUNCHER_SPEED = 0.80;
    final double POWERSHOT_RANGE = 1700;
    final double POWERSHOT_OFFSET = 500;
    final double POWERSHOT_ANGLE = -10;
    final double POWERSHOT_RANGE_AXIAL_GAIN = 0.0030;
    final double POWERSHOT_RANGE_LATERAL_GAIN = 0; //.0030;
    final double POWERSHOT_RANGE_YAW_GAIN = 0.0400;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initHardware(this);
        robot.initTfod(this);
        robot.calibrateGyro(this);

        while (!isStarted()) {
            robot.tfodInitTelemetry();
        }

        waitForStart();

        //robot.stopTfod(this);

        //robot.signalBlueAlliance();

        robot.initAutonomousPowerUp(LAUNCHER_SPEED);

        /** Example Movements
         // robot.gyroForward(12, .20, 0, 500);
         // robot.gyroReverse(12, .20, 0, 500);
         // robot.gyroStrafeLeft(12, .20, 0, 500);
         // robot.gyroStrafeRight(12, .20, 0, 000);
         // robot.gyroLeft(.20, 90, 5000;
         // robot.gyroRight(.20, -90, 500);
         **/

        if (robot.getTfodDetected() == "Quad") {

            robot.tfodRunningTelemetry();


        } else if (robot.getTfodDetected() == "Single") {

            robot.tfodRunningTelemetry();


        } else {

            robot.tfodRunningTelemetry();
            robot.gyroForward(20,.5,0,250);
//            robot.gyroRight(.50,-30,250);
//            robot.gyroForward(13,.5,-30,250);
//            robot.gyroLeft(.50,0,250);
//            robot.gyroForward(12,.5,0,250);

            while (robot.targetsAreVisible() && opModeIsActive()) {

                robot.powerShot(POWERSHOT_RANGE, POWERSHOT_OFFSET, POWERSHOT_ANGLE, POWERSHOT_RANGE_AXIAL_GAIN, POWERSHOT_RANGE_LATERAL_GAIN, POWERSHOT_RANGE_YAW_GAIN);
                robot.moveRobot();
                robot.ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                robot.cruiseControlTelemetry();
                if(robot.readyToShot(POWERSHOT_RANGE)) {
                    robot.ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    break;
                }
            }

            sleep(30000);

        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name ="AutoRightBlueTestCruiseControl")

public class AutoRightBlueTestCruiseControl extends LinearOpMode {

    MechWarriorCode robot = new MechWarriorCode();

    final double CRUISE_CONTROL_RANGE = 2000;
    final double CRUISE_CONTROL_OFFSET = 0;
    final double CRUISE_CONTROL_ANGLE = 5;
    final double CRUISE_CONTROL_AXIAL_GAIN = 0.0030;
    final double CRUISE_CONTROL_LATERAL_GAIN = 0;
    final double CRUISE_CONTROL_YAW_GAIN = 0.0400;

    final double POWERSHOT_RANGE = 2000;
    final double POWERSHOT_OFFSET = 0;
    final double POWERSHOT_ANGLE = -12;
    final double POWERSHOT_RANGE_AXIAL_GAIN = 0.0030;
    final double POWERSHOT_RANGE_LATERAL_GAIN = 0;
    final double POWERSHOT_RANGE_YAW_GAIN = 0.0400;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initHardware(this);
        robot.initTfod(this);
        robot.calibrateGyro(this);
        robot.magazineSetup(.02,2000);

        while (!isStarted()) {
            robot.initTfodTelemetry();
        }

        waitForStart();

        robot.activateVision(this);

        /** Example Movements
         // robot.gyroForward(12, .20, 0, 250);
         // robot.gyroReverse(12, .20, 0, 250);
         // robot.gyroStrafeLeft(12, .20, 0, 250);
         // robot.gyroStrafeRight(12, .20, 0, 250);
         // robot.gyroLeft(.20, 90, 250;
         // robot.gyroRight(.20, -90, 250);
         **/

        //robot.tfodRunningTelemetry();
        robot.signalBlueAlliance();
        robot.prepareLauncher(820);
        robot.gyroForward(63,0.60,-30,0);
        robot.gyroLeft(.30,5,0);
        robot.raiseMagazine();
        Thread.sleep(250);
        robot.cruiseControlTelemetry();
        while (robot.targetsAreVisible() && opModeIsActive() && time < 20) {
            robot.cruiseControl(CRUISE_CONTROL_RANGE, CRUISE_CONTROL_OFFSET, CRUISE_CONTROL_ANGLE,
                    CRUISE_CONTROL_AXIAL_GAIN, CRUISE_CONTROL_LATERAL_GAIN, CRUISE_CONTROL_YAW_GAIN);
            robot.moveRobot();
            robot.cruiseControlTelemetry();
            if (robot.getCloseEnough()) {
                break;
            }
        }
        robot.moveRobot(0,0,0);
        robot.launcherAutoPowershot(820);
        robot.launcherAutoPowershot(820);
        robot.launcherAutoPowershot(820);
        robot.signalBlueAlliance();
        //robot.launcherOff();
        Thread.sleep(30000);

//        if (robot.getTfodDetected() == "Quad") {
//
//            // Code for Zone C
//            robot.gyroLeft(.50,22,0);
//            robot.gyroForward(65, 1.0,22,0);
//            robot.gyroRight(.50,0,0);
//            robot.dropBlueWobbleGoal();
//            Thread.sleep(1000);
//            robot.gyroReverse(40, 0.60,0,0);
//
//        } else if (robot.getTfodDetected() == "Single") {
//
//            // Code for Zone B
//            robot.gyroLeft(.50,7,0);
//            robot.gyroForward(30, 1.0,7,0);
//            robot.gyroRight(.50,0,0);
//            robot.dropBlueWobbleGoal();
//            Thread.sleep(1000);
//            robot.gyroReverse(18, 0.60,0,0);
//
//        } else {
//
//            // Code for Default Zone A
//            robot.gyroLeft(.70,69,0);
//            robot.gyroForward(32, 1.0,69,0);
//            robot.gyroRight(.70,0,0);
//            Thread.sleep(1000);
//            robot.dropBlueWobbleGoal();
//            robot.gyroStrafeRight(2,.50,0,0);
//
//        }
//
//        Thread.sleep(1000);
//        robot.initAuxiliaryControls();

    }
}

//        while (robot.targetsAreVisible()) {
//            for (int i=0; i<100; i++){
//                robot.cruiseControl(CRUISE_CONTROL_RANGE, CRUISE_CONTROL_OFFSET, CRUISE_CONTROL_ANGLE,
//                        CRUISE_CONTROL_AXIAL_GAIN, CRUISE_CONTROL_LATERAL_GAIN, CRUISE_CONTROL_YAW_GAIN);
//                robot.moveRobot();
//                robot.cruiseControlTelemetry();
//            }
//        break;
//        }
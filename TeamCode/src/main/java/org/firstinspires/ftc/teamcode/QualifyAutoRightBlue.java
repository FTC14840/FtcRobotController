package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name ="QualifyAutoRightBlue")
//@Disabled

public class QualifyAutoRightBlue extends LinearOpMode {

    MechWarriorCode robot = new MechWarriorCode();

    final double CRUISE_CONTROL_RANGE = 1900;
    final double CRUISE_CONTROL_OFFSET = 0;
    final double CRUISE_CONTROL_ANGLE = 5;
    final double CRUISE_CONTROL_AXIAL_GAIN = 0.0030;
    final double CRUISE_CONTROL_LATERAL_GAIN = 0;
    final double CRUISE_CONTROL_YAW_GAIN = 0.0400;

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

        resetStartTime();
        robot.activateVision(this);
        //robot.tfodRunningTelemetry();
        robot.signalBlueAlliance();
        robot.prepareLauncher(820);
        robot.gyroForward(40,.80,-10.0,0);
        robot.gyroForward(40,.80,10.0,0);
        robot.raiseMagazine();
        robot.gyroLeft(.30,14,0);
        //robot.cruiseControlTelemetry();
        Thread.sleep(2000);
        //robot.cruiseControlTelemetry();

        while (robot.targetsAreVisible() && opModeIsActive()) {
            robot.cruiseControl(CRUISE_CONTROL_RANGE, CRUISE_CONTROL_OFFSET, CRUISE_CONTROL_ANGLE,
                    CRUISE_CONTROL_AXIAL_GAIN, CRUISE_CONTROL_LATERAL_GAIN, CRUISE_CONTROL_YAW_GAIN);
            robot.moveRobot();
            //robot.cruiseControlTelemetry();
            if (robot.getCloseEnough() == true){
                break;
            }
            if (time > 12){
                break;
            }
        }

        robot.moveRobot(0,0,0);
        Thread.sleep(500);
        robot.launcherAutoPowershot(820);
        Thread.sleep(500);
        robot.launcherAutoPowershot(820);
        Thread.sleep(500);
        robot.launcherAutoPowershot(820);
        robot.signalBlueAlliance();

        if (robot.getTfodDetected() == "Quad") {

            // Code for Zone C
            robot.gyroLeft(.25,18,0);
            robot.gyroForward(65,.70,18,0);
            robot.dropBlueWobbleGoal();
            Thread.sleep(500);
            //robot.gyroLeft(.25,20,0);
            robot.gyroReverse(54,.70,25,0);
            robot.gyroRight(.25,0,0);

        } else if (robot.getTfodDetected() == "Single") {

            // Code for Zone B
            robot.gyroRight(.25,-10,0);
            robot.gyroForward(32,.70,-10,0);
            robot.dropBlueWobbleGoal();
            Thread.sleep(500);
            robot.gyroLeft(.25,10,0);
            robot.gyroReverse(18,.70,10,0);
            robot.gyroRight(.25,0,0);

        } else {

            // Code for Default Zone A

            robot.gyroLeft(.25,45,0);
            robot.gyroForward(24,.70,45,0);
            robot.dropBlueWobbleGoal();
            Thread.sleep(500);
            robot.gyroLeft(.25,80,0);
            robot.gyroReverse(24,.70,80,0);
            robot.gyroRight(.25,0,0);
        }

        Thread.sleep(30000);

    }
}
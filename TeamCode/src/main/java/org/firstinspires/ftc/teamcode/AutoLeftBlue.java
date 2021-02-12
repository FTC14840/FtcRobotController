package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name ="AutoLeftBlue")
@Disabled

public class AutoLeftBlue extends LinearOpMode {

    MechWarriorCode robot = new MechWarriorCode();

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

        /** Example Movements
         // robot.gyroForward(12, .20, 0, 250);
         // robot.gyroReverse(12, .20, 0, 250);
         // robot.gyroStrafeLeft(12, .20, 0, 250);
         // robot.gyroStrafeRight(12, .20, 0, 250);
         // robot.gyroLeft(.20, 90, 250;
         // robot.gyroRight(.20, -90, 250);
         **/

        robot.tfodRunningTelemetry();
        robot.signalBlueAlliance();
        robot.prepareLauncher(840);
        robot.gyroForward(56,.80,0,0);
        robot.gyroRight(.25,-8,0);
        robot.raiseMagazine();
        robot.moveRobot(0,0,0);
        robot.launcherAutoPowershot(840);
        Thread.sleep(700);
        robot.launcherAutoPowershot(840);
        Thread.sleep(700);
        robot.launcherAutoPowershot(840);
        robot.signalBlueAlliance();

        if (robot.getTfodDetected() == "Quad") {
            robot.prepareLauncher(860);
            robot.dropBlueWobbleGoal();
            robot.lowerMagazine();
            robot.autoLowerIntake(.87);
            Thread.sleep(500);
            robot.gyroReverse(38,.80,-8,0);
            robot.gyroRight(.30,-43,0);
            robot.gyroForward(10,.70,-40,0);
            robot.autoIntakeOn();
            Thread.sleep(1000);
            robot.gyroForward(10,.10,-40,0);
            Thread.sleep(2000);
            robot.autoIntakeOff();
            robot.raiseMagazine();
            robot.gyroLeft(.30,3,0);
            robot.moveRobot(0,0,0);
            robot.launcherAutoPowershot(860);
            Thread.sleep(700);
            robot.launcherAutoPowershot(860);
            Thread.sleep(700);
            robot.launcherAutoPowershot(860);
            robot.signalBlueAlliance();



            // Code for Zone C
//            robot.gyroLeft(.25,0,0);
//            robot.gyroForward(60,.80,0,0);
//            robot.dropBlueWobbleGoal();
//            robot.autoLowerIntake();
//            Thread.sleep(1000);
//            robot.gyroForward(-100,.80,0,0);
//            robot.gyroRight(.25,-45,0);
//            robot.autoLowerIntake();
//            robot.autoIntakeOn();
//            robot.gyroForward(12,.20,-45,0);
//            robot.autoIntakeOff();
//            robot.gyroLeft(.25,0,0);
//            Thread.sleep(1000);
//            robot.launcherAutoPowershot(840);

        } else if (robot.getTfodDetected() == "Single") {

            // Code for Zone B
//            robot.gyroLeft(.50,7,0);
//            robot.gyroForward(30, 1.0,7,0);
//            robot.gyroRight(.50,0,0);
//            robot.dropBlueWobbleGoal();
//            Thread.sleep(1000);
//            robot.gyroReverse(18, 0.60,0,0);

        } else {

            // Code for Default Zone A
            robot.gyroForward(10, 1.0,-9,0);
            robot.dropBlueWobbleGoal();
            Thread.sleep(500);
            robot.gyroLeft(.70,90,0);
            robot.gyroForward(-30,1.0,70,0);
            robot.gyroRight(.70,0,0);
        }

        robot.initAuxiliaryControls();
        Thread.sleep(30000);

    }
}
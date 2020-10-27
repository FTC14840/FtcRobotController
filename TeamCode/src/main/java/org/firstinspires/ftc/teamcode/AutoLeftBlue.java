package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoLeftBlue")

public class AutoLeftBlue extends LinearOpMode {

    MechWarriorCode robot = new MechWarriorCode();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initHardware(this);
        robot.initTfod(this);
        robot.calibrateGyro(this);

        while (!isStarted()) {
            robot.tfodTelemetry();
        }

        waitForStart();

        robot.stopTfod(this);

        /** Example Movements
         // robot.gyroForward(12, .20, 0, 5000);
         // robot.gyroReverse(12, .20, 0, 5000);
         // robot.gyroStrafeLeft(12, .20, 0, 5000);
         // robot.gyroStrafeRight(12, .20, 0, 5000);
         // robot.gyroLeft(.20, -90, 5000);
         // robot.gyroRight(.20, 90, 5000);
         **/

        if (robot.getTfodDetected() == "Quad") {

            telemetry.log().clear();
            telemetry.addData("Running", "Quad Program");
            telemetry.update();

            robot.gyroRight(.20, 90, 0);

        } else if (robot.getTfodDetected() == "Single") {

            telemetry.log().clear();
            telemetry.addData("Running", "Single Program");
            telemetry.update();

            robot.gyroLeft(.20, -90, 5000);

        } else {

            telemetry.log().clear();
            telemetry.addData("Running", "Default Program");
            telemetry.update();

            robot.gyroForward(12, .20, 0, 0);

        }
    }
}





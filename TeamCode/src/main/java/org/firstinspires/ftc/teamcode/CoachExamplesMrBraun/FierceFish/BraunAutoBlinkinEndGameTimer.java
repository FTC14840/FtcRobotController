package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.FierceFish;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BraunAutoBlinkinEndGameTimer")

@Disabled

public class BraunAutoBlinkinEndGameTimer extends LinearOpMode {

    RevBlinkinLedDriver lights;
    int temp;

    @Override
    public void runOpMode() throws InterruptedException {

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        waitForStart();

        if (temp ==1){
            resetStartTime();
            temp = 2;
        }

        if(time >= 90 && time < 115){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
        } else if(time >= 115 && time < 120) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
    }
}

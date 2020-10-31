package org.firstinspires.ftc.teamcode.TeamLeadJonathan;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;


@TeleOp(name="Jonathan Launcher Test")

@Disabled

public class JonathanLauncherTest extends LinearOpMode {

    JonathanMethods launcher = new JonathanMethods();


    public void runOpMode() throws InterruptedException{
        telemetry.addData("Revving Up", "Prepare to Fire");
        telemetry.update();
        //Init specific launcher hardware for less confusion
        launcher.initLauncherHardwareOnly();


        waitForStart();
        //Sets motors to full power
        launcher.startLauncher();

        while (opModeIsActive()) {

            if (gamepad1.dpad_down){
                launcher.launcherSpeedDown();
            }
            if (gamepad1.dpad_up) {
               launcher.launcherSpeedUp();
            }
            if (gamepad1.dpad_left) {
                launcher.launcherDownButLikeMore();
            }
            if (gamepad1.dpad_right) {
                launcher.launcherUpButLikeMore();
            }

            launcher.launcherTelemetry(this);
        }
        

    }
}


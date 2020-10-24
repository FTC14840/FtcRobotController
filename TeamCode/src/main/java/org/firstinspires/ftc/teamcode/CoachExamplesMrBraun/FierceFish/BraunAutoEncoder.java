package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.FierceFish;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "BraunAutoEncoder")

@Disabled

public class BraunAutoEncoder extends LinearOpMode {

    DcMotor leftDriveMotor;
    DcMotor rightDriveMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        leftDriveMotor = hardwareMap.dcMotor.get("testMotor");
        rightDriveMotor = hardwareMap.dcMotor.get("testMotor");

        leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        resetEncoders();

        waitForStart();

        driveForward(1.0, 1000);
        runToPosition(1000, 1.0);

    }

    void runToPosition(int position, double power) {
        leftDriveMotor.setTargetPosition(position);
        rightDriveMotor.setTargetPosition(position);

        leftDriveMotor.setPower(power);
        rightDriveMotor.setPower(power);

        while(leftDriveMotor.isBusy() && rightDriveMotor.isBusy()){

        }
    }

    void driveForward (double power, double ticks){
        while(leftDriveMotor.getCurrentPosition() < ticks && rightDriveMotor.getCurrentPosition() < ticks){
            leftDriveMotor.setPower(power);
            rightDriveMotor.setPower(power);
        }
        resetEncoders();
    }

    void resetEncoders(){

        leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Might not need this?
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}

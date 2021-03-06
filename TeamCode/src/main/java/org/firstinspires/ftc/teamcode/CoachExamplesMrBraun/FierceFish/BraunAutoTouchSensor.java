package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.FierceFish;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "BraunAutoTouchSensor")

@Disabled

public class BraunAutoTouchSensor extends LinearOpMode {

    DcMotor leftDriveMotor;
    DcMotor rightDriveMotor;
    BNO055IMU imu;
    Orientation angles;

    RevBlinkinLedDriver lights;

    ColorSensor colors;

    TouchSensor frontTouch;

    @Override
    public void runOpMode() throws InterruptedException {

        leftDriveMotor = hardwareMap.dcMotor.get("leftDriveMotor");
        rightDriveMotor = hardwareMap.dcMotor.get("rightDriveMotor");

        leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        resetEncoders();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        colors = hardwareMap.get(ColorSensor.class, "colors");

        frontTouch = hardwareMap.get(TouchSensor.class, "frontTouch");

        waitForStart();

        if(frontTouch.isPressed()){
            // Do this code
        }

        if (frontTouch.getValue() == 1){
            // once it reaches this threashold, do this code
        }

        while (opModeIsActive()){
            if (frontTouch.isPressed()){
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }
        }

    }

    void driveForward (double power, double targetAngle) throws InterruptedException {
        double leftPower;
        double rightPower;
        leftDriveMotor.setPower(power);
        rightDriveMotor.setPower(power);
        while(!frontTouch.isPressed() && opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if(angles.firstAngle < targetAngle){
                leftDriveMotor.setPower(power + .05);  // Instead, look into using PID for the .05 value.
                rightDriveMotor.setPower(power - .05);
            } else if(angles.firstAngle > targetAngle) {
                leftDriveMotor.setPower(power - .05);
                rightDriveMotor.setPower(power +.05);
            } else {
                leftDriveMotor.setPower(power);
                rightDriveMotor.setPower(power);
            }
        }
        leftDriveMotor.setPower(0);
        rightDriveMotor.setPower(0);
        resetEncoders();
    }

    void turnleft (double turnAngle, double timeouts){
        sleep(250);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed = 0.5;
        double oldDegreesLeft = turnAngle;
        double scaleSpeed = speed;
        double targetHeading = angles.firstAngle + turnAngle;
        double oldAngle = angles.firstAngle;
        if(targetHeading < -180){
            targetHeading += 360;
        }
        if (targetHeading > 180) {
            targetHeading -= 360;
        }
        double degreesLeft = ((Math.signum(angles.firstAngle - targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading)) +
                (Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        resetStartTime();
        while(opModeIsActive() && time < timeouts && degreesLeft > 1 && oldDegreesLeft-degreesLeft >= 0){
            scaleSpeed = degreesLeft / (100 + degreesLeft) * speed;
            if(scaleSpeed > 1){
                scaleSpeed = .1;
            }
            leftDriveMotor.setPower(scaleSpeed);
            rightDriveMotor.setPower(-scaleSpeed);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            oldDegreesLeft = degreesLeft;
            degreesLeft = ((Math.signum(angles.firstAngle - targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading)) +
                    (Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
            if(Math.abs(angles.firstAngle-oldAngle) < 1) {
                speed *= 1.1;
            }
            oldAngle = angles.firstAngle;
        }
        leftDriveMotor.setPower(0);
        rightDriveMotor.setPower(0);
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

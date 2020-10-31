// Package name
package org.firstinspires.ftc.teamcode.TeamLeadJonathan;

// Imports

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

// Begin hardware class
public class JonathanMethods {

    // Define OpMode for bot hardware
    private LinearOpMode botOpMode;

    // Define DcMotors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor backLauncher;
    private DcMotor frontLauncher;

    // Define global variables/fields for three axis motion
    private double driveAxial = 0;  // Positive is forward
    private double driveLateral = 0;  // Positive is right
    private double driveYaw = 0;  // Positive is Counterclockwise

    // Define global variables/fields for driver control
    private double speed = 1;
    private double direction = 1;
    private boolean aButtonPad1 = false;
    private boolean bButtonPad1 = false;

    // Constants for driver control
    private static final double HIGHSPEED = 1;
    private static final double LOWSPEED = .75;
    private static final double TURNSENSITIVITY = 1.5;
    private static final double INCREMENT1 = 0.001;
    private static final double INCREMENT2 = 0.005;

    // Tick to inches conversion
    private static final double TICKS = 1440; // goBulda = 537.6, AndyMark = 1120, Tetrix = 1440
    private static final double GEARREDUCTION = 1.0; // Greater than 1.0; Less than 1.0 if geared up
    private static final double WHEELDIAMETERINCHES = 4.0;
    private static final double TICKSTOINCHES = (TICKS * GEARREDUCTION) / (Math.PI * WHEELDIAMETERINCHES);

    // Vuforia fields
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    WebcamName webcamName = null;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private TFObjectDetector tfod = null;
    private VuforiaTrackables targets;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final String VUFORIA_KEY = "ATqulq//////AAABmfYPXE+z1EORrVmv4Ppo3CcPktGk5mvdMnvPi9/T3DMYGc2mju8KUyG9gAB7pKlb9k9SZnM0YSq1JUZ6trE1ZKmMU8z5QPuhA/b6/Enb+XVGwmjrRjhMfNtUNgiZDhtsUvxr9fQP4HVjTzlz4pv0z3MeWZmkAgIN8T8YM0EFWrW4ODqYQmZjB0Nri2KKVM9dlOZ5udPfTZ9YvMgrCyxxG7O8P84AvwCAyXxzxelL4OfGnbygs0V60CQHx51gqrki613PT/9D1Q1io5+UbN6xAQ26AdYOTmADgJUGlfC2eMyqls4qAIoOj+pcJbm5ryF5yW9pEGHmvor1c9HlCFwhKxiaxw+cTu8AEaAdNuR65i/p";
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;
    private boolean targetVisible = false;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private String tfodDetected = "None";

    public String getTfodDetected() {
        return tfodDetected;
    }

    private static final int MAX_TARGETS = 5;
    private static final double ON_AXIS = 10;
    private static final double CLOSE_ENOUGH = 20;
    public static final double YAW_GAIN = 0.0180;   // Rate at which we respond to heading error
    public static final double LATERAL_GAIN = 0.0027;  // Rate at which we respond to off-axis error
    public static final double AXIAL_GAIN = 0.0017;  // Rate at which we respond to target distance errors
    private boolean targetFound;    // set to true if Vuforia is currently tracking a target
    private String targetName;     // Name of the currently tracked target
    private double robotX;         // X displacement from target center
    private double robotY;         // Y displacement from target center
    private double robotBearing;   // Robot's rotation around the Z axis (CCW is positive)
    private double targetRange;    // Range from robot's center to target in mm
    private double targetBearing;  // Heading of the target , relative to the robot's unrotated center
    private double relativeBearing;// Heading to the target from the robot's current bearing.

    // Gyro fields
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;

    /* Constructor for Cruise Control */
    public JonathanMethods() {
        targetFound = false;
        targetName = null;
        targets = null;
        robotX = 0;
        robotY = 0;
        targetRange = 0;
        targetBearing = 0;
        robotBearing = 0;
        relativeBearing = 0;
    }

    /**
     * Hardware Methods
     **/
    public void initLauncherHardwareOnly(){
        backLauncher = botOpMode.hardwareMap.get(DcMotor.class, "backLauncher");
        frontLauncher = botOpMode.hardwareMap.get(DcMotor.class, "frontLauncher");
        backLauncher.setDirection(REVERSE);
        frontLauncher.setDirection(REVERSE);
        backLauncher.setMode(STOP_AND_RESET_ENCODER);
        frontLauncher.setMode(STOP_AND_RESET_ENCODER);
        backLauncher.setMode(RUN_USING_ENCODER);
        frontLauncher.setMode(RUN_USING_ENCODER);
        backLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void initHardware(LinearOpMode opMode) throws InterruptedException {

        // Set opMode to one defined above
        botOpMode = opMode;

        // Remind the driver to keep the bot still during the hardware init... Specifically for the gyro.
        botOpMode.telemetry.log().clear();
        botOpMode.telemetry.log().add("Robot Initializing. Do Not Move The Bot...");
        botOpMode.telemetry.update();

        // Map global variables/ fields to config file on Robot Controller
        frontLeft = botOpMode.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = botOpMode.hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = botOpMode.hardwareMap.get(DcMotor.class, "backLeft");
        backRight = botOpMode.hardwareMap.get(DcMotor.class, "backRight");
        backLauncher = botOpMode.hardwareMap.get(DcMotor.class, "backLauncher");
        frontLauncher = botOpMode.hardwareMap.get(DcMotor.class, "frontLauncher");

        // Reverse motors if they don't drive forward
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLauncher.setDirection(REVERSE);
        frontLauncher.setDirection(REVERSE);
        backLauncher.setMode(STOP_AND_RESET_ENCODER);
        frontLauncher.setMode(STOP_AND_RESET_ENCODER);
        backLauncher.setMode(RUN_USING_ENCODER);
        frontLauncher.setMode(RUN_USING_ENCODER);
        backLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reset encoders
        stopAndResetEncoder();
        runUsingEncoder();

        // Set all motor power to zero
        moveRobot(0, 0, 0);
    }

    public void initTfod(LinearOpMode opMode) throws InterruptedException {

        botOpMode = opMode;
        int cameraMonitorViewId = botOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", botOpMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = botOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        int tfodMonitorViewId = botOpMode.hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", botOpMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        tfod.setZoom(2.5, 1.78);
        if (tfod != null) {
            tfod.activate();
        }
    }

    public void deactivateTfod() {
        tfod.deactivate();
    }

    public void stopTfod(LinearOpMode opMode) throws InterruptedException {

        // Set opMode to one defined above
        botOpMode = opMode;

        tfod.deactivate();
    }

    // This method is from the cruise control example and sets the targets relative to the bot.
    public void initVisionTracking(LinearOpMode opMode) throws InterruptedException {

        botOpMode = opMode;
        webcamName = botOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = botOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", botOpMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        targets = vuforia.loadTrackablesFromAsset("UltimateGoal");
        targets.get(0).setName("Blue Tower Goal Target");
        targets.get(1).setName("Red Tower Goal Target");
        targets.get(2).setName("Red Alliance Target");
        targets.get(3).setName("Blue Alliance Target");
        targets.get(4).setName("Front Wall Target");

//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
//        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
//        blueTowerGoalTarget.setName("Blue Tower Goal Target");
//        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
//        redTowerGoalTarget.setName("Red Tower Goal Target");
//        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
//        redAllianceTarget.setName("Red Alliance Target");
//        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
//        blueAllianceTarget.setName("Blue Alliance Target");
//        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
//        frontWallTarget.setName("Front Wall Target");
//        allTrackables.addAll(targetsUltimateGoal);

        /**
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         * Before being transformed, each target image is conceptually located at the origin of the field's
         * coordinate system (the center of the field), facing up.
         */

//        redAllianceTarget.setLocation(OpenGLMatrix
//                .translation(0, -halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
//        blueAllianceTarget.setLocation(OpenGLMatrix
//                .translation(0, halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
//        frontWallTarget.setLocation(OpenGLMatrix
//                .translation(-halfField, 0, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
//        blueTowerGoalTarget.setLocation(OpenGLMatrix
//                .translation(halfField, quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
//        redTowerGoalTarget.setLocation(OpenGLMatrix
//                .translation(halfField, -quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        allTrackables.addAll(targets);
        OpenGLMatrix targetOrientation = OpenGLMatrix
                .translation(0, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, -90));

        // Translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 9.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_LEFT_DISPLACEMENT = -.50f;     // eg: Camera is ON the robot's center line
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.25f * mmPerInch;   // eg: Camera is 8 Inches above ground

        /**
         Info:  The coordinate frame for the robot looks the same as the field.
         The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         pointing to the LEFT side of the Robot.
         We need to rotate the camera around it's long axis to bring the correct camera forward.
         **/

        final float PHONE_X_ROTATE = 90;
        final float PHONE_Y_ROTATE = -90;
        final float PHONE_Z_ROTATE = 0;

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, PHONE_X_ROTATE, PHONE_Y_ROTATE, PHONE_Z_ROTATE));

//        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // Camera is 110 mm in front of robot center
//        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // Camera is 200 mm above ground
//        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // Camera is ON the robots center line
//        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
//                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.YZX,
//                        AngleUnit.DEGREES, CAMERA_CHOICE == VuforiaLocalizer.CameraDirection.BACK ? 90 : -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables) {
            trackable.setLocation(targetOrientation);
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }
        targets.activate();
    }

    public void activateCruiseControl() {
        if (targets != null)
            targets.activate();

        botOpMode.telemetry.log().clear();
        botOpMode.telemetry.addData("Robot Initialized", "Press Play to Begin");
        botOpMode.telemetry.update();
    }

    public boolean targetsAreVisible() {

        // This method cycles through the targets until it finds one.
        int targetTestID = 0;
        while ((targetTestID < MAX_TARGETS) && !targetIsVisible(targetTestID)) {
            targetTestID++;
        }
        return (targetFound);
    }

    public boolean targetIsVisible(int targetId) {

        VuforiaTrackable target = targets.get(targetId);
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) target.getListener();
        OpenGLMatrix location = null;

        // If we have a target, look for an updated robot position
        if ((target != null) && (listener != null) && listener.isVisible()) {
            targetFound = true;
            targetName = target.getName();

            // If we have an updated robot location, update all the relevant tracking information
            location = listener.getUpdatedRobotLocation();
            if (location != null) {

                // Create a translation and rotation vector for the robot.
                VectorF trans = location.getTranslation();
                Orientation rot = Orientation.getOrientation(location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Robot position is defined by the standard Matrix translation (x and y)
                robotX = trans.get(0);
                robotY = trans.get(1);

                // Robot bearing (in +vc CCW cartesian system) is defined by the standard Matrix z rotation
                robotBearing = rot.thirdAngle;

                // target range is based on distance from robot position to origin.
                targetRange = Math.hypot(robotX, robotY);

                // target bearing is based on angle formed between the X axis to the target range line
                targetBearing = Math.toDegrees(-Math.asin(robotY / targetRange));

                // Target relative bearing is the target Heading relative to the direction the robot is pointing.
                relativeBearing = targetBearing - robotBearing + 90;
            }
            targetFound = true;
        } else {
            // Indicate that there is no target visible
            targetFound = false;
            targetName = "None";
        }

        return targetFound;
    }

    public void calibrateGyro(LinearOpMode opMode) throws InterruptedException {

        // Set opMode to one defined above
        botOpMode = opMode;

        navxMicro = botOpMode.hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope) navxMicro;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (navxMicro.isCalibrating()) {
            botOpMode.telemetry.log().clear();
            botOpMode.telemetry.addData("Calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            botOpMode.telemetry.update();
        }
    }

    /**
     * TeleOp Methods
     **/

    public boolean cruiseControl(double standOffDistance) {
        boolean closeEnough;
        double Y = (relativeBearing * YAW_GAIN);
        double L = (robotY * LATERAL_GAIN);
        double A = (-(robotX + standOffDistance) * AXIAL_GAIN);
        setYaw(Y);
        setAxial(A);
        setLateral(L);
        closeEnough = ((Math.abs(robotX + standOffDistance) < CLOSE_ENOUGH) && (Math.abs(robotY) < ON_AXIS));
        return (closeEnough);
    }

    public void manualDrive() {
        // Setting three motions to stick movements
        setAxial(-botOpMode.gamepad1.left_stick_y);
        setLateral(botOpMode.gamepad1.left_stick_x);
        setYaw(-botOpMode.gamepad1.right_stick_x);

        // Logic for speed control on button A
        if (botOpMode.gamepad1.a) {
            aButtonPad1 = true;
        } else if (aButtonPad1) {
            aButtonPad1 = false;
            if (speed == LOWSPEED) {
                speed = HIGHSPEED;
            } else {
                speed = LOWSPEED;
            }
        }

        // Logic for direction control on button B
        if (botOpMode.gamepad1.b) {
            bButtonPad1 = true;
        } else if (bButtonPad1) {
            bButtonPad1 = false;
            direction = -direction;
        }
    }

    // Clipping motor power for axial motion
    public void setAxial(double axial) {
        driveAxial = Range.clip(axial, -1, 1);
    }

    // Clipping motor power for lateral motion
    public void setLateral(double lateral) {
        driveLateral = Range.clip(lateral, -1, 1);
    }

    // Clipping motor power for yaw motion
    public void setYaw(double yaw) {
        driveYaw = Range.clip(yaw, -1, 1);
    }

    // Calculations for the power to each motor
    public void moveRobot() {
        // Vector addition and algebra for the pwer to each motor
        double moveFrontLeft = speed * (direction * ((-driveAxial) + (driveLateral)) + (driveYaw * TURNSENSITIVITY));
        double moveFrontRight = speed * (direction * ((-driveAxial - driveLateral)) - (driveYaw * TURNSENSITIVITY));
        double moveBackLeft = speed * (direction * ((-driveAxial - driveLateral)) + (driveYaw * TURNSENSITIVITY));
        double moveBackRight = speed * (direction * ((-driveAxial + driveLateral)) - (driveYaw * TURNSENSITIVITY));

        // Normalize all motor speeds so no value exceeds 100% power.
        double max = Math.max(Math.abs(moveFrontLeft), Math.abs(moveFrontRight));
        max = Math.max(max, Math.abs(moveBackLeft));
        max = Math.max(max, Math.abs(moveBackRight));

        // Proportional logic to reduce all motors to the highest value
        if (max > 1.0) {
            moveFrontLeft /= max;
            moveFrontRight /= max;
            moveBackLeft /= max;
            moveBackRight /= max;
        }

        // Set drive motor power based on calculations
        frontLeft.setPower(moveFrontLeft);
        frontRight.setPower(moveFrontRight);
        backLeft.setPower(moveBackLeft);
        backRight.setPower(moveBackRight);
    }

    // Overloaded Method to set all three motions and move the robot
    public void moveRobot(double axial, double lateral, double yaw) {
        setAxial(axial);
        setLateral(lateral);
        setYaw(yaw);
        moveRobot();
    }

    /**
     * Autonomous Methods
     **/

    public void gyroForward(int distance, double power, double angle, int pause) throws InterruptedException {
        if (botOpMode.opModeIsActive()) {
            stopAndResetEncoder();
            runUsingEncoder();
            driveByInches(distance);
            runToPosition();
            while (botOpMode.opModeIsActive()) {
                double gyroHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double errorMultiple = 1.0;
                double error = (errorMultiple * (gyroHeading - angle) / 100);
                if (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                    frontLeft.setPower(power + error);
                    frontRight.setPower(power - error);
                    backLeft.setPower(power + error);
                    backRight.setPower(power - error);
                } else {
                    stopDriving();
                    Thread.sleep(pause);
                    break;
                }
            }
        }
    }

    public void driveByInches(int distance) {
        frontLeft.setTargetPosition(-distance * (int) TICKSTOINCHES);
        frontRight.setTargetPosition(-distance * (int) TICKSTOINCHES);
        backLeft.setTargetPosition(-distance * (int) TICKSTOINCHES);
        backRight.setTargetPosition(-distance * (int) TICKSTOINCHES);
    }

    public void gyroReverse(int distance, double power, double angle, int pause) throws InterruptedException {
        if (botOpMode.opModeIsActive()) {
            stopAndResetEncoder();
            runUsingEncoder();
            driveByInches(-distance);
            runToPosition();
            while (botOpMode.opModeIsActive()) {
                double gyroHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double errorMultiple = 1.0;
                double error = (errorMultiple * (gyroHeading - angle) / 100);
                if (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                    frontLeft.setPower(power - error);
                    frontRight.setPower(power + error);
                    backLeft.setPower(power - error);
                    backRight.setPower(power + error);
                } else {
                    stopDriving();
                    Thread.sleep(pause);
                    break;
                }
            }
        }
    }

    public void gyroStrafeLeft(int distance, double power, double angle, int pause) throws InterruptedException {
        if (botOpMode.opModeIsActive()) {
            stopAndResetEncoder();
            runUsingEncoder();
            strafeByInches(-distance);
            runToPosition();
            while (botOpMode.opModeIsActive()) {
                double gyroHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double errorMultiple = 1.0;
                double error = (errorMultiple * (gyroHeading - angle) / 100);
                if (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                    frontLeft.setPower(power + error);
                    frontRight.setPower(power + error);
                    backLeft.setPower(power - error);
                    backRight.setPower(power - error);
                } else {
                    stopDriving();
                    Thread.sleep(pause);
                    break;
                }
            }
        }
    }

    public void strafeByInches(int distance) {
        frontLeft.setTargetPosition(distance * (int) TICKSTOINCHES);
        frontRight.setTargetPosition(-distance * (int) TICKSTOINCHES);
        backLeft.setTargetPosition(-distance * (int) TICKSTOINCHES);
        backRight.setTargetPosition(distance * (int) TICKSTOINCHES);
    }

    public void gyroStrafeRight(int distance, double power, double angle, int pause) throws InterruptedException {
        if (botOpMode.opModeIsActive()) {
            stopAndResetEncoder();
            runUsingEncoder();
            strafeByInches(distance);
            runToPosition();
            while (botOpMode.opModeIsActive()) {
                double gyroHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double errorMultiple = 1.0;
                double error = (errorMultiple * (gyroHeading - angle) / 100);
                if (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                    frontLeft.setPower(power - error);
                    frontRight.setPower(power - error);
                    backLeft.setPower(power + error);
                    backRight.setPower(power + error);
                } else {
                    stopDriving();
                    Thread.sleep(pause);
                    break;
                }
            }
        }
    }

    public void gyroLeft(double power, double angle, int pause) throws InterruptedException {
        if (botOpMode.opModeIsActive()) {
            stopAndResetEncoder();
            runUsingEncoder();
            while (botOpMode.opModeIsActive()) {
                double gyroHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double invertGyroHeading = gyroHeading * -1;
                if (invertGyroHeading > angle) {
                    frontLeft.setPower(power);
                    frontRight.setPower(-power);
                    backLeft.setPower(power);
                    backRight.setPower(-power);
                } else {
                    stopDriving();
                    Thread.sleep(pause);
                    break;
                }
            }
        }
    }

    public void gyroRight(double power, double angle, int pause) throws InterruptedException {
        if (botOpMode.opModeIsActive()) {
            stopAndResetEncoder();
            runUsingEncoder();
            while (botOpMode.opModeIsActive()) {
                double gyroHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double invertGyroHeading = gyroHeading * -1;
                if (invertGyroHeading < angle) {
                    frontLeft.setPower(-power);
                    frontRight.setPower(power);
                    backLeft.setPower(-power);
                    backRight.setPower(power);
                } else {
                    stopDriving();
                    Thread.sleep(pause);
                    break;
                }
            }
        }
    }

    public void stopDriving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void stopAndResetEncoder() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoder() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runWithoutEncoder() {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Telemetry Methods
     **/

    public void driveTelemetry(LinearOpMode opMode) throws InterruptedException {

        // Set opMode to one defined above
        botOpMode = opMode;

        if (botOpMode.gamepad1.y) {
            stopAndResetEncoder();
            runUsingEncoder();
        }

        double frontLeftInches = frontLeft.getCurrentPosition() / TICKSTOINCHES;
        double frontRightInches = frontRight.getCurrentPosition() / TICKSTOINCHES;
        double backLeftInches = backLeft.getCurrentPosition() / TICKSTOINCHES;
        double backRightInches = backRight.getCurrentPosition() / TICKSTOINCHES;
        double gyroHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        botOpMode.telemetry.log().clear();
        botOpMode.telemetry.addData("Note", "--> Tap Y to reset encoders");
        botOpMode.telemetry.addData("Heading", "%.2f", gyroHeading);
        botOpMode.telemetry.addData("Inches", "FL: %2d, FR: %2d, BL: %2d, BR: %2d", -(int) frontLeftInches, -(int) frontRightInches, -(int) backLeftInches, -(int) backRightInches);
        botOpMode.telemetry.addData("Encoder", "FL: %2d,  FR: %2d, BL: %2d, BR: %2d", -frontLeft.getCurrentPosition(), -frontRight.getCurrentPosition(), -backLeft.getCurrentPosition(), -backRight.getCurrentPosition());
        botOpMode.telemetry.addData("Target", "FL: %2d, FR: %2d, BL: %2d, BR: %2d", -frontLeft.getTargetPosition(), -frontRight.getTargetPosition(), -backLeft.getTargetPosition(), -backRight.getTargetPosition());
        botOpMode.telemetry.addData("Power", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f", frontLeft.getPower(), frontRight.getPower(), backLeft.getPower(), backRight.getPower());
        botOpMode.telemetry.addData("Axes", "A: %.2f, L: %.2f, Y: %.2f", driveAxial, driveLateral, driveYaw);
        botOpMode.telemetry.update();
    }

    public void launcherTelemetry(LinearOpMode opMode) {
        botOpMode.telemetry.log().clear();
        botOpMode.telemetry.addData("FrontPower", "Launcher: %.2f,", frontLauncher.getPower());
        botOpMode.telemetry.addData("BackPower", "Launcher: %.2f,", backLauncher.getPower());
        botOpMode.telemetry.update();
    }


    public void tfodTelemetry(LinearOpMode opMode) throws InterruptedException {

        // Set opMode to one defined above
        botOpMode = opMode;

        if (botOpMode.gamepad1.y) {
            botOpMode.telemetry.log().clear();
            botOpMode.telemetry.addData("Note", "Resetting TFOD");
            botOpMode.telemetry.update();
            tfod.deactivate();
            tfod.activate();
            Thread.sleep(1000);
        }

        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        if (updatedRecognitions != null) {
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                tfodDetected = recognition.getLabel();
                botOpMode.telemetry.log().clear();
                botOpMode.telemetry.addData("Robot Initialized", "Press Play to Begin");
                botOpMode.telemetry.addData("Note", "--> Tap Y to Reset TFOD");
                botOpMode.telemetry.addData(String.format("Detected (%d)", i), recognition.getLabel());
                botOpMode.telemetry.update();
                Thread.sleep(1000);
            }
        } else {
            tfodDetected = "None";
            botOpMode.telemetry.log().clear();
            botOpMode.telemetry.addData("Robot Initialized", "Press Play to Begin");
            botOpMode.telemetry.addData("Note", "--> Tap Y to Reset TFOD");
            botOpMode.telemetry.addData("Detected", "None");
            botOpMode.telemetry.update();
        }
    }

    public void cruiseControlTelemetry() {
        if (targetFound) {
            // Display the current visible target name, robot info, target info, and required robot action.
            botOpMode.telemetry.addData("Visible", targetName);
            botOpMode.telemetry.addData("Robot", "[X]:[Y] (B) [%5.0fmm]:[%5.0fmm] (%4.0f°)",
                    robotX, robotY, robotBearing);
            botOpMode.telemetry.addData("Target", "[R] (B):(RB) [%5.0fmm] (%4.0f°):(%4.0f°)",
                    targetRange, targetBearing, relativeBearing);
            botOpMode.telemetry.addData("- Turn    ", "%s %4.0f°", relativeBearing < 0 ? ">>> CW " : "<<< CCW", Math.abs(relativeBearing));
            botOpMode.telemetry.addData("- Strafe  ", "%s %5.0fmm", robotY < 0 ? "LEFT" : "RIGHT", Math.abs(robotY));
            botOpMode.telemetry.addData("- Distance", "%5.0fmm", Math.abs(robotX));
        } else {
            botOpMode.telemetry.addData("Visible", "- - - -");
        }
    }
    //I know everything is out of order I'll fix it later
    public void launcherSpeedUp() {
        frontLauncher.setPower(Range.clip(frontLauncher.getPower() + INCREMENT1, 0, 1));
        backLauncher.setPower(Range.clip(backLauncher.getPower() + INCREMENT1, 0, 1));
    }

    public void launcherSpeedDown() {
        frontLauncher.setPower(Range.clip(frontLauncher.getPower() - INCREMENT1, 0, 1));
        backLauncher.setPower(Range.clip(backLauncher.getPower() - INCREMENT1, 0, 1));
    }

    public void launcherUpButLikeMore() {
        frontLauncher.setPower(Range.clip(frontLauncher.getPower() + INCREMENT2, 0, 1));
        backLauncher.setPower(Range.clip(backLauncher.getPower() + INCREMENT2, 0, 1));
    }

    public void launcherDownButLikeMore() {
        frontLauncher.setPower(Range.clip(frontLauncher.getPower() - INCREMENT2, 0, 1));
        backLauncher.setPower(Range.clip(backLauncher.getPower() - INCREMENT2, 0, 1));

    }

    public void startLauncher(){
        backLauncher.setPower(1.0);
        frontLauncher.setPower(1.0);
    }
}
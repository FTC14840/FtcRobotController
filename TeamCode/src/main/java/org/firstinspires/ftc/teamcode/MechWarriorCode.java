// Package name
package org.firstinspires.ftc.teamcode;

// Imports
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
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
import org.firstinspires.ftc.robotcore.external.tfod.TfodBase;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;

import java.util.ArrayList;
import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

// Begin utility class
public class MechWarriorCode {

    // Define OpMode for bot hardware
    private LinearOpMode botOpMode;

    // Right Control Hub
    private DcMotor frontRight;      // 0
    private DcMotor backRight;       // 1
    private DcMotor magazineMotor;   // 2
    private DcMotorEx launcher;      // 3
    private Servo redWobbleGoal;     // 0
    private Servo redCam;            // 1
    private Servo ringServo;         // 2
    private Servo intakeServo;       // 3
    // ledLights on 5
    // imu on IC2 Bus 0

    // Left Expansion Hub
    private DcMotorEx intakeMotor;     // 0
    private DcMotor frontLeft;       // 2
    private DcMotor backLeft;        // 3
    private Servo blueWobbleGoal;    // 0
    private Servo blueCam;           // 1

    private double launcherVelocity = 820; //Starting Velocity in Increments of 20
    private double launcherHighGoalVelocity = 820;
    private double launcherPowershotVelocity = 780;
    private double launcherVelocityIncrement = 20;

    double kP = 800.0;
    double kI = 80.0;
    double kD = 8.0;
    double F = 15.0;
    int shotSpeed = 600;

    private double intakeVelocity = 800;
    private int magazineTargetPosition = 200;
    private double ringServoOpen = .05;
    private double ringServoClose = .44;

    // Define global variables/fields for three axis motion
    private double driveAxial = 0;  // Positive is forward
    private double driveLateral = 0;  // Positive is right
    private double driveYaw = 0;  // Positive is Counterclockwise

    // Define global variables/fields for driver control
    private double speed = 1.0;
    private int direction = 1;
    private boolean aButtonPad1 = false;
    private boolean bButtonPad1 = false;

    // Constants for driver control
    private static final double HIGHSPEED = 1.0;
    private static final double LOWSPEED = 0.75;
    private static final double TURNSENSITIVITY = 2;
    private static final double STRAFESENSITIVITY = 2;

    // Tick to inches conversion
    private static final double TICKS = 537.6; // goBulda = 537.6, AndyMark = 1120, Tetrix = 1440
    private static final double GEARREDUCTION = 1.0; // Greater than 1.0; Less than 1.0 if geared up
    private static final double WHEELDIAMETERINCHES = 4.0;
    private static final double TICKSTOINCHES = (TICKS * GEARREDUCTION) / (Math.PI * WHEELDIAMETERINCHES);

    // Vuforia fields
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    WebcamName webcamName = null;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private TFObjectDetector tfod = null;
    private TfodCurrentGame tfodCurrentGame = null;
    //private TfodBase tfodbase = null;
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
    private String tfodDetected = "Zero";

    public String getTfodDetected() {
        return tfodDetected;
    }


    private boolean closeEnough = false;
    private static final int MAX_TARGETS = 5;
    private static final double ON_AXIS = 10;
    private static final double CLOSE_ENOUGH = 30;
    private boolean targetFound;    // set to true if Vuforia is currently tracking a target
    private String targetName;     // Name of the currently tracked target
    private double robotX;         // X displacement from target center
    private double robotY;         // Y displacement from target center
    private double robotBearing;   // Robot's rotation around the Z axis (CCW is positive)
    private double targetRange;    // Range from robot's center to target in mm
    private double targetBearing;  // Heading of the target , relative to the robot's unrotated center
    private double relativeBearing;// Heading to the target from the robot's current bearing.

    public boolean getCloseEnough() {
        return closeEnough;
    }

    // Gyro fields
    BNO055IMU imu;

    RevBlinkinLedDriver ledLights;
    int blinkinTimer = 0;

    /* Constructor for Cruise Control */
    public MechWarriorCode() {
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

    public void initHardware(LinearOpMode opMode) throws InterruptedException {

        botOpMode = opMode;

        // Remind the driver to keep the bot still during the hardware init... Specifically for the gyro.
        botOpMode.telemetry.log().clear();
        botOpMode.telemetry.log().add("Robot Initializing. Do Not Move The Bot...");
        botOpMode.telemetry.update();

        ledLights = botOpMode.hardwareMap.get(RevBlinkinLedDriver.class,"ledLights");
        ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);

        // Map global variables/ fields to config file on Robot Controller
        frontLeft = botOpMode.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = botOpMode.hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = botOpMode.hardwareMap.get(DcMotor.class, "backLeft");
        backRight = botOpMode.hardwareMap.get(DcMotor.class, "backRight");

        // Reverse motors if they don't drive forward
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        driveBreak();
        stopAndResetEncoder();
        runWithoutEncoder();

        // Set all motor power to zero
        moveRobot(0, 0, 0);

        launcher = botOpMode.hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        launcher.setVelocityPIDFCoefficients(kP,kI,kD,F);
        launcher.setPositionPIDFCoefficients(5.0);
        launcher.setPower(0.0);

        intakeMotor = botOpMode.hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setVelocity(0.0);

        magazineMotor = botOpMode.hardwareMap.get(DcMotor.class, "magazineMotor");
        magazineMotor.setDirection(DcMotor.Direction.FORWARD);
        magazineMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazineMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        magazineMotor.setPower(0.0);

        blueWobbleGoal = botOpMode.hardwareMap.get(Servo.class, "blueWobbleGoal");
        blueWobbleGoal.setDirection(Servo.Direction.FORWARD);
        blueWobbleGoal.setPosition(0.0);

        blueCam = botOpMode.hardwareMap.get(Servo.class, "blueCam");
        blueCam.setDirection(Servo.Direction.FORWARD);
        blueCam.setPosition(0.0);

        redWobbleGoal = botOpMode.hardwareMap.get(Servo.class, "redWobbleGoal");
        redWobbleGoal.setDirection(Servo.Direction.REVERSE);
        redWobbleGoal.setPosition(0.0);

        redCam = botOpMode.hardwareMap.get(Servo.class, "redCam");
        redCam.setDirection(Servo.Direction.FORWARD);
        redCam.setPosition(0.0);

        ringServo = botOpMode.hardwareMap.get(Servo.class,"ringServo");
        ringServo.setDirection(Servo.Direction.REVERSE);
        ringServo.setPosition(ringServoClose);

        intakeServo = botOpMode.hardwareMap.get(Servo.class,"intakeServo");
        intakeServo.setDirection(Servo.Direction.REVERSE);
        intakeServo.setPosition(0.0);
    }

    public void initVisionAndTfod(LinearOpMode opMode) throws InterruptedException {
        botOpMode = opMode;
        webcamName = botOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        //int cameraMonitorViewId = botOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", botOpMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
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

        allTrackables.addAll(targets);
        OpenGLMatrix targetOrientation = OpenGLMatrix
                .translation(0, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, -90));

        final float CAMERA_FORWARD_DISPLACEMENT = 9.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_LEFT_DISPLACEMENT = -.50f;     // eg: Camera is ON the robot's center line
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.25f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float PHONE_X_ROTATE = 90;
        final float PHONE_Y_ROTATE = -90;
        final float PHONE_Z_ROTATE = 0;

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, PHONE_X_ROTATE, PHONE_Y_ROTATE, PHONE_Z_ROTATE));

        for (VuforiaTrackable trackable : allTrackables) {
            trackable.setLocation(targetOrientation);
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        if (targets != null) {
            targets.activate();
        }

        //botOpMode = opMode;
        //int cameraMonitorViewId = botOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", botOpMode.hardwareMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //vuforiaParameters.vuforiaLicenseKey = VUFORIA_KEY;
        //vuforiaParameters.cameraName = botOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        //vuforia = ClassFactory.getInstance().createVuforia(vuforiaParameters);
        int tfodMonitorViewId = botOpMode.hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", botOpMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        //TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();
        tfodParameters.minResultConfidence = 0.6f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        tfod.setZoom(1.0, 1.78);
        if (tfod != null) {
            tfod.activate();
        }

    }

    public void deactivateTfod (LinearOpMode opMode) throws InterruptedException {
        botOpMode = opMode;
        tfodCurrentGame.close();
    }

    public void initTfod(LinearOpMode opMode) throws InterruptedException {
        botOpMode = opMode;
        int cameraMonitorViewId = botOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", botOpMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vuforiaParameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuforiaParameters.cameraName = botOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(vuforiaParameters);
        int tfodMonitorViewId = botOpMode.hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", botOpMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.7f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        tfod.setZoom(1.0, 1.78);
        if (tfod != null) {
            tfod.activate();
        }

        //webcamName = botOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        //int cameraMonitorViewId = botOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", botOpMode.hardwareMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraName = webcamName;
        //vuforia = ClassFactory.getInstance().createVuforia(parameters);
//        vuforiaParameters.useExtendedTracking = false;
//        targets = vuforia.loadTrackablesFromAsset("UltimateGoal");
//        targets.get(0).setName("Blue Tower Goal Target");
//        targets.get(1).setName("Red Tower Goal Target");
//        targets.get(2).setName("Red Alliance Target");
//        targets.get(3).setName("Blue Alliance Target");
//        targets.get(4).setName("Front Wall Target");
//
//        allTrackables.addAll(targets);
//        OpenGLMatrix targetOrientation = OpenGLMatrix
//                .translation(0, 0, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
//                        AngleUnit.DEGREES, 90, 0, -90));
//
//        final float CAMERA_FORWARD_DISPLACEMENT = 9.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
//        final float CAMERA_LEFT_DISPLACEMENT = -.50f;     // eg: Camera is ON the robot's center line
//        final float CAMERA_VERTICAL_DISPLACEMENT = 6.25f * mmPerInch;   // eg: Camera is 8 Inches above ground
//        final float PHONE_X_ROTATE = 90;
//        final float PHONE_Y_ROTATE = -90;
//        final float PHONE_Z_ROTATE = 0;
//
//        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
//                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, PHONE_X_ROTATE, PHONE_Y_ROTATE, PHONE_Z_ROTATE));
//
//        for (VuforiaTrackable trackable : allTrackables) {
//            trackable.setLocation(targetOrientation);
//            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, vuforiaParameters.cameraDirection);
//        }
//
//        if (targets != null) {
//            targets.activate();
//        }

//        stopAndResetEncoder();
//        runWithoutEncoder();
    }

//    public void stopTfod(LinearOpMode opMode) throws InterruptedException {
//        botOpMode = opMode;
//        tfod.deactivate();
//        tfod.shutdown();
//    }



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

        allTrackables.addAll(targets);
        OpenGLMatrix targetOrientation = OpenGLMatrix
                .translation(0, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, 0, -90));

        final float CAMERA_FORWARD_DISPLACEMENT = 9.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_LEFT_DISPLACEMENT = -.50f;     // eg: Camera is ON the robot's center line
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.25f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float PHONE_X_ROTATE = 90;
        final float PHONE_Y_ROTATE = -90;
        final float PHONE_Z_ROTATE = 0;

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, PHONE_X_ROTATE, PHONE_Y_ROTATE, PHONE_Z_ROTATE));

        for (VuforiaTrackable trackable : allTrackables) {
            trackable.setLocation(targetOrientation);
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        if (targets != null) {
            targets.activate();
        }
    }

    OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x,y,z).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, u, v, w));
    }

    String formatMatrix (OpenGLMatrix matrix) {
        return matrix.formatAsTransform();
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
        botOpMode = opMode;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = botOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

//        navxMicro = botOpMode.hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
//        gyro = (IntegratingGyroscope) navxMicro;
//
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//
//        while (navxMicro.isCalibrating()) {
//            botOpMode.telemetry.log().clear();
//            botOpMode.telemetry.addData("Calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
//            botOpMode.telemetry.update();
//        }
    }

    /**
     * TeleOp Methods
     **/

    public void startBlinkinBlue() {

        if (blinkinTimer == 0) {
            botOpMode.resetStartTime();
            blinkinTimer = 1;
        }

        if(targetName == "Blue Tower Goal Target"  && botOpMode.time < 80) {
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else if(targetName == "Blue Tower Goal Target"  && botOpMode.time >= 80 && botOpMode.time < 90){
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
        } else if(botOpMode.time >= 80 && botOpMode.time < 90){
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_MEDIUM);
        } else if(targetName == "Blue Tower Goal Target"  && botOpMode.time >= 90 && botOpMode.time < 110){
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else if(botOpMode.time >= 90 && botOpMode.time < 110){
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        } else if(targetName == "Blue Tower Goal Target"  && botOpMode.time >= 110 && botOpMode.time < 120) {
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
        } else if(botOpMode.time >= 110 && botOpMode.time < 120) {
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_MEDIUM);
        } else if(botOpMode.time >= 120) {
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        } else {
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
    }

    public void startBlinkinRed() {

        if (blinkinTimer == 0) {
            botOpMode.resetStartTime();
            blinkinTimer = 1;
        }

        if(targetName == "Red Tower Goal Target"  && botOpMode.time < 70) {
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else if(targetName == "Red Tower Goal Target"  && botOpMode.time >= 70 && botOpMode.time < 90){
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
        } else if(botOpMode.time >= 70 && botOpMode.time < 90){
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_MEDIUM);
        } else if(targetName == "Red Tower Goal Target"  && botOpMode.time >= 90 && botOpMode.time < 110){
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else if(botOpMode.time >= 90 && botOpMode.time < 110){
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        } else if(targetName == "Red Tower Goal Target"  && botOpMode.time >= 110 && botOpMode.time < 120) {
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
        } else if(botOpMode.time >= 110 && botOpMode.time < 120) {
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_MEDIUM);
        } else if(botOpMode.time >= 120) {
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        } else {
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
    }

    public boolean cruiseControl(double cruiseControlRange, double cruiseControlOffet, double cruisecontrolAngle,
                                 double cruiseControlAxialGain, double cruiseControlLateralGain, double cruiseControlYawGain) {
        double Y = ((relativeBearing + cruisecontrolAngle) * cruiseControlYawGain);
        double L = ((robotY + cruiseControlOffet) * cruiseControlLateralGain);
        double A = (-(robotX + cruiseControlRange) * cruiseControlAxialGain);
        setYaw(Y);
        setAxial(A);
        setLateral(L);
        closeEnough = ((Math.abs(robotX + cruiseControlRange) < CLOSE_ENOUGH) && (Math.abs(robotY) < ON_AXIS));
        return (closeEnough);
    }

    public boolean powerShot(double powerShotRange, double powershotOffset, double powershotAngle,
                                 double powerShotAxialGain, double powershotLateralGain,double powershotYawGain) {
        boolean closeEnough;
        double Y = ((relativeBearing + powershotAngle) * powershotYawGain);
        double L = ((robotY + powershotOffset) * powershotLateralGain);
        double A = (-(robotX + powerShotRange) * powerShotAxialGain);
        setYaw(Y);
        setAxial(A);
        setLateral(L);
        closeEnough = ((Math.abs(robotX + powerShotRange) < CLOSE_ENOUGH) && (Math.abs(robotY) < ON_AXIS));
        return (closeEnough);
    }

    public void manualDrive() throws InterruptedException {
        // Setting three motions to stick movements
        setAxial(-botOpMode.gamepad1.left_stick_y);
        setLateral(botOpMode.gamepad1.left_stick_x);
        setYaw(-botOpMode.gamepad1.right_stick_x);

        // Logic for speed control on button A
//        if (botOpMode.gamepad1.x) {
//            aButtonPad1 = true;
//        } else if (aButtonPad1) {
//            aButtonPad1 = false;
//            if (speed == LOWSPEED) {
//                speed = HIGHSPEED;
//            } else {
//                speed = LOWSPEED;
//            }
//        }

        // Logic for direction control on button B
//        if (botOpMode.gamepad1.y) {
//            bButtonPad1 = true;
//        } else if (bButtonPad1) {
//            bButtonPad1 = false;
//            direction = -direction;
//        }
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
        double moveBackLeft = speed * (direction * ((-driveAxial - (driveLateral * STRAFESENSITIVITY))) + (driveYaw * TURNSENSITIVITY));
        double moveBackRight = speed * (direction * ((-driveAxial + (driveLateral * STRAFESENSITIVITY))) - (driveYaw * TURNSENSITIVITY));

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

    public void shootLauncher() throws InterruptedException {
        if (launcherVelocity == launcher.getVelocity()) {
            ringServo.setPosition(ringServoClose);
            Thread.sleep(shotSpeed);
            ringServo.setPosition(ringServoOpen);
        }
    }

    public void initAuxiliaryControls() throws InterruptedException {

        intakeMotor.setVelocity(0.0);
        ringServo.setPosition(ringServoOpen);
        launcher.setVelocity(launcherVelocity);

        double position = .01;
        for (int i=0; i<90; i++) {
            intakeServo.setPosition(position);
            Thread.sleep(10);
            position = position + .01;
        }
    }

    public void auxiliaryControls() throws InterruptedException {

        if (botOpMode.gamepad1.x && intakeMotor.getVelocity() == 0.0) {
            intakeMotor.setVelocity(intakeVelocity);
            lowerMagazine();
        }

        if (botOpMode.gamepad1.y) {
            intakeMotor.setVelocity(0.0);
            raiseMagazine();
        }

        if (botOpMode.gamepad1.b && intakeMotor.getVelocity() == 0.0) {
            intakeMotor.setVelocity(-intakeVelocity);
            lowerMagazine();
        }

//        if (botOpMode.gamepad1.dpad_up){
//            //launcher.setPower(launcher.getPower() + launcherPowerIncrement);
//            launcherVelocity = launcher.getVelocity() + launcherVelocityIncrement;
//            launcher.setVelocity(launcherVelocity);
//        }
//
//        if (botOpMode.gamepad1.dpad_down){
//            //launcher.setPower(launcher.getPower() - launcherPowerIncrement);
//            launcherVelocity = launcher.getVelocity() - launcherVelocityIncrement;
//            launcher.setVelocity(launcherVelocity);
//        }
//
//        if (botOpMode.gamepad1.dpad_right) {
//            //launcher.setPower(launcherPower);
//            launcherVelocity = launcherHighGoalVelocity;
//            launcher.setVelocity(launcherVelocity);
//        }
//
//        if (botOpMode.gamepad1.dpad_left) {
//            //launcher.setPower(0);
//            launcherVelocity = 0.0;
//            launcher.setVelocity(launcherVelocity);
//
//        }

        if (botOpMode.gamepad1.a && magazineMotor.getCurrentPosition() > 100 && launcher.getVelocity() == launcherVelocity) {
            shootLauncher();
        }

        if (botOpMode.gamepad1.left_trigger == 1.0) {
            launcherVelocity = launcherPowershotVelocity;
            launcher.setVelocity(launcherVelocity);
        }

        if (botOpMode.gamepad1.left_trigger == 0.0) {
            launcherVelocity = launcherHighGoalVelocity;
            launcher.setVelocity(launcherVelocity);
        }

        if (botOpMode.gamepad2.x && intakeMotor.getVelocity() == 0.0) {
            intakeMotor.setVelocity(intakeVelocity);
        }

        if (botOpMode.gamepad2.y) {
            intakeMotor.setVelocity(0.0);
        }

        if (botOpMode.gamepad2.b && intakeMotor.getVelocity() == 0.0) {
            intakeMotor.setVelocity(-intakeVelocity);
        }

        if (botOpMode.gamepad2.left_bumper) {
            pickupBlueWobbleGoal();
        }

        if (botOpMode.gamepad2.right_bumper) {
            raiseBlueWobbleGoal();
        }
    }

    /**
     * Autonomous Methods
     **/

    public boolean autoCruiseControl(double cruiseControlRange, double cruiseControlOffet, double cruisecontrolAngle,
                                 double cruiseControlAxialGain, double cruiseControlLateralGain, double cruiseControlYawGain) {
        boolean closeEnough;
        double Y = ((relativeBearing + cruisecontrolAngle) * cruiseControlYawGain);
        double L = ((robotY + cruiseControlOffet) * cruiseControlLateralGain);
        double A = (-(robotX + cruiseControlRange) * cruiseControlAxialGain);
        setYaw(Y);
        setAxial(A);
        setLateral(L);
        closeEnough = ((Math.abs(robotX + cruiseControlRange) < CLOSE_ENOUGH) && (Math.abs(robotY) < ON_AXIS));
        return (closeEnough);
    }

    public void signalBlueAlliance() {
        ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    public void launcherAutoPowershot(double velocity) throws InterruptedException {

        launcher.setVelocity(velocity);

        while (botOpMode.opModeIsActive()) {
            if (launcher.getVelocity() == velocity) {
                shootAutoLauncher();
                break;
            }
        }
    }

    public void launcherOff () {

        launcher.setPower(0.0);

    }

    public void magazineSetup(double power, int time) throws InterruptedException {

        magazineMotor.setTargetPosition(magazineTargetPosition);
        magazineMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        magazineMotor.setPower(power);
        Thread.sleep(time);
        magazineMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazineMotor.setTargetPosition(0);
        magazineMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        magazineMotor.setPower(0);
    }

    public void raiseMagazine() {

        magazineMotor.setTargetPosition(magazineTargetPosition);
        magazineMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        magazineMotor.setPower(1.0);
        while (magazineMotor.isBusy()) {
            try {
                manualDrive();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        ringServo.setPosition(ringServoOpen);
    }

    public void lowerMagazine() {
        magazineMotor.setTargetPosition(0);
        magazineMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        magazineMotor.setPower(.30);
        while (magazineMotor.isBusy()) {
            try {
                manualDrive();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void shootAutoLauncher() throws InterruptedException {
        ringServo.setPosition(ringServoClose);
        Thread.sleep(700);
        ringServo.setPosition(ringServoOpen);
        Thread.sleep(700);
    }

    public void prepareLauncher(double launcherAutoVelocity) throws InterruptedException {
        launcher.setVelocity(launcherAutoVelocity);
    }

    public void dropBlueWobbleGoal() {

        blueWobbleGoal.setPosition(1.0);
        redWobbleGoal.setPosition(1.0);

    }

    public void pickupBlueWobbleGoal() {

        blueWobbleGoal.setPosition(.60);
        redWobbleGoal.setPosition(.60);

    }


    public void raiseBlueWobbleGoal() {

        blueWobbleGoal.setPosition(0.0);
        redWobbleGoal.setPosition(0.0);

    }

    public void gyroForward(int distance, double power, double angle, int pause) throws InterruptedException {
        if (botOpMode.opModeIsActive()) {
            stopAndResetEncoder();
            driveBreak();
            driveByInches(distance);
            runToPosition();
            while (botOpMode.opModeIsActive()) {
                double gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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
            driveBreak();
            stopAndResetEncoder();
            driveByInches(-distance);
            runToPosition();
            while (botOpMode.opModeIsActive()) {
                double gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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
            driveBreak();
            stopAndResetEncoder();
            strafeByInches(-distance);
            runToPosition();
            while (botOpMode.opModeIsActive()) {
                double gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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
            driveBreak();
            stopAndResetEncoder();
            strafeByInches(distance);
            runToPosition();
            while (botOpMode.opModeIsActive()) {
                double gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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
            double halfPower = power/2;
            int cutSpeed = 20;
            int angleTolerance = 5;
            int cycleTime = 250;
            driveBreak();
            stopAndResetEncoder();
            runUsingEncoder();
            while (botOpMode.opModeIsActive()) {
                double gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (angle - angleTolerance - cutSpeed > gyroHeading) {
                    frontLeft.setPower(power);
                    frontRight.setPower(-power);
                    backLeft.setPower(power);
                    backRight.setPower(-power);
                } else if (angle - angleTolerance > gyroHeading) {
                    frontLeft.setPower(halfPower);
                    frontRight.setPower(-halfPower);
                    backLeft.setPower(halfPower);
                    backRight.setPower(-halfPower);
                    Thread.sleep(cycleTime);
                } else if (angle + angleTolerance < gyroHeading) {
                    frontLeft.setPower(-halfPower);
                    frontRight.setPower(halfPower);
                    backLeft.setPower(-halfPower);
                    backRight.setPower(halfPower);
                    Thread.sleep(cycleTime);
                } else {
                    stopDriving();
                    Thread.sleep(pause);
                    driveTelemetry();
                    break;
                }
            }
        }
    }

    public void gyroRight(double power, double angle, int pause) throws InterruptedException {
        if (botOpMode.opModeIsActive()) {
            double halfPower = power/2;
            int cutSpeed = 20;
            int angleTolerance = 5;
            int cycleTime = 250;
            driveBreak();
            stopAndResetEncoder();
            runUsingEncoder();
            while (botOpMode.opModeIsActive()) {
                double gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (angle + angleTolerance + cutSpeed < gyroHeading) {
                    frontLeft.setPower(-power);
                    frontRight.setPower(power);
                    backLeft.setPower(-power);
                    backRight.setPower(power);
                } else if (angle + angleTolerance < gyroHeading) {
                    frontLeft.setPower(-halfPower);
                    frontRight.setPower(halfPower);
                    backLeft.setPower(-halfPower);
                    backRight.setPower(halfPower);
                    Thread.sleep(cycleTime);
                } else if (angle - angleTolerance > gyroHeading) {
                    frontLeft.setPower(halfPower);
                    frontRight.setPower(-halfPower);
                    backLeft.setPower(halfPower);
                    backRight.setPower(-halfPower);
                    Thread.sleep(cycleTime);
                } else {
                    stopDriving();
                    Thread.sleep(pause);
                    driveTelemetry();
                    break;
                }
            }
        }
    }

    public void gyroLeftPowershot(double power, double angle, int pause) throws InterruptedException {
        if (botOpMode.opModeIsActive()) {
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            Thread.sleep(pause);
            double halfPower = power/2;
            int cutSpeed = 20;
            int angleTolerance = 3;
            int cycleTime = 250;
            driveBreak();
            stopAndResetEncoder();
            runUsingEncoder();
            while (botOpMode.opModeIsActive()) {
                double gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (angle - angleTolerance - cutSpeed > gyroHeading) {
                    frontLeft.setPower(power);
                    frontRight.setPower(-power);
                    backLeft.setPower(power);
                    backRight.setPower(-power);
                } else if (angle - angleTolerance > gyroHeading) {
                    frontLeft.setPower(halfPower);
                    frontRight.setPower(-halfPower);
                    backLeft.setPower(halfPower);
                    backRight.setPower(-halfPower);
                    Thread.sleep(cycleTime);
                } else if (angle + angleTolerance < gyroHeading) {
                    frontLeft.setPower(-halfPower);
                    frontRight.setPower(halfPower);
                    backLeft.setPower(-halfPower);
                    backRight.setPower(halfPower);
                    Thread.sleep(cycleTime);
                } else {
                    stopDriving();
                    ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    Thread.sleep(pause * 2);
                    break;
                }
            }
        }
    }

    public void gyroRightPowershot(double power, double angle, int pause) throws InterruptedException {
        if (botOpMode.opModeIsActive()) {
            ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            Thread.sleep(pause);
            double halfPower = power/2;
            int cutSpeed = 20;
            int angleTolerance = 2;
            int cycleTime = 250;
            driveBreak();
            stopAndResetEncoder();
            runUsingEncoder();
            while (botOpMode.opModeIsActive()) {
                double gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                if (angle + angleTolerance + cutSpeed < gyroHeading) {
                    frontLeft.setPower(-power);
                    frontRight.setPower(power);
                    backLeft.setPower(-power);
                    backRight.setPower(power);
                } else if (angle + angleTolerance < gyroHeading) {
                    frontLeft.setPower(-halfPower);
                    frontRight.setPower(halfPower);
                    backLeft.setPower(-halfPower);
                    backRight.setPower(halfPower);
                    Thread.sleep(cycleTime);
                } else if (angle - angleTolerance > gyroHeading) {
                    frontLeft.setPower(halfPower);
                    frontRight.setPower(-halfPower);
                    backLeft.setPower(halfPower);
                    backRight.setPower(-halfPower);
                    Thread.sleep(cycleTime);
                } else {
                    stopDriving();
                    ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    Thread.sleep(pause * 2);
                    break;
                }
            }
        }
    }

    /**
     *  Drive Train Methods
     */

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
        frontLeft.setMode(RUN_USING_ENCODER);
        frontRight.setMode(RUN_USING_ENCODER);
        backLeft.setMode(RUN_USING_ENCODER);
        backRight.setMode(RUN_USING_ENCODER);
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

    public void driveBreak () {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void driveFloat () {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Telemetry Methods
     **/

    public void driveTelemetry() throws InterruptedException {

        if (botOpMode.gamepad1.y) {
            stopAndResetEncoder();
            runUsingEncoder();
        }

        double gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double gyroZ = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double gyroY = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES).secondAngle;
        double gyroX = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES).thirdAngle;
        double frontLeftInches = frontLeft.getCurrentPosition() / TICKSTOINCHES;
        double frontRightInches = frontRight.getCurrentPosition() / TICKSTOINCHES;
        double backLeftInches = backLeft.getCurrentPosition() / TICKSTOINCHES;
        double backRightInches = backRight.getCurrentPosition() / TICKSTOINCHES;

        botOpMode.telemetry.log().clear();
        botOpMode.telemetry.addData("Note", "--> Tap Y to reset encoders");
        botOpMode.telemetry.addData("Heading", "%.2f", gyroHeading);
        botOpMode.telemetry.addData("Z", "%.2f", gyroZ);
        botOpMode.telemetry.addData("Y", "%.2f", gyroY);
        botOpMode.telemetry.addData("X", "%.2f", gyroX);
//        botOpMode.telemetry.addData("Inches", "FL: %2d, FR: %2d, BL: %2d, BR: %2d", -(int) frontLeftInches, -(int) frontRightInches, -(int) backLeftInches, -(int) backRightInches);
//        botOpMode.telemetry.addData("Encoder", "FL: %2d,  FR: %2d, BL: %2d, BR: %2d", -frontLeft.getCurrentPosition(), -frontRight.getCurrentPosition(), -backLeft.getCurrentPosition(), -backRight.getCurrentPosition());
//        botOpMode.telemetry.addData("Drive Power", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f", -frontLeft.getPower(), -frontRight.getPower(), -backLeft.getPower(), -backRight.getPower());
//        botOpMode.telemetry.addData("Axes  ", "A[%+5.2f], L[%+5.2f], Y[%+5.2f]", driveAxial, driveLateral, driveYaw);
        //        botOpMode.telemetry.addData("Launcher Power", "LL: %.2f, RL: %.2f", -leftLauncher.getPower(), -rightLauncher.getPower());
        botOpMode.telemetry.update();
    }

    public void initTelemetry() {
        botOpMode.telemetry.log().clear();
        botOpMode.telemetry.addData("Robot Initialized", "Press Play to Begin");
        botOpMode.telemetry.update();
        ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void initTfodTelemetry() throws InterruptedException {

        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        if (updatedRecognitions != null) {
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                tfodDetected = recognition.getLabel();
                botOpMode.telemetry.log().clear();
                botOpMode.telemetry.addData("Robot Initialized", "Press Play to Begin");
                botOpMode.telemetry.addData(String.format("Detected (%d)", i), recognition.getLabel());
                botOpMode.telemetry.update();
                Thread.sleep(1000);
            }
        } else {
            tfodDetected = "Zero";
            botOpMode.telemetry.log().clear();
            botOpMode.telemetry.addData("Robot Initialized", "Press Play to Begin");
            botOpMode.telemetry.addData("Detected", "Zero");
            botOpMode.telemetry.update();
        }

        ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

    }

    public void tfodRunningTelemetry() {
        botOpMode.telemetry.log().clear();
        botOpMode.telemetry.addData("Running", getTfodDetected() + " Program");
        botOpMode.telemetry.update();
    }

    public void cruiseControlTelemetry() {
        if (targetFound) {
            // Display the current visible target name, robot info, target info, and required robot action.
            botOpMode.telemetry.log().clear();
            botOpMode.telemetry.addData(">", "Press Left Bumper to track target");
            botOpMode.telemetry.addData("Magazine Target Position", "%2d", magazineTargetPosition);
            botOpMode.telemetry.addData("Magazine Current Position", "%2d", magazineMotor.getCurrentPosition());
            botOpMode.telemetry.addData("Launcher Power", launcher.getPower());
            botOpMode.telemetry.addData("Launcher Velocity", "%.2f", launcher.getVelocity());
            botOpMode.telemetry.addData("Robot", "[X]:[Y] (B) [%5.0fmm]:[%5.0fmm] (%4.0f°)", robotX, robotY, robotBearing);
            botOpMode.telemetry.addData("Target", "[R] (B):(RB) [%5.0fmm] (%4.0f°):(%4.0f°)", targetRange, targetBearing, relativeBearing);
            botOpMode.telemetry.addData("- Turn    ", "%s %4.0f°", relativeBearing < 0 ? ">>> CW " : "<<< CCW", Math.abs(relativeBearing));
            botOpMode.telemetry.addData("- Strafe  ", "%s %5.0fmm", robotY < 0 ? "LEFT" : "RIGHT", Math.abs(robotY));
            botOpMode.telemetry.addData("- Distance", "%5.0fmm", Math.abs(robotX));
            botOpMode.telemetry.addData("Axes  ", "A[%+5.2f], L[%+5.2f], Y[%+5.2f]", driveAxial, driveLateral, driveYaw);
            botOpMode.telemetry.update();

        } else {
            botOpMode.telemetry.log().clear();
            botOpMode.telemetry.addData(">", "Press Left Bumper to track target");
            botOpMode.telemetry.addData("Visible", "- - - -");
            botOpMode.telemetry.addData("Magazine Target Position", "%2d", magazineTargetPosition);
            botOpMode.telemetry.addData("Magazine Current Position", "%2d", magazineMotor.getCurrentPosition());
            botOpMode.telemetry.addData("Launcher Power", launcher.getPower());
            botOpMode.telemetry.addData("Launcher Velocity", "%.2f", launcher.getVelocity());
            botOpMode.telemetry.update();
        }
    }
}
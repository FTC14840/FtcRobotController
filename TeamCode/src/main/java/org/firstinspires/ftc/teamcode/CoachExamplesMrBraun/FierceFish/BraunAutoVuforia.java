package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.FierceFish;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

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
import org.firstinspires.ftc.teamcode.R;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous(name = "BraunAutoVuforia")

@Disabled

public class BraunAutoVuforia extends LinearOpMode {

    // https://developer.vuforia.com/license-manager
    private static final String VUFORIA_KEY = "ATqulq//////AAABmfYPXE+z1EORrVmv4Ppo3CcPktGk5mvdMnvPi9/T3DMYGc2mju8KUyG9gAB7pKlb9k9SZnM0YSq1JUZ6trE1ZKmMU8z5QPuhA/b6/Enb+XVGwmjrRjhMfNtUNgiZDhtsUvxr9fQP4HVjTzlz4pv0z3MeWZmkAgIN8T8YM0EFWrW4ODqYQmZjB0Nri2KKVM9dlOZ5udPfTZ9YvMgrCyxxG7O8P84AvwCAyXxzxelL4OfGnbygs0V60CQHx51gqrki613PT/9D1Q1io5+UbN6xAQ26AdYOTmADgJUGlfC2eMyqls4qAIoOj+pcJbm5ryF5yW9pEGHmvor1c9HlCFwhKxiaxw+cTu8AEaAdNuR65i/p";
    private static final float MM_PER_INCH = 25.4f;
    private static final float TARGET_HEIGHT = MM_PER_INCH * 6;  // Center of target is 6 inches above the ground
    private static final float HALF_FIELD = MM_PER_INCH * 72;  // (12*12)/2
    private static final float QUARTER_FIELD = MM_PER_INCH * 36;  // (12*12)/4
    private static final float CAMERA_FORWARD = MM_PER_INCH * 0;  // Inches forward from center of bot
    private static final float CAMERA_LATERAL = MM_PER_INCH * 0;  // Inches left(-) or right(+) of bot's midline
    private static final float CAMERA_VERTICAL = MM_PER_INCH * 0;  // Inches above the ground
    private static final float CAMERA_U_Rotation = 90;  // Webcam/Back = 90 or Front = -90
    private static final float CAMERA_V_Rotation = 0;
    private static final float CAMERA_W_Rotation = 0;

    WebcamName webcamName;
    VuforiaLocalizer vuforia;
    VuforiaLocalizer.Parameters vuforiaParameters;
    VuforiaTrackableDefaultListener vuforiaListener;
    VuforiaTrackables vuforiaTargets;
//    VuforiaTrackable blueTowerGoal = vuforiaTargets.get(0);
//    VuforiaTrackable RedTowerGoal = vuforiaTargets.get(1);
    OpenGLMatrix phoneLocation;
    OpenGLMatrix robotsLastKnownLocation;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    @Override
    public void runOpMode() throws InterruptedException {

        initVuforia();

        while (!isStarted()) {

        }

        waitForStart();

        while(opModeIsActive()){

        }

    }

    void initVuforia(){

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vuforiaParameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuforiaParameters.cameraName  = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforiaParameters.useExtendedTracking = false;
        vuforia = ClassFactory.getInstance().createVuforia(vuforiaParameters);

        vuforiaTargets = vuforia.loadTrackablesFromAsset("UltimateGoal");
        // Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,5);

        vuforiaTargets.get(0).setName("Blue Tower Goal Target");
        //vuforiaTargets.get(0).setLocation(createMatrix(HALF_FIELD, QUARTER_FIELD, TARGET_HEIGHT,90, 0 , -90));

        vuforiaTargets.get(1).setName("Red Tower Goal Target");
        //vuforiaTargets.get(1).setLocation(createMatrix(HALF_FIELD, -QUARTER_FIELD, TARGET_HEIGHT,90, 0 , -90));

        vuforiaTargets.get(2).setName("Red Alliance Target");
        //vuforiaTargets.get(2).setLocation(createMatrix(0, -HALF_FIELD, TARGET_HEIGHT,90, 0 , 180));

        vuforiaTargets.get(3).setName("Blue Alliance Target");
        //vuforiaTargets.get(3).setLocation(createMatrix(0, HALF_FIELD, TARGET_HEIGHT,90, 0 , 0));

        vuforiaTargets.get(4).setName("Front Wall Target");
        //vuforiaTargets.get(4).setLocation(createMatrix(-HALF_FIELD, 0, TARGET_HEIGHT,90, 0 , 90));

        // vuforiaListener = (VuforiaTrackableDefaultListener) target.getListener();
        // vuforiaListener.setPhoneInformation(phoneLocation,vuforiaParameters.cameraDirection);

        allTrackables.addAll(vuforiaTargets);
        OpenGLMatrix targetOrientation = createMatrix(0,0, TARGET_HEIGHT,90,0,0);

        phoneLocation = createMatrix (CAMERA_FORWARD,CAMERA_LATERAL,CAMERA_VERTICAL,CAMERA_U_Rotation,CAMERA_V_Rotation,CAMERA_W_Rotation);
        robotsLastKnownLocation = createMatrix(0,0,0,0,0,0);

        for (VuforiaTrackable trackable : allTrackables) {
            trackable.setLocation(targetOrientation);
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocation, vuforiaParameters.cameraDirection);
        }
        vuforiaTargets.activate();
    }

    OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w){
        return OpenGLMatrix.translation(x,y,z).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, u, v, w));
    }

    String formatMatrix (OpenGLMatrix matrix) {
        return matrix.formatAsTransform();
    }

}


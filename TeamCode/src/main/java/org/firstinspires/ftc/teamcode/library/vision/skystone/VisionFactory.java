package org.firstinspires.ftc.teamcode.library.vision.skystone;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.library.vision.skystone.opencv.OpenCvContainer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.library.vision.skystone.VisionConstants.*;
public class VisionFactory {

    static String webcamName = "Webcam 720p";

    public static VuforiaLocalizer createVuforia(CameraType cameraType, HardwareMap hardwareMap) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(getCameraMonitorViewId(hardwareMap));
        parameters.vuforiaLicenseKey = "AVsEI0//////AAABmfbnDsRuiEqQsFcH7Lyqo3QuISqpJopGJxX15CNQ6JRwa6IhhZMdS346pkBRyp94aOkulPOzR+MQZ84lQbPclj/UW9I95nliTUyCT+Ie8Bw9qNN5X5Cv4sBkIAyNJpUXfxMjUf/5Hw098czCJ1HTiVoqUVB+AGNgZ6tLD4AGqbv/ftucrrA/nnzT045vPyCZCKujFStiBc1Hkab9Y96FE5wHPrfBeCrq8nYd0T+mB3eaCO3kUahLaqyjhFAyQnQCpa1oOzsqbDARsc5FuIcACzdzOFSG+LAEOj+Bgb7Nm+jJQHn8rYsCJi94aoSptfsgXSR0N2pTwhM3eQZEf9g6m1fbUJH18eCFJWwS1FVr3U+6";
        switch (cameraType) {
            case PHONE_REAR:
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                break;
            case PHONE_FRONT:
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
                break;
            case WEBCAM:
                parameters.cameraName = hardwareMap.get(WebcamName.class, webcamName);
        }
        return ClassFactory.getInstance().createVuforia(parameters);
    }

    public static TFObjectDetector attachTFOD(HardwareMap hardwareMap, VuforiaLocalizer vuforia) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        TFObjectDetector tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET_EXT, TFOD_LABEL_STONE, TFOD_LABEL_SKYSTONE);
        return tfod;
    }

    public static <Pipeline extends OpenCvPipeline> OpenCvContainer<Pipeline> createOpenCv(CameraType cameraType, HardwareMap hardwareMap, Pipeline pipeline) {
        OpenCvCamera camera;
        ImageResolution resolution = ImageResolution.R_1280x720;
        OpenCvCameraRotation rotation = OpenCvCameraRotation.UPRIGHT;

        if (cameraType == CameraType.WEBCAM) {
            camera =
                    OpenCvCameraFactory.getInstance().createWebcam(
                            hardwareMap.get(WebcamName.class, webcamName),
                            getCameraMonitorViewId(hardwareMap));
            resolution = ImageResolution.R_640x480;
        }
        else  {
            camera =
                    OpenCvCameraFactory.getInstance().createInternalCamera(
                            (cameraType==CameraType.PHONE_FRONT)? OpenCvInternalCamera.CameraDirection.FRONT : OpenCvInternalCamera.CameraDirection.BACK,
                            getCameraMonitorViewId(hardwareMap));
            rotation = OpenCvCameraRotation.SIDEWAYS_RIGHT;
        }

        return new OpenCvContainer<Pipeline>(camera, pipeline, resolution, rotation);
    }

    private static int getCameraMonitorViewId(HardwareMap hardwareMap) {
        return hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    }

    public enum CameraType {
        WEBCAM, PHONE_REAR, PHONE_FRONT
    }
}

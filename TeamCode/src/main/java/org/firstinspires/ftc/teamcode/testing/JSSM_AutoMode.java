package org.firstinspires.ftc.teamcode.testing;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="JSSM Detector", group="Auto")
public class JSSM_AutoMode extends LinearOpMode {
    OpenCvCamera webCam;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        JSSM_CV1 detector = new JSSM_CV1(telemetry);
        webCam.setPipeline(detector);

        /*@Override
        public void onOpened() {
            webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        }*/
        webCam.openCameraDevice();
        webCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);

        waitForStart();
        switch (detector.getLocation()) {
            case LEFT:
                break;
            case RIGHT:
                break;
            case NOT_FOUND:
        }
        webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }
}

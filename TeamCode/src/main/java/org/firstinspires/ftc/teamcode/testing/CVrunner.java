package org.firstinspires.ftc.teamcode.testing;

import org.firstinspires.ftc.teamcode.testing.JSSM_CV1;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


import java.util.jar.Attributes;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="cvRunner-Nipun", group="TeleOp")
public class CVrunner extends LinearOpMode {

    private BluePipeline.Location location;

    OpenCvWebcam camera;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "identifyier","teamcode");
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        JSSM_CV1 detector = new JSSM_CV1(telemetry);
        camera.setPipeline(detector);
        camera.setMillisecondsPermissionTimeout(5000);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {



            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);

        waitForStart();

        switch(detector.getLocation()) {
            case LEFT:
                break;
            case RIGHT:
                break;
            case NOT_FOUND:
                break;
        }

        camera.stopStreaming();
    }
}
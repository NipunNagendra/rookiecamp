package org.firstinspires.ftc.teamcode.testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Config
@Autonomous(name="detectorMain", group="Auto")
public class detectorMain extends LinearOpMode {
    OpenCvWebcam camera;
    public static double color = 3;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "identifyier","teamcode");
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        RedPipeline detectRed = new RedPipeline(telemetry);
        BluePipeline detectBlue = new BluePipeline(telemetry);
        RobotDetectionPipeline detectRobot = new RobotDetectionPipeline(telemetry);
        if (color==1){
            camera.setPipeline(detectRed);
        }
        else if (color==2) {
            camera.setPipeline(detectBlue);
        }
        else if (color == 3) {
            camera.setPipeline(detectRobot);
        }
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


        camera.stopStreaming();
    }
}

package org.firstinspires.ftc.teamcode.testing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RobotDetectionPipeline extends OpenCvPipeline {

        Telemetry telemetry;
        Mat mat = new Mat();
        public enum Location {
            LEFT,
            RIGHT,
            FRONT,
            NOT_FOUND
        }
        private org.firstinspires.ftc.teamcode.testing.BluePipeline.Location location;

        static final Rect FRONT_ROI = new Rect(
                new Point(220, -35),
                new Point(280, 75));
        public static double PERCENT_COLOR_THRESHOLD = 0.4;
        public RobotDetectionPipeline(Telemetry t) { telemetry = t; }

        public static double lh;
        public static double ls;
        public static double lv;
        public static double hh;
        public static double hs;
        public static double hv;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

            //Blue
            Scalar lowerBlueHSV = new Scalar(100, 60, 100);
            Scalar highBlueHSV = new Scalar(180, 255, 255);

            Core.inRange(mat, lowerBlueHSV, highBlueHSV, mat);

            Mat front = mat.submat(FRONT_ROI);


            double frontValue = Core.sumElems(front).val[0] / FRONT_ROI.area() / 255;


            front.release();


            telemetry.addData("Front Raw Value", (int) Core.sumElems(front).val[0]);

            telemetry.addData("Front Percentage", Math.round(frontValue * 100) + "%");


            boolean pixelFront = frontValue > PERCENT_COLOR_THRESHOLD;

            /*if (pixelLeft && pixelRight || pixelLeft && pixelFront || pixelRight && pixelFront) {
                location = org.firstinspires.ftc.teamcode.testing.BluePipeline.Location.NOT_FOUND;
                telemetry.addData("Pixel Location", "not found");
            }
            else if (pixelRight) {
                location = org.firstinspires.ftc.teamcode.testing.BluePipeline.Location.RIGHT;
                telemetry.addData("Pixel Location", "right");
            }
            else if (pixelFront) {
                location = org.firstinspires.ftc.teamcode.testing.BluePipeline.Location.FRONT;
                telemetry.addData("Pixel Location", "front");
            }
            else {
                location = org.firstinspires.ftc.teamcode.testing.BluePipeline.Location.LEFT;
                telemetry.addData("Pixel Location", "left");
            }
            telemetry.update();*/

            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

            Scalar undetected = new Scalar(255, 0, 0);
            Scalar detected = new Scalar(0, 255, 0);



            Imgproc.rectangle(mat, FRONT_ROI, location == org.firstinspires.ftc.teamcode.testing.BluePipeline.Location.FRONT? detected:undetected);

            return mat;
        }

        public org.firstinspires.ftc.teamcode.testing.BluePipeline.Location getLocation() {
            return location;
        }
}

package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
@Config

public class BluePipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        RIGHT,
        FRONT,
        NOT_FOUND
    }
    private Location location;

    static final Rect LEFT_ROI = new Rect(
            new Point(60, 35),
            new Point(120, 75));
    static final Rect RIGHT_ROI = new Rect(
            new Point(260, 70),
            new Point(320, 130));
    static final Rect FRONT_ROI = new Rect(
            new Point(110, 55),
            new Point(170, 115));
    public static double PERCENT_COLOR_THRESHOLD = 0.4;
    public BluePipeline(Telemetry t) { telemetry = t; }

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

        Mat right = mat.submat(RIGHT_ROI);
        Mat front = mat.submat(FRONT_ROI);

        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double frontValue = Core.sumElems(front).val[0] / FRONT_ROI.area() / 255;

        right.release();
        front.release();

        telemetry.addData("Right Raw Value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Front Raw Value", (int) Core.sumElems(front).val[0]);
        telemetry.addData("Right Percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData("Front Percentage", Math.round(frontValue * 100) + "%");

        boolean pixelRight = rightValue > PERCENT_COLOR_THRESHOLD;
        boolean pixelFront = frontValue > PERCENT_COLOR_THRESHOLD;

        if (pixelRight) {
            location = Location.RIGHT;
            telemetry.addData("Pixel Location", "right");
        }
        else if (pixelFront) {
            location = Location.FRONT;
            telemetry.addData("Pixel Location", "front");
        }
        else{
            location = Location.LEFT;
            telemetry.addData("Pixel Location", "left");
        }

        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar undetected = new Scalar(255, 0, 0);
        Scalar detected = new Scalar(0, 255, 0);


        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? detected:undetected);
        Imgproc.rectangle(mat, FRONT_ROI, location == Location.FRONT? detected:undetected);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}
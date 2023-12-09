package org.firstinspires.ftc.teamcode.testing;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
public class NipunCV extends OpenCvPipeline{
    Telemetry telemetry;
        Mat mat = new Mat();
        public enum Location {
            LEFT,
            FRONT,
            RIGHT,
            NOT_FOUND
        }
        private Location location;

        static final Rect LEFT_ROI = new Rect(
                new Point(60, 35),
                new Point(120, 75));
        static final Rect RIGHT_ROI = new Rect(
                new Point(140, 35),
                new Point(200, 75));
        static final Rect FRONT_ROI = new Rect(
            new Point(20, 10),
            new Point(0, 5));
        static double PERCENT_COLOR_THRESHOLD = 0.4;

        public NipunCV(Telemetry t) { telemetry = t; }


    @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            Scalar lowHSV = new Scalar(23, 50, 70);
            Scalar highHSV = new Scalar(32, 255, 255);

            mat.inv();

            Mat left = mat.submat(LEFT_ROI);
            Mat front = mat.submat(FRONT_ROI);
            Mat right = mat.submat(RIGHT_ROI);

            double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
            double frontValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
            double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

            left.release();
            front.release();
            right.release();

            telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
            telemetry.addData("Front raw value", (int) Core.sumElems(front).val[0]);
            telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
            telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
            telemetry.addData("Front percentage", Math.round(frontValue * 100) + "%");
            telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

            boolean propLeft = leftValue > PERCENT_COLOR_THRESHOLD;
            boolean propFront = frontValue > PERCENT_COLOR_THRESHOLD;
            boolean propRight = rightValue > PERCENT_COLOR_THRESHOLD;

            if (propLeft && propRight || propLeft && propFront || propRight && propFront) {
                location = Location.NOT_FOUND;
                telemetry.addData("Prop Location", "not found");
            }
            else if(propRight) {
                location = Location.RIGHT;
                telemetry.addData("Prop Location", "right");
            }
            else if(propLeft){
                location = Location.LEFT;
                telemetry.addData("Prop Location", "left");
            }
            else{
                location = Location.FRONT;
                telemetry.addData("Prop Location", "front");
            }
            telemetry.update();

            return mat;
        }

        public Location getLocation() {
            return location;
        }
    }

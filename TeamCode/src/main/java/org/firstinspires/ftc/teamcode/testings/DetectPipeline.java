package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
@Config
public class DetectPipeline extends OpenCvPipeline{
    public static Scalar lower_yellow = new Scalar(10, 60, 50);
    public static Scalar upper_yellow = new Scalar(30, 255, 255);
    public static int min_detect_area = 200;
    public static double recangle;
    Mat hsv = new Mat();
    Mat yellow_mask = new Mat();
    List<MatOfPoint> ctr_yellow = new ArrayList<>();
    @Override
public Mat processFrame(Mat input){
        ctr_yellow.clear();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsv, lower_yellow, upper_yellow, yellow_mask);
        Imgproc.findContours(yellow_mask, ctr_yellow, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        if (!ctr_yellow.isEmpty() && Imgproc.contourArea(ctr_yellow.get(0)) > min_detect_area) {
            RotatedRect rec = Imgproc.minAreaRect(new MatOfPoint2f(ctr_yellow.get(0).toArray()));
            Point[] boxPoints = new Point[4];
            rec.points(boxPoints);
            double angle = rec.angle;
            recangle = angle;
        }
        return yellow_mask;
    }
public double getAngle(){
    return recangle;
}

}

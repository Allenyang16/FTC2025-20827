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
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
@Config
public class DetectPipeline extends OpenCvPipeline{
    public static Scalar lower_yellow = new Scalar(10, 60, 50);
    public static Scalar upper_yellow = new Scalar(30, 255, 255);
    public static int min_detect_area = 200;
    public static double recangle;
    public static double recCenterX, recCenterY;
    public static double horizontalDistance, verticalDistance;
    Mat hsv = new Mat();
    Mat yellow_mask = new Mat();
    List<MatOfPoint> ctr_yellow = new ArrayList<>();
    @Override
public Mat processFrame(Mat input){
        ctr_yellow.clear();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, lower_yellow, upper_yellow, yellow_mask);
        Imgproc.findContours(yellow_mask, ctr_yellow, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        if (!ctr_yellow.isEmpty()) {
            // Find the largest contour

            List<MatOfPoint> filtered_contours_yellow = new ArrayList<>();
            for(MatOfPoint contour : ctr_yellow) {
                if (Imgproc.contourArea(contour) > min_detect_area) {
                    filtered_contours_yellow.add(contour);
                }
            }
            MatOfPoint largestContour = Collections.max(ctr_yellow, Comparator.comparingDouble(Imgproc::contourArea));
            RotatedRect rec = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));
            Point[] boxPoints = new Point[4];
            rec.points(boxPoints);
            Point recCenter = rec.center;
            recCenterX = recCenter.x;
            recCenterY = recCenter.y;

            double cameraCenterX = 320;
            double cameraCenterY = 240;

            horizontalDistance = recCenterX - cameraCenterX;
            verticalDistance = recCenterY - cameraCenterY;

// 计算第一条边的斜率
            double deltaY = boxPoints[1].y - boxPoints[0].y;
            double deltaX = boxPoints[1].x - boxPoints[0].x;
            double angle = Math.toDegrees(Math.atan2(deltaY, deltaX));

// 确保角度在 [0, 180) 范围内
            recangle = angle;
        }
        return yellow_mask;
    }
public double getAngle(){
    return recangle;
}
public double getPositionX(){
        return recCenterX;
    }
public double getPositionY(){
        return recCenterY;
    }

}

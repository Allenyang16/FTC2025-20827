package org.firstinspires.ftc.teamcode.testings;
import android.annotation.SuppressLint;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

public class cv extends OpenCvPipeline {
    // Define the lower and upper bounds for the blue color in HSV
    private int MIN_CONTOUR_AREA = 200;
    private final Scalar lower_blue = new Scalar(80, 120, 0);   // Lower bound for blue
    private final Scalar upper_blue = new Scalar(130, 255, 255); // Upper bound for blue
    private final Scalar lower_red = new Scalar(0, 100, 100);    // Lower bound for red
    private final Scalar upper_red = new Scalar(10, 255, 255);   // Upper bound for red
    private final Scalar lower_yellow = new Scalar(10, 60, 50);  // Lower bound for yellow
    private final Scalar upper_yellow = new Scalar(30, 255, 255); // Upper bound for yellow
    private double largestContourArea = 0; // Variable to store the largest contour area
    private double largestContourAngle = 0; // Variable to store the largest contour angle

    @Override
    public Mat processFrame(Mat input) {
        Mat frame = new Mat();
        Mat framed = input.clone();
        Mat hsv = new Mat();
        Mat blue_mask = new Mat();
        Mat blank = new Mat();
        Mat red_mask = new Mat();
        Mat yellow_mask = new Mat();
        Mat maskedFrame_blue = new Mat();
        Mat maskedFrame_red = new Mat();
        Mat maskedFrame_yellow = new Mat();
        Mat blur = new Mat();

        Mat hierarchy_blue = new Mat();
        Mat hierarchy_red = new Mat();
        Mat hierarchy_yellow = new Mat();




        ArrayList<MatOfPoint> ctr_blue = new ArrayList<>();
        ArrayList<MatOfPoint> ctr_red = new ArrayList<>();
        ArrayList<MatOfPoint> ctr_yellow = new ArrayList<>();

        Imgproc.bilateralFilter(input,blur,5, 15, 15);

        Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_BGR2HSV);

        Core.inRange(hsv, lower_blue, upper_blue, blue_mask);
        Core.inRange(hsv, lower_red, upper_red, red_mask);
        Core.inRange(hsv, lower_yellow, upper_yellow, yellow_mask);

        Core.bitwise_and(input, input, maskedFrame_blue, blue_mask);
        Core.bitwise_and(input, input, maskedFrame_red, red_mask);
        Core.bitwise_and(input, input, maskedFrame_yellow, yellow_mask);

        Imgproc.findContours(blue_mask, ctr_blue, hierarchy_blue, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(red_mask, ctr_red, hierarchy_red, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(yellow_mask, ctr_yellow, hierarchy_yellow, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // ... existing code ...



// ... existing code ...

        ArrayList<MatOfPoint> filtered_contours_side = new ArrayList<>();
        for(MatOfPoint contour : ctr_blue) {
            if (Imgproc.contourArea(contour) > MIN_CONTOUR_AREA) {
                filtered_contours_side.add(contour);
            }
        }

        ArrayList<MatOfPoint> filtered_contours_yellow = new ArrayList<>();
        for(MatOfPoint contour : ctr_yellow) {
            if (Imgproc.contourArea(contour) > MIN_CONTOUR_AREA) {
                filtered_contours_side.add(contour);
            }
        }

        if (!filtered_contours_side.isEmpty()) {
            MatOfPoint largestContour = Collections.max(ctr_blue, Comparator.comparingDouble(Imgproc::contourArea));
            largestContourArea = Imgproc.contourArea(largestContour); // Update the area

            // Convert MatOfPoint to MatOfPoint2f
            MatOfPoint2f largestContour2f = new MatOfPoint2f(largestContour.toArray());
            RotatedRect rect = Imgproc.minAreaRect(largestContour2f);
            Point[] boxPoints = new Point[4];
            rect.points(boxPoints);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(frame, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(0, 255, 0), 2);
            }

            // Calculate the center of the rectangle
            Point centerRect = rect.center;

            // Calculate the center of the frame
            Point centerFrame = new Point((double) frame.cols() / 2, (double) frame.rows() / 2);

            // Draw a line from the center of the rectangle to the center of the frame
            Imgproc.line(framed, centerRect, centerFrame, new Scalar(255, 0, 0), 2);  // Blue line

            double angle = rect.angle;
            Size size = rect.size;
            double area = Math.round(size.width * size.height * 100.0) / 100.0;

            // Position the text above the rectangle
            Point textPosition = new Point(centerRect.x, centerRect.y - 10);  // Adjust y-coordinate to place above
            @SuppressLint("DefaultLocale") String text = String.format("Angle: %.2f Area: %.2f", angle, area);
            Imgproc.putText(framed, text, textPosition, Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(255, 255, 255), 2);

            largestContourAngle = angle; // Update the angle
        }

        if (!filtered_contours_yellow.isEmpty()) {
            MatOfPoint largestContour = Collections.max(ctr_blue, Comparator.comparingDouble(Imgproc::contourArea));
            largestContourArea = Imgproc.contourArea(largestContour); // Update the area

            // Convert MatOfPoint to MatOfPoint2f
            MatOfPoint2f largestContour2f = new MatOfPoint2f(largestContour.toArray());
            RotatedRect rect = Imgproc.minAreaRect(largestContour2f);
            Point[] boxPoints = new Point[4];
            rect.points(boxPoints);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(frame, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(0, 255, 0), 2);
            }

            // Calculate the center of the rectangle
            Point centerRect = rect.center;

            // Calculate the center of the frame
            Point centerFrame = new Point((double) frame.cols() / 2, (double) frame.rows() / 2);

            // Draw a line from the center of the rectangle to the center of the frame
            Imgproc.line(framed, centerRect, centerFrame, new Scalar(255, 0, 0), 2);  // Blue line

            double angle = rect.angle;
            Size size = rect.size;
            double area = Math.round(size.width * size.height * 100.0) / 100.0;

            // Position the text above the rectangle
            Point textPosition = new Point(centerRect.x, centerRect.y - 10);  // Adjust y-coordinate to place above
            @SuppressLint("DefaultLocale") String text = String.format("Angle: %.2f Area: %.2f", angle, area);
            Imgproc.putText(framed, text, textPosition, Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(255, 255, 255), 2);

            largestContourAngle = angle; // Update the angle
        }




        // // Create a new Mat to hold the processed image
        // Mat output = new Mat();

        // // Convert the input image to HSV color space
        // Mat hsv = new Mat();
        // Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

        // // Create a mask for the blue color



        // // Apply the mask to get the blue regions
        // Mat blueRegions = new Mat();
        // Core.bitwise_and(input, input, blueRegions, mask);

        // // Convert the blue regions to grayscale for further processing
        // Imgproc.cvtColor(blueRegions, output, Imgproc.COLOR_BGR2GRAY);

        // // Apply Gaussian blur to reduce noise
        // Imgproc.GaussianBlur(output, output, new org.opencv.core.Size(5, 5), 0);

        // // Apply Canny edge detection
        // Imgproc.Canny(output, output, 100, 200);

        // // Return the processed image
         return frame;
    }

    // Getter for the largest contour area
    public double getLargestContourArea() {
        return largestContourArea;
    }

    // Getter for the largest contour angle
    public double getLargestContourAngle() {
        return largestContourAngle;
    }
}

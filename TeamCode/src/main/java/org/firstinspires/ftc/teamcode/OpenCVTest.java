package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.openftc.easyopencv.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

@TeleOp
public class OpenCVTest extends LinearOpMode {
    OpenCvWebcam webcam;

    class CustomPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Split channels (like your Python b,g,r = cv.split(frame))
            java.util.List<Mat> channels = new java.util.ArrayList<>();
            Core.split(input, channels);
            Mat blue = channels.get(0);  // Blue channel

            // Bilateral Filter
            Imgproc.bilateralFilter(blue, blue, 5, 15, 15);

            // Threshold
            Imgproc.threshold(blue, blue, 175, 255, Imgproc.THRESH_BINARY);

            // Canny
            Mat edges = new Mat();
            Imgproc.Canny(blue, edges, 50, 125);

            // Dilate
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));
            Imgproc.dilate(edges, edges, kernel, new Point(-1,-1), 3);

            // Convert back to RGB for display
            Imgproc.cvtColor(edges, input, Imgproc.COLOR_GRAY2RGB);
            
            return input;
        }
    }

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new CustomPipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                webcam.setFps(30);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
            }
        });

        waitForStart();
        
        while (opModeIsActive()) {
            telemetry.addData("FPS", webcam.getFps());
            telemetry.update();
        }
    }
} 
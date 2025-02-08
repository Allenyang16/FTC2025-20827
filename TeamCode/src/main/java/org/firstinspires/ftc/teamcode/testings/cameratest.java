package org.firstinspires.ftc.teamcode.testings;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@TeleOp(name = "Camera Test", group = "Testing")
@Config
public class cameratest extends LinearOpMode {
    OpenCvCamera camera;
    cv pipeline; // Use your cv class as the pipeline

    @Override
    public void runOpMode() {  
        // Get the camera monitor view ID
        // Initialize the camera
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera =OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        // Initialize your custom pipeline
        pipeline = new cv(); // Instantiate your cv pipeline
        camera.setPipeline(pipeline);

        // Open the camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming the camera feed
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        // Wait for the start button to be pressed
        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Access the largest contour area and angle
            double area = pipeline.getLargestContourArea();
            double angle = pipeline.getLargestContourAngle();

            // Use the area and angle as needed
            telemetry.addData("Largest Contour Area", area);
            telemetry.addData("Largest Contour Angle", angle);
            telemetry.update();
        }

        // Stop streaming when done
        camera.stopStreaming();
    }
}
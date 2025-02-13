package org.firstinspires.ftc.teamcode.testings;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;


@TeleOp(name = "Camera Test Area and Angle", group = "Testing")
@Config
public class cameratest extends LinearOpMode {
    OpenCvCamera camera;
    DetectPipeline pipeline; // Use your cv class as the pipeline

    @Override
    public void runOpMode() {  
        // Get the camera monitor view ID
        // Initialize the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new DetectPipeline();
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.setPipeline(pipeline);
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
            double angle = pipeline.getAngle();
            telemetry.addData("angle",angle);
            telemetry.addData("x",pipeline.getPositionX());
            telemetry.addData("y",pipeline.getPositionY());
            telemetry.update();
//            if(angle>= 60 && angle <= 90){
//                telemetry.addData("angle 1",angle);
//            }
//            else if(angle>= 0 && angle <= 30){
//                telemetry.addData("angle 2",angle);
//            }
//            else if (angle>= 30 && angle <= 60){
//                telemetry.addData("angle 3",angle);
//            }
        }

        // Stop streaming when done
        camera.stopStreaming();
    }
}
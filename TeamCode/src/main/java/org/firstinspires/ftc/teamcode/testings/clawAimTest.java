package org.firstinspires.ftc.teamcode.testings;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
//import org.firstinspires.ftc.teamcode.testings.DetectPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;
@Config
@TeleOp(name = "Claw Aim Test", group = "Testing")
public class clawAimTest extends LinearOpMode{
    private final Telemetry telemetryM = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    private double defaultValue = 0;
    private double angle = 0;
    private Vision vision;
    private double irlX = 0;
    private double irlY = 0;
    private double cameraOffset = 0;
    private double specimenPositionRelativeToClaw;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo aim = hardwareMap.get(Servo.class,"aim");
        vision = new Vision(hardwareMap, telemetryM);
        vision.initializeCamera();
        waitForStart();
        while(opModeIsActive()){
            vision.updateFrame();
            if(vision.isTargetVisible()){
                vision.turnClaw(defaultValue);
                telemetry.addData("irlX",vision.getSamplePosition(defaultValue,0));
                telemetry.addData("irlY",vision.getSamplePosition(defaultValue,1));
            }
//
            telemetry.update();
        }
    }
}





//    angle = vision.getSampleAngle(defaultValue);
//                telemetry.addData("angle", angle);
//                if(angle != 0){
//                    //竖直
//                    if((angle<= -70 && angle >= -90.0) || (angle >= 70 && angle <= 90)){
////                        aim.setPosition(0.20);
//                        aim.setPosition(0.27);
//                    }
//                    //顺时针转锐角
//                    else if (angle >= -70 && angle <= -20){
////                        aim.setPosition(0.00);
//                        aim.setPosition(0.55);
//                    }
//                    //水平
//                    else if (angle >=-20 && angle <= 20){
////                        aim.setPosition(0.52);
//                    }
//                    //顺时针转钝角
//                    else if (angle >=20 && angle <= 70){
////                        aim.setPosition(0.35);
//                        aim.setPosition(0.27);
//                    }
//                }
//                irlX = vision.getSamplePosition(defaultValue,0);
//                specimenPositionRelativeToClaw = irlX - cameraOffset;
//
//
//
//                irlY = vision.getSamplePosition(defaultValue,1);
//
//            }
//            else{
//                telemetry.addData("Nodata avalible","no");
//            }
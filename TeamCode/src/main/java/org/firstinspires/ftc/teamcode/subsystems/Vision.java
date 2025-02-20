package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Config
public class Vision {
    private final Limelight3A limelight;
    private final Servo aim;
    private LLResult result;
    private double cameraOffset = 6.01574803;
    private double slideOffset = 10;

    private double inchToSlideNumber = 70;

    private double yPixelToInch = 8.82;
    private double pixelToInch = 0.24;
//
//    public static double ledPWM = 0.5;
//
//
//    public static double CAMERA_HEIGHT = 307.0;
//    public static double CAMERA_ANGLE = -45.0;
//    public static double TARGET_HEIGHT = 19.05;
//
//    public static double strafeConversionFactor = 6.6667;
//    public static double cameraStrafeToBot = -20;
//
//    public static double sampleToRobotDistance = 145;

    Telemetry telemetry;

    private static double clamp(double value, double min, double max) {
        if (value < min) {
            return min;
        } else if (value > max) {
            return max;
        } else {
            return value;
        }
    }

    public Vision(final HardwareMap hardwareMap, Telemetry telemetry) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        aim = hardwareMap.get(Servo.class,"spinWrist");
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }


    public void initializeCamera() {
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    public boolean isTargetVisible() {
        return (result != null);
    }

//    public double getSampleAngle(double defaultValue){
//        if (result != null) {
//            double height = 15;
//            double irlAngle = defaultValue;
//            double irlAngleDegree = defaultValue;
//            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
//            for (LLResultTypes.ColorResult cr : colorResults) {
//                List<List<Double>> corners = cr.getTargetCorners();
//                double angleRadians = 0;
//                if (corners.size() >= 4) {
//                    // 获取角点 0 和角点 1 的坐标
//                    double x0 = corners.get(0).get(0);
//                    double y0 = corners.get(0).get(1);
//                    double x1 = corners.get(1).get(0);
//                    double y1 = corners.get(1).get(1);
//                    double x3 = corners.get(3).get(0);
//                    double y3 = corners.get(3).get(1);
//
//                    double length01 = Math.sqrt(Math.pow(x1 - x0, 2) + Math.pow(y1 - y0, 2));
//
//                    // 计算 corner0 到 corner3 的距离
//                    double length03 = Math.sqrt(Math.pow(x3 - x0, 2) + Math.pow(y3 - y0, 2));
//                    double deltaY;
//                    double deltaX;
//                    // 计算角点 0 和角点 1 形成的直线的角度
//                    if (length01 > length03) {
//                        deltaX = x1 - x0;
//                        deltaY = y1 - y0;
//                    } else {
//                        deltaX = x3 - x0;
//                        deltaY = y3 - y0;
//                    }
//
//                    angleRadians = Math.atan2(deltaY, deltaX);
//
//                }
//                irlAngle = Math.atan(Math.tan(angleRadians));
//                irlAngleDegree = Math.toDegrees(irlAngle);
//
//            }
//
//            return irlAngleDegree;
//        } else {
//            return defaultValue;
//        }
//    }

    public double getSamplePosition(double defaultValue,int resultType){

        double irlY = defaultValue;
        double irlX = defaultValue;
        if (result != null) {
            double tX = result.getTx();
            double tY = result.getTy();
            //像素转英寸
            irlY = ((Math.tan(Math.toRadians(tX + 45)) * yPixelToInch)-slideOffset)*inchToSlideNumber;
            irlX = tY * pixelToInch - cameraOffset;
            if(resultType == 0){
                return irlX;
            }
            else{
                return irlY;
            }
        }
        else{
            return defaultValue;
        }
    }

    public void turnClaw(double defaultValue){
        if (result != null) {
            double irlAngle = defaultValue;
            double irlAngleDegree = defaultValue;
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            for (LLResultTypes.ColorResult cr : colorResults) {
                List<List<Double>> corners = cr.getTargetCorners();
                double angleRadians = 0;
                if (corners.size() >= 4) {
                    
                    double x0 = corners.get(0).get(0);
                    double y0 = corners.get(0).get(1);
                    double x1 = corners.get(1).get(0);
                    double y1 = corners.get(1).get(1);
                    double x3 = corners.get(3).get(0);
                    double y3 = corners.get(3).get(1);

                    double length01 = Math.sqrt(Math.pow(x1 - x0, 2) + Math.pow(y1 - y0, 2));

                    
                    double length03 = Math.sqrt(Math.pow(x3 - x0, 2) + Math.pow(y3 - y0, 2));
                    double deltaY;
                    double deltaX;
                
                    if (length01 > length03) {
                        deltaX = x1 - x0;
                        deltaY = y1 - y0;
                    } else {
                        deltaX = x3 - x0;
                        deltaY = y3 - y0;
                    }

                    angleRadians = Math.atan2(deltaY, deltaX);

                }
                irlAngle = Math.atan(Math.tan(angleRadians)*0.707);
                irlAngleDegree = Math.toDegrees(irlAngle);
                if(irlAngleDegree != defaultValue){
                    telemetry.addData("angle",irlAngleDegree);
                    //水平
                    if((irlAngleDegree<= -60 && irlAngleDegree >= -90.0) || (irlAngleDegree >= 67 && irlAngleDegree <= 90))   {
//                        aim.setPosition(0.52);
                        aim.setPosition(0.55);
                    }
                    //顺时针转钝角
                    else if (irlAngleDegree >= -65 && irlAngleDegree <= -20){
//                        aim.setPosition(0.35);
                        aim.setPosition(0.41);
                    }
                    //竖直
                    else if (irlAngleDegree >=-20 && irlAngleDegree <= 25){
//                        aim.setPosition(0.20);
                        aim.setPosition(0.27);
                    }
                    //顺时针转锐角
                    else if (irlAngleDegree >=30 && irlAngleDegree <= 67){
//                        aim.setPosition(0.00);
                        aim.setPosition(0.13);
                    }
                }

            }

        } else {

        }




    }
    public void updateFrame(){
        result = limelight.getLatestResult();
    }

}

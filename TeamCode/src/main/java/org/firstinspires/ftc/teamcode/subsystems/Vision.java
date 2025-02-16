package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
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
    private LLResult result;
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

    public Vision(final HardwareMap hardwareMap, Telemetry telemetry) {
         limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }


    public void initializeCamera() {
        limelight.start();
    }
    public double getSampleAngle(double defaultValue){
        if (result != null) {
            double height = 15;
            double irlAngle = defaultValue;
            double irlAngleDegree = defaultValue;
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            for (LLResultTypes.ColorResult cr : colorResults) {
//                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXPixels(), cr.getTargetYPixels());
//                    String result_corners = cr.getTargetCorners().stream()
//                            .map(subList -> subList.stream()
//                                    .map(String::valueOf) // 将 Double 转换为 String
//                                    .collect(Collectors.joining(", "))) // 子列表内元素拼接
//                            .collect(Collectors.joining("; "));
//                    telemetry.addData("Color_corner", result_corners);
//                    telemetry.addData("Color_angle: ",cr.getTargetPoseRobotSpace().toString());
                List<List<Double>> corners = cr.getTargetCorners();
                double angleRadians = 0;
                if (corners.size() >= 4) {
                    // 获取角点 0 和角点 1 的坐标
                    double x0 = corners.get(0).get(0);
                    double y0 = corners.get(0).get(1);
                    double x1 = corners.get(1).get(0);
                    double y1 = corners.get(1).get(1);
                    double x3 = corners.get(3).get(0);
                    double y3 = corners.get(3).get(1);

                    double length01 = Math.sqrt(Math.pow(x1 - x0, 2) + Math.pow(y1 - y0, 2));

                    // 计算 corner0 到 corner3 的距离
                    double length03 = Math.sqrt(Math.pow(x3 - x0, 2) + Math.pow(y3 - y0, 2));
                    double deltaY;
                    double deltaX;
                    // 计算角点 0 和角点 1 形成的直线的角度
                    if (length01 > length03) {
                        deltaX = x1 - x0;
                        deltaY = y1 - y0;
                    } else {
                        deltaX = x3 - x0;
                        deltaY = y3 - y0;
                    }

                    angleRadians = Math.atan2(deltaY, deltaX);

                    // 输出到遥测
                    telemetry.addData("Corner 0", "X: %.2f, Y: %.2f", x0, y0);
                    telemetry.addData("Corner 1", "X: %.2f, Y: %.2f", x1, y1);

                }
                irlAngle = Math.atan(Math.tan(angleRadians) * 0.707);
                irlAngleDegree = Math.toDegrees(irlAngle);
                telemetry.addData("irl angle", irlAngleDegree);

            }
            double irlY;
            double irlX;
            double tX = result.getTx();
            double tY = result.getTy();
            irlY = Math.tan(Math.toRadians(tX + 45)) * height;
            irlX = Math.tan(Math.toRadians(tY)) * height;
            telemetry.addData("inreallifeY",irlY);
            telemetry.addData("inreallifeX",irlX);
            return irlAngleDegree;
        } else {
            return defaultValue;
        }
    }

    public double getSamplePosition(double defaultValue,int resultType){
        double height = 15;
        double irlY = defaultValue;
        double irlX = defaultValue;
        if (result != null) {
            double tX = result.getTx();
            double tY = result.getTy();
            irlY = Math.tan(Math.toRadians(tX + 45)) * height;
            irlX = Math.tan(Math.toRadians(tY)) * height;
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
    public void updateFrame(){
        result = limelight.getLatestResult();
    }

}
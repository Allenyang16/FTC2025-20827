package org.firstinspires.ftc.teamcode.testings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import java.util.stream.Collectors;

@TeleOp(name = "Limelight Test", group = "Testing")
@Config
public class LlPipeline extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null) {

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
                        if(length01> length03){
                            deltaX = x1 - x0;
                            deltaY = y1 - y0;
                        }
                        else{
                            deltaX = x3 - x0;
                            deltaY = y3 - y0;
                        }

                        angleRadians = Math.atan2(deltaY, deltaX); // 弧度值
                        double angleDegrees = Math.toDegrees(angleRadians); // 转换为角度

                        // 输出到遥测
                        telemetry.addData("Corner 0", "X: %.2f, Y: %.2f", x0, y0);
                        telemetry.addData("Corner 1", "X: %.2f, Y: %.2f", x1, y1);
                        telemetry.addData("Line Angle", "%.2f degrees", angleDegrees);

                    }
                    double tX = result.getTx();
                    double tY = result.getTy();
                    double height = 15;
                    double irlY;
                    double irlX;
                    irlY = Math.tan(Math.toRadians(tX + 45)) * height;
                    irlX = Math.tan(Math.toRadians(tY)) * height;
                    telemetry.addData("inreallifeY",irlY);
                    telemetry.addData("inreallifeX",irlX);
                    double irlAngle = Math.atan(Math.tan(angleRadians)*0.707);
                    double irlAngleDegree = Math.toDegrees(irlAngle);
                    telemetry.addData("irl angle",irlAngleDegree);


                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();
        }
        limelight.stop();
    }
}

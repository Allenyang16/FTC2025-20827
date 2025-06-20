/*package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;




public class Vision extends LinearOpMode{
    private Limelight3A limelight;
    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        limelight.pipelineSwitch(0);

        telemetry.addData(">","Ready");
        telemetry.update();
        waitForStart();

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()){
            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        // Sending numbers to Python
        double[] inputs = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
        limelight.updatePythonInputs(inputs);

        // Getting numbers from Python
        double[] pythonOutputs = result.getPythonOutput();
        if (pythonOutputs != null && pythonOutputs.length > 0) {
            double firstOutput = pythonOutputs[0];
            telemetry.addData("Python output:", firstOutput);
        }

        //Position of the robot
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double currentX = botpose.getPosition().x;
                double currentY = botpose.getPosition().y;
                telemetry.addData("Location", "(" + currentX + ", " + currentY + ")");


            }
        }


        long staleness = result.getStaleness();
        if (staleness < 100) { // Less than 100 milliseconds old
            telemetry.addData("Data", "Good");
        } else {
            telemetry.addData("Data", "Old (" + staleness + " ms)");
        }
    }
}
*/

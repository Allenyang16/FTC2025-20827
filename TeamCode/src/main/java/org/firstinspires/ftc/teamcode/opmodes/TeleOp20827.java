package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.XCYBoolean;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

@TeleOp
public class TeleOp20827 extends LinearOpMode {
    Runnable update;
    @Override
    public void runOpMode() throws InterruptedException {
        SuperStructure superStructure = new SuperStructure(this);
        NewMecanumDrive drive = new NewMecanumDrive();
        XCYBoolean intakeFar = new XCYBoolean(()-> gamepad1.a);

        update = ()->{
            drive.update();
            superStructure.update();
            XCYBoolean.bulkRead();
        };

        waitForStart();
        while (opModeIsActive()){
            if(intakeFar.toTrue()){

            }
        }
    }
}

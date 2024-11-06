package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.XCYBoolean;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;

@TeleOp
public class TeleOpTest extends LinearOpMode {
    Runnable update;

    @Override
    public void runOpMode() throws InterruptedException {
        SuperStructure upper = new SuperStructure(this);
        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap);

        update = ()->{
            drive.update();
            upper.update();
            XCYBoolean.bulkRead();
        };

        XCYBoolean intakeFar = new XCYBoolean(()-> gamepad1.y);
        XCYBoolean intakeNear = new XCYBoolean(()-> gamepad1.a);
        XCYBoolean grab = new XCYBoolean(()-> gamepad1.right_bumper);
        XCYBoolean toOrigin = new XCYBoolean(()-> gamepad1.left_stick_button);
        XCYBoolean toHighRelease = new XCYBoolean(()-> gamepad1.dpad_up);

        upper.initialize();
        waitForStart();

        while (opModeIsActive()){

            if(intakeFar.toTrue()){
                upper.setArmPosition(SuperStructure.ARM_INTAKE);
                delay(1000);
                upper.setSlidePosition(SuperStructure.SLIDE_INTAKE_MAX);
                // miniArm
                upper.setWristIntake();
            }

            if(intakeNear.toTrue()){
                upper.setArmPosition(SuperStructure.ARM_INTAKE);
                delay(1000);
                upper.setSlidePosition(SuperStructure.SLIDE_MIN);
                delay(500);
            }

            if(grab.toTrue()){
                upper.switchClawState();
            }

            if(toOrigin.toTrue()){
                upper.setSlidePosition(SuperStructure.SLIDE_MIN);
                delay(1000);
                upper.setArmPosition(SuperStructure.ARM_RELEASE);
            }

            if(toHighRelease.toTrue()){
                upper.setSlidePosition(SuperStructure.SLIDE_BOX_HIGH);
            }

            update.run();
        }
    }

    protected void delay(int millisecond) {
        long end = System.currentTimeMillis() + millisecond;
        while (opModeIsActive() && end > System.currentTimeMillis() && update!=null) {
            idle();
            update.run();
        }
    }
}

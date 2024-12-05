package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class AutoMove extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        side_color = RED;
        startSide = POSITIVE;

        initHardware();
        resetPoseforTest();
        while (opModeInInit()){

        }
        if(opModeIsActive()){
            strafe();


        }
    }
}

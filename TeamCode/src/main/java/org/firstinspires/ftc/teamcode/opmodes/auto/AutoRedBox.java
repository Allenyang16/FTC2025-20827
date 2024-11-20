package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class AutoRedBox extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        side_color = RED;
        startSide = NEGATIVE;

        // TODO: Check the init of positions
        initHardware();

        while(opModeInInit()){

        }
        moveToDropFirst_sample();
        toOrigin();

        intakeSample_1();
        moveToDrop_sample();
        toOrigin();

        // TODO: Test the values later
        intakeSample_2();
        moveToDrop_sample();
        toOrigin();

        intakeSample_3();
        moveToDrop_sample();
        toOrigin();
        moveToStartPos();
    }
}

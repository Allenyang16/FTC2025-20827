package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
@Disabled
public class AutoBlueChamber extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        side_color = BLUE;
        startSide = NEGATIVE;

        initHardware();
        while (opModeInInit()){

        }

        if(opModeIsActive()){
            releaseSpecimen(1);
            dropSpecimen_toOrigin();

            intakeSample(1);
            dropSampleToHP();
            intakeSpecimen();
            releaseSpecimen(2);
            dropSpecimen_toOrigin();

            intakeSample(2);
            dropSampleToHP();
            intakeSpecimen();
            releaseSpecimen(1);
            dropSpecimen_toOrigin();

            intakeSample_3();
            dropSampleToHP();
            intakeSpecimen();
            releaseSpecimen(2);
            dropSpecimen_toOrigin();
            park_observation();
        }
    }
}

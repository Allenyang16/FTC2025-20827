package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class AutoRedChamber extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        side_color = RED;
        startSide = POSITIVE;

        initHardware();
        while (opModeInInit()){

        }

        if(opModeIsActive()){
            releaseSpecimen();
            dropSpecimen_toOrigin();

            intakeSample_1();
            dropSampleToHP();
            intakeSpecimen();
            releaseSpecimen();
            dropSpecimen_toOrigin();

            intakeSample_2();
            dropSampleToHP();
            intakeSpecimen();
            releaseSpecimen();
            dropSpecimen_toOrigin();

            intakeSample_3();
            dropSampleToHP();
            intakeSpecimen();
            releaseSpecimen();
            dropSpecimen_toOrigin();
        }
    }
}

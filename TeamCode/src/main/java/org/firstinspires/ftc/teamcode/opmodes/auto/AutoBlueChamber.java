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
            autoUpperToOrigin();
            dropSpecimen_toOrigin();

            intakeSample(1);
            dropSampleToHP();
            intakeSpecimen(1);
            autoUpperToOrigin();
            dropSpecimen_toOrigin();

            intakeSample(2);
            dropSampleToHP();
            intakeSpecimen(2);
            autoUpperToOrigin();
            dropSpecimen_toOrigin();

            intakeSample_3();
            dropSampleToHP();
            intakeSpecimen(3);
            autoUpperToOrigin();
            dropSpecimen_toOrigin();
            park_observation();
        }
    }
}

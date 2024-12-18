package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class AutoRedChamber_pushSample extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        side_color = RED;
        startSide = POSITIVE;

        initHardware();
        while (opModeInInit()){

        }

        if(opModeIsActive()){
            releaseSpecimen(1);
            pushSample();

            intakeSpecimen();
            releaseSpecimen(2);

            dropSpecimen_toIntakeSpecimen();
            releaseSpecimen(3);

            dropSpecimen_toIntakeSpecimen();
            releaseSpecimen(4);

            park_observation();
        }
    }
}

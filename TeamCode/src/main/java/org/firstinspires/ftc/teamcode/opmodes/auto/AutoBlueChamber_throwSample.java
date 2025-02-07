package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoMaster;

@Autonomous
public class AutoBlueChamber_throwSample extends AutoMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        side_color = BLUE;
        startSide = NEGATIVE;

        initHardware();
        while (opModeInInit()){

        }

        if(opModeIsActive()){
            moveToChamber(1);
            //releaseSpecimen();

            grabSample(1);
            grabSample(2);
            //grabSample(3);

            intakeSpecimen(1);
            moveToChamber(2);
            autoUpperToOrigin();

            dropSpecimen_toIntakeSpecimen(3);
            moveToChamber(3);
            autoUpperToOrigin();

            dropSpecimen_toIntakeSpecimen(4);
            moveToChamber(4);
            autoUpperToOrigin();

            //dropSpecimen_toIntakeSpecimen(5);
            //moveToPreChamber(5);
            //releaseSpecimen();

            park_observation();
        }
    }
}

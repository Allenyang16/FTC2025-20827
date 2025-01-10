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
        waitForStart();

        if(isStopRequested()) return;

        moveToPreChamber(1);
        releaseSpecimen(1);

        grabSample(1);
        delay(100);
        grabSample(2);
        //intake_release_RedSample();

        intakeSpecimen();
        moveToPreChamber(2);
        releaseSpecimen(2);

        dropSpecimen_toIntakeSpecimen(3);
        moveToPreChamber(3);
        releaseSpecimen(3);

        dropSpecimen_toIntakeSpecimen(4);
        moveToPreChamber(4);
        releaseSpecimen(4);

        dropSpecimen_toIntakeSpecimen(5);
        moveToPreChamber(5);
        releaseSpecimen(5);
        park();
    }
}

package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slide extends SubsystemBase {
    private DcMotorEx mSlideLeft = null;
    private DcMotorEx mSlideRight = null;
    private final PIDController pidController;
    private final PIDController leftSlideController;

    public Slide (final HardwareMap hardwareMap){
        mSlideLeft = hardwareMap.get(DcMotorEx.class,"slideLeft");
        mSlideRight = hardwareMap.get(DcMotorEx.class,"slideRight");

        mSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        mSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pidController = new PIDController(0,0,0);
        leftSlideController = new PIDController(0,0,0);
    }

    public void setSlidePosition(){

    }

    @Override
    public void periodic(){

    }

}

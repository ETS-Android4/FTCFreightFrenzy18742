package org.firstinspires.ftc.teamcode.Activities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DuckSpinner {

    private DcMotor upDrive = null;
    private LinearOpMode linearOpMode = null;

    public void init(LinearOpMode linearOpMode) {
        upDrive = linearOpMode.hardwareMap.get(DcMotor.class, "Duck");
        upDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setMotorPower(double power) {
        upDrive.setPower(power);
    }

    public void duck_spin_final(boolean u){

    }

}

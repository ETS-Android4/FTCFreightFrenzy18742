package org.firstinspires.ftc.teamcode.Activities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DuckSpinner {
    private LinearOpMode linearOpMode = null;
    private DcMotor upDrive = null;

    public void init(LinearOpMode linearOpMode2) {
        DcMotor dcMotor = (DcMotor) linearOpMode2.hardwareMap.get(DcMotor.class, "Duck");
        this.upDrive = dcMotor;
        dcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setMotorPower(double power) {
        this.upDrive.setPower(power);
    }

    public void duck_spin_final(boolean u) {
    }
}

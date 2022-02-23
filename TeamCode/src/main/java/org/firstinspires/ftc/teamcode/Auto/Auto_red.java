package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Activities.MotorBase;

@Autonomous
public class Auto_red extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private MotorBase motorBase = new MotorBase();

    @Override
    public void runOpMode() {
        motorBase.init(this);
        waitForStart();

        //motorBase.goToPosition(0.0,0);
        motorBase.goToPosition(0,50.0);
        motorBase.goToPosition(-80,0);
    }
}

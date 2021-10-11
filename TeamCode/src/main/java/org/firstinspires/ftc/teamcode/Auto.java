package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class Auto extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private MotorBase motorBase = new MotorBase();

    @Override
    public void runOpMode() {
        motorBase.motor_init(this);
        waitForStart();
        while (motorBase.getForwardDistants()<100 && opModeIsActive()) {
motorBase.move(1,0,0);
        }
    }
}

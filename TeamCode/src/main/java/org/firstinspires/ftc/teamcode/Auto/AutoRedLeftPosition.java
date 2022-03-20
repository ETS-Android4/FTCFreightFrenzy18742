package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Activities.AutoPlus;
import org.firstinspires.ftc.teamcode.Activities.Bucket;
import org.firstinspires.ftc.teamcode.Activities.Intake;
import org.firstinspires.ftc.teamcode.Activities.MotorBase;
import org.firstinspires.ftc.teamcode.Activities.Slide;

@Autonomous
public class AutoRedLeftPosition extends LinearOpMode {

    private Slide slide = new Slide();
    private Bucket bucket = new Bucket();
    private Intake intake = new Intake();
    private ElapsedTime runtime = new ElapsedTime();
    private MotorBase motorBase = new MotorBase();
    private AutoPlus autoPlus = new AutoPlus();

    @Override
    public void runOpMode() {
        telemetry = FtcDashboard.getInstance().getTelemetry();
        motorBase.init(this);
        slide.init(this);
        intake.init(this);
        bucket.init(this);
        autoPlus.init(this);
        waitForStart();
        AutoST();
    }

    public void AutoST(){
        autoPlus.setServoAPosition(false); sleep(1000);
        motorBase.goToPosition(50,50); autoPlus.setServoAPosition(true); sleep(1000);
    }
}

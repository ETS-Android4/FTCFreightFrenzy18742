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
import org.firstinspires.ftc.teamcode.Auto.AutoBlueLeftPosition;

@Autonomous
public class AutoTest extends LinearOpMode {

    private Slide slide = new Slide();
    private Bucket bucket = new Bucket();
    private Intake intake = new Intake();
    private ElapsedTime runtime = new ElapsedTime();
    private MotorBase motorBase = new MotorBase();
    private AutoPlus autoPlus = new AutoPlus();
    public double i = 0, a = 0;

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
        runtime.reset();
        motorBase.move(0.3,0,0); sleep(1800);
        motorBase.TurnP(70);
        motorBase.move(0.3,0,0); sleep(1000);
        motorBase.move(0,0,0);
    }

    public void ledRise(boolean d){
        if(d){ while(i<61){ autoPlus.ledPlus.setPower(1.0/60.0*i); i+=a; a++; sleep(100);} }
        else{ while(i>0){ autoPlus.ledPlus.setPower(1.0/60.0*i); i-=a; a++; sleep(100);} }
    }
}

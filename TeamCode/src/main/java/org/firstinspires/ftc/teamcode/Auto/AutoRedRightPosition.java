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
import org.firstinspires.ftc.teamcode.OpenCV.ArucoDetect;
import org.firstinspires.ftc.teamcode.OpenCV.FreightPosition;

@Autonomous
public class AutoRedRightPosition extends LinearOpMode {

    private Slide slide = new Slide();
    private Bucket bucket = new Bucket();
    private Intake intake = new Intake();
    private ElapsedTime runtime = new ElapsedTime();
    private MotorBase motorBase = new MotorBase();
    private AutoPlus autoPlus = new AutoPlus();
    public double i = 0, a = 0;
    private final ArucoDetect arucoDetect = new ArucoDetect();
    private FreightPosition variation;

    @Override
    public void runOpMode() {
        telemetry = FtcDashboard.getInstance().getTelemetry();
        arucoDetect.initialize(this);
        motorBase.init(this);
        slide.init(this);
        intake.init(this);
        bucket.init(this);
        autoPlus.init(this);
        variation = arucoDetect.forceGetPosition(); sleep(1500);
        arucoDetect.stopCamera();
        telemetry.addData("Variation", variation);
        telemetry.update();
        waitForStart();
        telemetry.update(); sleep(1500);
        AutoST();
    }

    public void AutoST(){
        runtime.reset();
        ledRise(true);
        switch (variation){
            case LEFT:
                break;
            case CENTER:
                break;
            case RIGHT:
                autoPlus.setServoAPosition(false); autoPlus.setServoBPosition(true); sleep(500);
                motorBase.goToPosition(0,-36);
                motorBase.TurnP(-40);
                autoPlus.setServoAPosition(true); sleep(1000);
                autoPlus.setServoBPosition(false); sleep(1000);
                autoPlus.setServoAPosition(false); autoPlus.setServoBPosition(true); sleep(500);
                motorBase.TurnP(0);
                //motorBase.goToPosition(0,38);
                //motorBase.TurnP(-90);
                //motorBase.goToPosition(10,0);
                //motorBase.goToPosition(0,50);
                break;
            case UNKNOWN:
                break;
        }
        ledRise(false);
    }

    public void ledRise(boolean d){
        if(d){ while(i<61){ autoPlus.ledPlus.setPower(1.0/60.0*i); i+=a; a++; sleep(100);} }
        else{ while(i>0){ autoPlus.ledPlus.setPower(1.0/60.0*i); i-=a; a++; sleep(100);} }
    }
}

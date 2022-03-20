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
import org.firstinspires.ftc.teamcode.OpenCV.ArucoDetect.*;
import org.firstinspires.ftc.teamcode.OpenCV.FreightPosition;

@Autonomous
public class AutoBlueLeftPosition extends LinearOpMode {

    private Slide slide = new Slide();
    private Bucket bucket = new Bucket();
    private Intake intake = new Intake();
    private ElapsedTime runtime = new ElapsedTime();
    private MotorBase motorBase = new MotorBase();
    private AutoPlus autoPlus = new AutoPlus();
    public double i = 0, a = 0;
    private final ArucoDetect arucoDetect = new ArucoDetect();
    private FreightPosition variation;
    public double[] distances = {20, 25, 38};
    public double[] angles = {25, 30, 35};
    private int variationB = 0;

    @Override
    public void runOpMode() {
        arucoDetect.initialize(this);
        motorBase.init(this);
        slide.init(this);
        intake.init(this);
        bucket.init(this);
        autoPlus.init(this);
        // - OpenCV -
        variation = arucoDetect.forceGetPosition();
        switch (variation) {
            case LEFT:
                variationB = 0;
                break;
            case CENTER:
                variationB = 1;
                break;
            case RIGHT:
                variationB = 2;
                break;
        }
        telemetry.addData("Pos", arucoDetect.forceGetPosition());
        telemetry.addData("Variation", variation);
        telemetry.addData("odasfawes", arucoDetect.timePosition);
        telemetry.update();
        // - Action -
        waitForStart();
        arucoDetect.stopCamera();
        AutoST();
    }

    public void AutoST() {
        runtime.reset();
        ledRise(true);
        autoPlus.setServoAPosition(false); autoPlus.setServoBPosition(true); sleep(500);
        motorBase.goToPosition(0, -distances[variationB]); motorBase.TurnP(angles[variationB]);
        autoPlus.setServoAPosition(true); sleep(500);
        autoPlus.setServoBPosition(false); sleep(500);
        autoPlus.setServoAPosition(false); autoPlus.setServoBPosition(true); sleep(500);
        motorBase.TurnP(0);
        motorBase.goToPosition(0,distances[variationB]);
        //motorBase.TurnP(90);
        //motorBase.goToPosition(-10,0);
        //motorBase.goToPosition(0,50);
        ledRise(false);
    }


    public void ledRise(boolean d) {
        if (d) {
            while (i < 61) {
                autoPlus.ledPlus.setPower(1.0 / 60.0 * i);
                i += a;
                a++;
                sleep(100);
            }
        } else {
            while (i > 0) {
                autoPlus.ledPlus.setPower(1.0 / 60.0 * i);
                i -= a;
                a++;
                sleep(100);
            }
        }
    }
}
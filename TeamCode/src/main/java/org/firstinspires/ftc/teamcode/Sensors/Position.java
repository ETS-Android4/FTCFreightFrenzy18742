package org.firstinspires.ftc.teamcode.Sensors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class Position {

    private LinearOpMode linearOpMode = null;
    private DcMotor position_x_left = null;
    private DcMotor position_x_right = null;
    private DcMotor position_y = null;

    private double PointsPerWheelRotation =  8192;
    private double CmPerRotation = 10;
    private double CmPerRobotRotation = 40;

    public void init(LinearOpMode linearOpMode){
        position_x_left = linearOpMode.hardwareMap.get(DcMotor.class,"x_left");
        position_x_right = linearOpMode.hardwareMap.get(DcMotor.class,"x_right");
        position_y = linearOpMode.hardwareMap.get(DcMotor.class,"y");
        position_x_left.setDirection(DcMotorSimple.Direction.FORWARD);
        position_x_right.setDirection(DcMotorSimple.Direction.FORWARD);
        position_y.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public double position_x(){ return (position_x_right.getCurrentPosition() + position_x_left.getCurrentPosition())/2/PointsPerWheelRotation * CmPerRotation; }
    public double position_y(){ return (position_y.getCurrentPosition())/ PointsPerWheelRotation * CmPerRotation; }
    public double position_angle(){ return (position_x_left.getCurrentPosition() - position_x_right.getCurrentPosition())/2/CmPerRotation*360; }

}
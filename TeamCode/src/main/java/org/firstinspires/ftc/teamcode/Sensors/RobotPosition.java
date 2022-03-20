package org.firstinspires.ftc.teamcode.Sensors;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class RobotPosition {

    private LinearOpMode linearOpMode = null;
    /*private DcMotor position_y_left = null;
    private DcMotor position_y_right = null;
    private DcMotor position_x = null;*/

    private double PointsPerWheelRotation =  8192;
    private double CmPerWheelRotation = 4.8 * Math.PI;
    private double CmPerRobotRotation = 40 * Math.PI;

    public void init(LinearOpMode linearOpMode){
        /*position_y_left = linearOpMode.hardwareMap.get(DcMotor.class,"x_left");
        position_y_right = linearOpMode.hardwareMap.get(DcMotor.class,"x_right");
        position_x = linearOpMode.hardwareMap.get(DcMotor.class,"y");
        position_y_left.setDirection(DcMotorSimple.Direction.FORWARD);
        position_y_right.setDirection(DcMotorSimple.Direction.FORWARD);
        position_x.setDirection(DcMotorSimple.Direction.FORWARD);*/
    }

    /*public double positionX(){ return (position_y_right.getCurrentPosition() + position_y_left.getCurrentPosition())/2.0/PointsPerWheelRotation * CmPerWheelRotation; }
    public double positionY(){ return (position_x.getCurrentPosition())/ PointsPerWheelRotation * CmPerWheelRotation; }
    public double positionAngleDegrees(){ return (position_y_left.getCurrentPosition() - position_y_right.getCurrentPosition())/2.0/PointsPerWheelRotation*CmPerWheelRotation/CmPerRobotRotation*360; }
    public double positionAngleRadians(){ return (position_y_left.getCurrentPosition() - position_y_right.getCurrentPosition())/PointsPerWheelRotation*CmPerWheelRotation/CmPerRobotRotation; }*/
    }
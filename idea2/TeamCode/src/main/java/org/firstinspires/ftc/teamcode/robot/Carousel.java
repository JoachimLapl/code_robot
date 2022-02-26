package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Carousel {
    private Telemetry telemetry;
    private ElapsedTime runtime;
    private double lastRuntime = 0.0;
    private DcMotorEx CarouselMotor;

    private double velocity;
    public static double maxVelocity = 2e3;

    public Carousel(Telemetry globalTelemetry, ElapsedTime globalRuntime, HardwareMap hardwareMap) {
        telemetry = globalTelemetry;
        runtime = globalRuntime;
        CarouselMotor = hardwareMap.get(DcMotorEx.class, "carousel_motor");
        CarouselMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        CarouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void reset(){velocity=0;}
    public void maxSpeed(){velocity=maxVelocity;}
    public void increaseSpeed(double speed){velocity=Math.min(velocity+100, maxVelocity);}
    public void increaseSpeed(){increaseSpeed(100);
    }
    public void apply(){
        //telemetry.addData("speed real", CarouselMotor.getVelocity());
        //telemetry.addData("power real", CarouselMotor.getPower());
        //telemetry.addData("velocity:",velocity);
        CarouselMotor.setVelocity(velocity);
    }
}

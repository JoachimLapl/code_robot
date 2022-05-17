package org.firstinspires.ftc.teamcode.auto;
import android.graphics.Color;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.CameraReco;
import org.firstinspires.ftc.teamcode.robot.Carousel;
import org.firstinspires.ftc.teamcode.robot.Movement;
import org.firstinspires.ftc.teamcode.robot.Movement2;
import org.firstinspires.ftc.teamcode.robot.Position2;

@Autonomous(name="AutoOpMode Red Warehouse")
public class AutoR2 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Movement2 movement;
    private Carousel carousel;
    private Arm arm;
    private CameraReco camreco;
    private double lastRuntime = 0;

    @Override
    public void runOpMode() {
        // INIT //
        telemetry = new MultipleTelemetry(telemetry);
        movement = new Movement2(telemetry, runtime, hardwareMap);
        carousel = new Carousel(telemetry, runtime, hardwareMap);
        arm = new Arm(telemetry, hardwareMap);
        arm.closeGripper();
        update();
        waitForStart();

        // LOOP //
        move(250,.3);    // avance de 200 ticks
        pointTowards(60);    // se tourne vers -87 degrés
        arm.arm_pos = -28;
        move(250, .2);
        arm.openGripper();
        update();
        /*move(-2000);  // recule de 2500 ticks
        move(-1000,.2);  // recule de 200 ticks lentement
        turnCarousel();  // fait tourner la wheel pour le carousel
        arm.setPresetPosition(1);
        move(3000);
        move(2000,.2);
        arm.setPresetPosition(0);
        update();*/
        sleepT(100);

        // Pour s'assurer que la position de la pince est la bonne à la fin du programme
        while (opModeIsActive()){
            update();
        }
    }
    void move(int n, double speed) {
        movement.reset();
        double front_index = Math.signum(n);
        movement.front= Range.clip(speed,0,1)*front_index;
        while(movement.nv.mean()*front_index<n*front_index && opModeIsActive()){
            update();
        }
    }
    void move(int n) { move(n, 1); }
    void pointTowards(double a){
        movement.reset();
        while(movement.pointTowards(a)!=0 && opModeIsActive()){
            telemetry.addData("offset", movement.pointTowards(a));
            movement.positioning.show();
            update();
        }
        sleepT(1000);
    }
    void turnCarousel(){
        carousel.maxSpeed();
        carousel.apply();
        sleep(4000);
        sleepT(100);
    }
    void sleepT(int n) {
        movement.reset();
        movement.apply();
        movement.update();
        carousel.reset();
        carousel.apply();
        sleep(n);
    }
    void update(){
        movement.apply();
        movement.update();
        carousel.apply();
        arm.apply(-.02);
        telemetry.update();
    }
}

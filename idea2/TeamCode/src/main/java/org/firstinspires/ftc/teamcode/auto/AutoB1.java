package org.firstinspires.ftc.teamcode.auto;
import android.graphics.Color;

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

@Autonomous(name="AutoOpMode Blue Carousel")
public class AutoB1 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Movement2 movement;
    private Carousel carousel;
    private Arm arm;
    private CameraReco camreco;
    private double lastRuntime = 0;

    @Override
    public void runOpMode() {
        // INIT //
        movement = new Movement2(telemetry, runtime, hardwareMap);
        carousel = new Carousel(telemetry, runtime, hardwareMap);
        arm = new Arm(telemetry, hardwareMap);
        arm.openGripper();
        waitForStart();
        arm.closeGripper();
        arm.apply(-0.02);
        sleep(1000);
        arm.setPresetPosition(1);
        arm.apply(-0.02);
        sleep(1000);
        // LOOP //
        move(400);    // avance de 200 ticks
        pointTowards(85);    // se tourne vers -85 degrés
        rectMove(-1400);  // recule de 2500 ticks
        move(100,.2);
        pointTowards(0);
        move(-100);
        turnCarousel();
        rectMove(100);
        pointTowards(-10);
        rectMove(1000);
        sleepT(250);
        pointTowards(90);
        move(-300,0.4);
        sleepT(250);
        arm.setPresetPosition(0);
        arm.apply(-0.02);
        sleep(1000);
        arm.openGripper();
        arm.apply(-0.02);
        sleep(1000);

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
        sleepT(100);
    }
    void move(int n) { move(n, 1); }
    void rectMove(int n){
        movement.reset();
        double front_index = Math.signum(n);
        while(movement.nv.mean()*front_index<n*front_index && opModeIsActive()){
            movement.front = Range.clip((n-movement.nv.mean())/400+.2,-1,1);
            update();
        }
        sleepT(100);
    }
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
        carousel.maxSpeed(-1);
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
        carousel.apply();
        arm.apply(-.02);
        movement.update();
        telemetry.update();
    }
}

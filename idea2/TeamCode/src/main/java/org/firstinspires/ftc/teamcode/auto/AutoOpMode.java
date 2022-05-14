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

@Autonomous(name="AutoOpMode Linear")
public class AutoOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Movement2 movement;
    private Carousel carousel;
    private Arm arm;
    private CameraReco camreco;
    private double lastRuntime = 0;

    private static float goToWH_time = 0;
    private boolean first_return = true;
    private boolean first_freight_in = false;

    public void sleepFor(double millis) {
        telemetry.addData("Sleeping for ", millis);
        telemetry.update();
        double startTime = runtime.time();
        while (runtime.time() - startTime < (millis / 1000)) {
            if (!opModeIsActive()) {
                break;
            }
        }
    }

    @Override
    public void runOpMode() {
        // INIT //
        telemetry = new MultipleTelemetry(telemetry);
        movement = new Movement2(telemetry, runtime, hardwareMap);
        carousel = new Carousel(telemetry, runtime, hardwareMap);
        arm = new Arm(telemetry, hardwareMap);
        arm.closeGripper();
        waitForStart();

        // LOOP //
        move(180);    // avance de 200 ticks
        pointTowards(-85);    // se tourne vers -87 degrés
        move(-2000);  // recule de 2500 ticks
        move(-1000,.2);  // recule de 200 ticks lentement
        turnCarousel();  // fait tourner la wheel pour le carousel
        arm.setPresetPosition(1);
        move(3000);
        move(2000,.2);
        arm.setPresetPosition(0);
        update();
        sleepT(100);

        // Just pour afficher
        while (opModeIsActive()){
            telemetry.addData("mean", movement.nv.mean());
            update();
        }

        // camreco = new CameraReco(telemetry, hardwareMap);
        // Auto //
        // int duck_position = camreco.duck_position();
        /*
        get the position of the duck ( between 0, 1 and 2)
        the associated position of the arm is given by position+2 (associated_pos = position+2)
        go to the shipping hub (while ([not close enough]) { movement.update(); movement.moveTo([shipping hub's position]); })
        put the duck in the right position (while ([arm not in the right place]) { arm.keepPosition(associated_pos); } arm.openGripper();)
        go to the carousel (while ([not close enough]) { movement.update(); movement.moveTo([carousel's position]); })
        turn the carousel until the duck falls (double time = runtime.time(); while (runtime.time()-time<[some value]) { carousel.increaseSpeed(); })
        if the time to go to the warehouse isn't over:
            take the duck
            go to the shipping hub
            put the duck in the right position in the shipping hub
         go to the warehouse
         */
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
    void turn(double a){
        movement.reset();
        movement.turn=-Math.signum(a)*.5; // *0.5 temp demi puissance
        double init = movement.positioning.getAngle();
        while((movement.positioning.getAngle()-init)*movement.turn<a*movement.turn && opModeIsActive()){
            movement.positioning.show();
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

package org.firstinspires.ftc.teamcode.manual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.AccelPositioning;
import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Carousel;
import org.firstinspires.ftc.teamcode.robot.Movement2;
import org.firstinspires.ftc.teamcode.robot.Vector;

@TeleOp(name="Manual solo")
public class ManualOpMode  extends OpMode {


    //////////////////////////////////////////////////////// CLASS MEMBERS /////////////////////////////////////////////////////////

    private ElapsedTime runtime = new ElapsedTime();
    private Movement2 movement;
    private Carousel carousel;
    private Arm arm;
    private boolean g1a = true, g1b = true;

    private AccelPositioning accel;

    private double lastRuntime = 0.0;

    ///////////////////////////////////////////////////////// OPMODE METHODS /////////////////////////////////////////////////////////
    @Override
    public void init() {
        movement = new Movement2(telemetry, runtime, hardwareMap);
        carousel = new Carousel(telemetry, runtime, hardwareMap);
        arm = new Arm(telemetry, hardwareMap);
        arm.openGripper();
        accel = new AccelPositioning(hardwareMap, telemetry);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }


    ///////////////////////////////////////////////////////// MAIN LOOP /////////////////////////////////////////////////////////

    @Override
    public void loop() {
        movement.update();

        //accel.show();

        double DeltaT = lastRuntime - runtime.time(); // la différence de temps
        telemetry.addData("time", runtime.time());

        //------ Wheels ------//
        movement.gamepadMoves(gamepad1); // basically does all the motions asked by player through gamepad

        //telemetry.addData("a,b", String.valueOf(gamepad1.a)+','+String.valueOf(gamepad1.b));

        //------ Arm ------//
        if (gamepad1.a && !g1a){ // if we press a, we go to the next preset position
            arm.toNewPreset(1);
        } else if (gamepad1.b && !g1b){ // if we press b, we go to the previous preset position
            arm.toNewPreset(-1);
        }
        g1a = gamepad1.a; // so if we press a button then it doesn't stay pressed all the time
        g1b = gamepad1.b; // same thing
        if (gamepad1.x){
            arm.openGripper();
        } else if (gamepad1.y){
            arm.closeGripper();
        }
        arm.movestick(gamepad1);
        if (gamepad1.start){
            movement.pointTowards(0);
        }
        //------ Carousel ------//
        if (gamepad1.right_bumper){
            carousel.maxSpeed();
        } else if (gamepad1.left_bumper){
            carousel.increaseSpeed();// TODO: that thing doesn't seem to work
        } else {
            carousel.reset();
        }

        //telemetry.addData("ticks", arm.arm_motor.getCurrentPosition());

        arm.apply(DeltaT);
        carousel.apply();
        movement.apply();
        lastRuntime = runtime.time();
        telemetry.update();
    }

    ///////////////////////////////////////////////////////// STOP METHOD /////////////////////////////////////////////////////////
    @Override
    public void stop() {
    }
}

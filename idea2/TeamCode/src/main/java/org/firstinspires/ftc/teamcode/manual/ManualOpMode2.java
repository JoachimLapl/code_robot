package org.firstinspires.ftc.teamcode.manual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Carousel;
import org.firstinspires.ftc.teamcode.robot.Movement2;

@TeleOp(name="Manual")
public class ManualOpMode2  extends OpMode {

    //////////////////////////////////////////////////////// CLASS MEMBERS /////////////////////////////////////////////////////////

    private ElapsedTime runtime = new ElapsedTime();
    private Movement2 movement;
    private Carousel carousel;
    private Arm arm;
    private boolean g1a = true, g1b = true;

    private double lastRuntime = 0.0;

    ///////////////////////////////////////////////////////// OPMODE METHODS /////////////////////////////////////////////////////////
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry);
        movement = new Movement2(telemetry, runtime, hardwareMap);
        carousel = new Carousel(telemetry, runtime, hardwareMap);
        arm = new Arm(telemetry, hardwareMap);
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

        double DeltaT = lastRuntime - runtime.time();

        movement.gamepadMoves(gamepad1);

        telemetry.addData("a,b", String.valueOf(gamepad1.a)+','+String.valueOf(gamepad1.b));

        /*
        if (gamepad1.a) {
            if (gamepad1.b){
                arm.keepPosition(-360, DeltaT);
            }else{
                arm.keepPosition(-303, DeltaT);
            }
        } else if (gamepad1.b) {
            arm.keepPosition(-342, DeltaT);
        } else {
            arm.keepPosition(-6, DeltaT);
        }
         */
        if (gamepad1.a && !g1a){
            arm.toNewPreset(1);
        } else if (gamepad1.b && !g1b){
            arm.toNewPreset(-1);
        }
        g1a = gamepad1.a;
        g1b = gamepad1.b;
        if (gamepad1.x){
            arm.openGripper();
        } else if (gamepad1.y){
            arm.closeGripper();
        };

        arm.movestick(gamepad1);

        if (gamepad1.right_bumper){
            carousel.maxSpeed();
        } else if (gamepad1.left_bumper){
            carousel.increaseSpeed();
        } else {
            carousel.reset();
        }

        telemetry.addData("ticks", arm.arm_motor.getCurrentPosition());



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

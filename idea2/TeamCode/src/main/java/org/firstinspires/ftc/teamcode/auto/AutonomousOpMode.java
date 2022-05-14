package org.firstinspires.ftc.teamcode.auto;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Carousel;
import org.firstinspires.ftc.teamcode.robot.Movement2;
import org.firstinspires.ftc.teamcode.robot.Vector;

@Autonomous(name="AutoOpMode")
public class AutonomousOpMode extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Movement2 movement;
    private Carousel carousel;
    private Arm arm;
    private double lastRuntime = 0;

    private static float goToWH_time = 0;
    private boolean first_return = true;
    private boolean first_freight_in = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry);
        movement = new Movement2(telemetry, runtime, hardwareMap);
        carousel = new Carousel(telemetry, runtime, hardwareMap);
        arm = new Arm(telemetry, hardwareMap);
        arm.openGripper();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        runtime.reset();
        first_return = true;
    }

    private void goToWareHouse() {
        if (first_return) {
            arm.closeGripper();
            arm.setPresetPosition(0);
        }
        //movement.moveTowards(new Vector(0,0));
        first_return=false;
    }

    @Override
    public void loop() {
        movement.update();
        double time = runtime.time(); // seconds
        double DeltaT = lastRuntime - time;
        if (first_freight_in) {
            if (time > goToWH_time) {
                goToWareHouse();
            } else {

            }
        } else {
            /*
            TODO: find the right position where to put the freight and put the freight in the shipping hub
             */
        }
        arm.apply(DeltaT);
        carousel.apply();
        movement.apply();
        lastRuntime = runtime.time();
        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
    }
}

package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class WheelsUpdate {
    private DcMotor[] wheels;
    public Vector position = new Vector(0,0);
    public Vector orientation = new Vector(1,0);

    private static Vector[] positions = Consts.WHEELS_POSITIONS; // to be calculated
    private static Vector[] directions = Consts.WHEELS_DIRECTIONS;

    private int[] ticks = new int[4];
    private static Vector middle = positions[0].plus(positions[1]).plus(positions[2]).plus(positions[3]).multiply(.25);
    private Telemetry telemetry;

    public WheelsUpdate(Telemetry globalTelemetry, DcMotor ...w) {
        telemetry = globalTelemetry;
        wheels = w;
        telemetry.addData("wheels_update", "init");
        telemetry.update();
        for (int i = 0; i < 4; i++) {
            ticks[i] = w[i].getCurrentPosition();
        }
    }
    private Vector getMiddle(Vector ...v){
        Vector u = new Vector(0,0);
        for (Vector a:v) { u = u.plus(a); }
        return u.multiply(1/v.length);
    }
    public void update() {
        int[] n_ticks = new int[4];
        for (int i = 0; i<4; i++) {
            int t = wheels[i].getCurrentPosition();
            n_ticks[i] = ticks[i]-t;
            telemetry.addData("mvt "+String.valueOf(i), n_ticks[i]);
            ticks[i] = t;
        }
        Vector[] vectors = new Vector[4];
        for (int i = 0; i< 4; i++) {
            vectors[i] = directions[i].multiply(n_ticks[i]);
            telemetry.addData("vector "+String.valueOf(i), vectors[i].toStr());
        }
        Vector middleVec = getMiddle(vectors);
        telemetry.addData("middle_Vec", middleVec.toStr());
        double a = 0;
        for (int i = 0; i < 4; i++) {
            telemetry.addData("angle_vector_"+String.valueOf(i), vectors[i].plus(positions[i]).minus(middleVec).toStr());
            a += vectors[i].plus(positions[i]).minus(middleVec).getAngle();
        }
        a *= .25;
        telemetry.addData("angle", a);
        Vector v = new Vector(-Math.cos(a), Math.sin(a));
        if (v.x != 0 && v.y != 0) {
            Vector perp_traj = new Vector(middleVec.y, -middleVec.x).multiply(1 / a);
            Vector r_pos = perp_traj.multiply(-1);
            Vector n_pos = r_pos.rotate(-a);
            middleVec = n_pos.plus(perp_traj);
        }
        orientation.rotate(v);
        position.plus(middleVec);
    }
}

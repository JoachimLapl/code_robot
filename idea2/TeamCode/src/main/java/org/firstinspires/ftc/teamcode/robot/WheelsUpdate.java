package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class WheelsUpdate {
    private DcMotor[] wheels;
    private static Vector[] positions = Consts.WHEELS_POSITIONS; // to be calculated
    private static float[] diameters = Consts.WHEELS_DIAMETERS; // mm
    private static Vector[] directions = Consts.WHEELS_DIRECTIONS;
    private Vector[] estim_pos = new Vector[4];
    private int[] ticks = new int[4];
    private static Vector mean = positions[0].plus(positions[1]).plus(positions[2]).plus(positions[3]).multiply(.25);
    private static Vector[] vectorAngles = {positions[0].minus(mean),positions[1].minus(mean),positions[2].minus(mean),positions[3].minus(mean)};
    private static double[] wAngles = {vectorAngles[0].getAngle(),vectorAngles[1].getAngle(),vectorAngles[2].getAngle(),vectorAngles[3].getAngle()};
    private Telemetry telemetry;
    public WheelsUpdate(Telemetry globalTelemetry, DcMotor ...w) {
        telemetry = globalTelemetry;
        wheels = w;
        telemetry.addData("wheels_update", "init");
        telemetry.update();
        /*
        for (int i = 0; i < 4; i++) {
            ticks[i] = w[i].getCurrentPosition();
        }
         */
    }
    public Vector[] update() {
        Vector[] motions = new Vector[4];
        Vector mid = new Vector(0,0);
        for (int i = 0; i < 4; i++) {
            int n_pos = wheels[i].getCurrentPosition();
            int offset = n_pos - ticks[i];
            if (offset > 250) {
                offset -= 500;
            } else if (offset < 250) {
                offset += 500;
            }
            double distance = (double)offset/500*Math.PI*diameters[i];
            Vector vector = directions[i].multiply(distance);
            motions[i] = vector;
            mid = mid.plus(vector);
            ticks[i] = n_pos;
        }
        mid = mid.multiply(.25);
        double angle = 0;
        // Vector orient = new Vector(0,1);
        for (int i = 0; i < 4; i++) {
            // orient = orient.rotate(vectorAngles[i],false).rotate(motions[i].minus(mid), true);
            angle += wAngles[i] - motions[i].getAngle();
        }
        // orient = new Vector(0,3).plus(orient.setToNorm(1));
        angle *= .25;
        Vector[] result = {mid.minus(mean), new Vector(Math.cos(angle), Math.sin(angle))};//orient.setToNorm(1)};
        return result;
    }
}

package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Vector {
    public double x;
    public double y;
    public double norm;
    public Vector(double a, double b) {
        x = a;
        y = b;
        norm = Math.sqrt(Math.pow(a,2)+Math.pow(b,2));
    }
    public Vector plus(Vector v){
        return new Vector(x+v.x,y+v.y);
    }
    public Vector minus(Vector v){
        return new Vector(x-v.x,y-v.y);
    }
    public Vector multiply(double k){
        return new Vector(x*k,y*k);
    }
    public Boolean isOpposite(Vector v){
        return Math.pow(x+v.x,2)+Math.pow(y+v.y,2)<Math.pow(x,2)+Math.pow(y,2)+Math.pow(v.x,2)+Math.pow(v.y,2);
    }
    public Vector setToNorm(double s){
        return multiply(norm == 0 ? 0 : s/norm);
    }
    public double getAngle(){
        return y>0 ? Math.acos(x/norm) : -Math.acos(x/norm);
    }
    public Vector rotate(Vector v, Boolean trigo){
        //v = v.setToNorm(1);
        int coef = trigo ? 1: -1;
        return new Vector(v.x*x-v.y*y*coef,v.x*y+v.y*x*coef); // ne conserve pas la norme
    }
    public Vector rotate(double n, Boolean trigo){ return rotate(new Vector(Math.cos(n), Math.sin(n)), trigo); }
    public Vector rotate(double n){ return rotate(n, true); }
    public Vector rotate(Vector v){ return rotate(v, true); }

}

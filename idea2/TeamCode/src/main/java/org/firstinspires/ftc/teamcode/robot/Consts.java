package org.firstinspires.ftc.teamcode.robot;

public class Consts { // units: mm, rad
    /* Robot Dimensions */
    public static float Robot_Width = 0f;
    public static float Robot_Height = 0f;
    public static float Robot_Depth = 0f;

    /* Wheels */
    // GripWheel
    public static int GripWheel_diameter = 90;
    public static double GripWheel_circumference = Math.PI*GripWheel_diameter;
    // TractionWheel
    public static int TractionWheel_diameter = 90;
    public static double TractionWheel_circumference = Math.PI*TractionWheel_diameter;
    // OmniWheel
    public static int OmniWheel_diameter = 90;
    public static double OmniWheel_circumference = Math.PI*OmniWheel_diameter;

    // LF  RF  RB  LB
    public static Vector[] WHEELS_POSITIONS = {new Vector(0,0), new Vector(0,0), new Vector(0,0), new Vector(0,0)};
    public static float[] WHEELS_DIAMETERS = {127, 127, 127, 127}; // mm
    public static Vector[] WHEELS_DIRECTIONS = {
            new Vector(0,1).multiply(Math.PI/250*WHEELS_DIAMETERS[0]),
            new Vector(0,-1).multiply(Math.PI/250*WHEELS_DIAMETERS[0]),
            new Vector(0,-1).multiply(Math.PI/250*WHEELS_DIAMETERS[0]),
            new Vector(0,1).multiply(Math.PI/250*WHEELS_DIAMETERS[0])
    };

    /* Motors */
    public static int N_Ticks = 500;
}

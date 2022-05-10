package org.firstinspires.ftc.teamcode.robot;

import java.util.Arrays;

public class VectorN {
    public double[] coordinates;
    public int length;
    public VectorN(double[] coord){
        coordinates = coord;
        length = coord.length;
    }
    public VectorN add(VectorN ...v){
        double[] arr = coordinates.clone();
        for (int i = 0; i<length; i++){
            for (VectorN vec:v) {
                arr[i] += vec.coordinates[i];
            }
        }
        return new VectorN(arr);
    }
    public VectorN subtract(VectorN v){
        double[] arr = coordinates.clone();
        for (int i = 0; i<length; i++){
            arr[i] -= v.coordinates[i];
        }
        return new VectorN(arr);
    }
    public VectorN multiply(double n){
        double[] arr = coordinates.clone();
        for (int i = 0; i<length; i++){ arr[i] *= n; }
        return new VectorN(arr);
    }
    public void setCoordinates(double ...coord) { coordinates = coord; }
    public double mean(){
        double s = 0;
        for (double i:coordinates){
            s+=i;
        }
        return s/length;
    }
    public double median(){
        double[] s =  coordinates.clone();
        Arrays.sort(s);
        if (length%2 == 0){
            return (s[length/2]+s[length/2+1])/2;
        } else {
            return s[(length+1)/2];
        }
    }
}

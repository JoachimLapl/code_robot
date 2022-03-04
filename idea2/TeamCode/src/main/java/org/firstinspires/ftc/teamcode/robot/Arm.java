/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.robot;

// import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// @Config
public class Arm {
    private Servo pince_servo;
    public DcMotorEx arm_motor;

    private Telemetry telemetry;

    private double position;
    private int velocity = 0;

    private static int openPosition = 0;   // requires testing
    private static int closePosition = 1;// requires testing

    public static double[] preset = { -6,-303,-342,-360 };
    public double arm_pos = -6;


    public Arm(Telemetry globalTelemetry, HardwareMap hardwareMap) {
        telemetry = globalTelemetry;
        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor"); arm_motor.setDirection(DcMotorEx.Direction.FORWARD);
        pince_servo = hardwareMap.get(Servo.class, "bucket_servo"); pince_servo.setDirection(Servo.Direction.FORWARD);
    }
    public void keepPosition(double ticks, double DeltaT){
        // ticks = position voulue, DeltaT = différence de temps
        /*
            soit p = position:
            différence est: (p-ticks+250)%500-250
         */
        double diff = ((getPosition()-ticks)%500);
        telemetry.addData("ticks", arm_motor.getCurrentPosition()/25*3);
        telemetry.addData("différence", diff);
        velocity = (int) Math.min(Math.max(diff/DeltaT,-1e3),1e3);
        telemetry.addData("velocity",velocity);
    }
    public void movestick(Gamepad gamepad) {
        arm_pos += gamepad.right_stick_y;
    }
    public double getPosition(){ return ((double)arm_motor.getCurrentPosition())/25*3; }
    public void setPresetPosition(int n){ arm_pos = preset[n]; }
    public void openGripper(){
        position = openPosition;
    }
    public void closeGripper(){
        position = closePosition;
    }
    public void apply(double DeltaT) {
        pince_servo.setPosition(position);
        telemetry.addData("pince position", position);
        telemetry.addData("velocity", velocity);

        keepPosition(arm_pos, DeltaT);
        if ((getPosition()>-393 && velocity<0) || (getPosition()<-9 && velocity>0)) {
            arm_motor.setVelocity(velocity);
            telemetry.addData("Arm Velocity", velocity);
        } else {
            arm_motor.setPower(0);
            telemetry.addData("Arm Velocity", 0);
        }
        telemetry.addData("Current arm position", getPosition());
    }
}

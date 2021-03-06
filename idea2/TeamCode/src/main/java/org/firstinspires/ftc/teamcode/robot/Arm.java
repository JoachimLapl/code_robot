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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// @Config
public class Arm {
    private Servo pince_servo;
    //public DcMotorEx arm_motor;
    public DcMotor arm_motor;
    private Telemetry telemetry;

    private double position;
    private int velocity = 0;

    private static double openPosition = 0.8;         // requires testing
    private static double closePosition = 0.4;        // requires testing 0.5

    //public static double[] preset = { 0, -30, -280, -327, -355 };
    public static int[] preset = { 0, 300, 2350, 2650, 3000 };
    public int pos_index = 0;

    private int relative = 100;
    public int arm_pos = 0;

    public Arm(Telemetry globalTelemetry, HardwareMap hardwareMap) {
        telemetry = globalTelemetry;
        //arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor"); arm_motor.setDirection(DcMotorEx.Direction.FORWARD);
        arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
        arm_motor.setDirection(DcMotor.Direction.FORWARD);
        relative = 0; //getPosition();
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_motor.setTargetPosition(0);

        arm_motor.setPower(0.6);
        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pince_servo = hardwareMap.get(Servo.class, "bucket_servo"); pince_servo.setDirection(Servo.Direction.FORWARD);
    }
    public void keepPosition(double ticks, double DeltaT) {
        double diff = ((getPosition()+ticks+100-relative)%500-100);
        velocity = (int) Math.min(Math.max(diff/DeltaT/2,-2e3),2e3);
    }
    public void movestick(Gamepad gamepad) {
        arm_pos += gamepad.right_stick_y;
    }
    public double getPosition() { return ((double)arm_motor.getCurrentPosition())/25*3; }
    public void setPresetPosition(int n) { Math.max(Math.min(pos_index = n,4),0); arm_pos = preset[n]; }
    public void toNewPreset(int index) {
        pos_index = Math.max(Math.min(pos_index+index, 4),0);
        arm_pos = preset[pos_index];
    }
    public void openGripper() {
        pince_servo.setPosition(openPosition);
    }
    public void closeGripper() {
        pince_servo.setPosition(closePosition);
    }
    public void apply(double DeltaT) {
        arm_motor.setTargetPosition((int) arm_pos);


        //keepPosition(arm_pos, DeltaT);
        //arm_motor.setVelocity(velocity);
    }
}

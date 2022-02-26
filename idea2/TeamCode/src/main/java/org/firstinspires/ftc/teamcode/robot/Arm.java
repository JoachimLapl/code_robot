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
    private DcMotorEx arm_motor;

    private Telemetry telemetry;

    private double position;
    private int velocity;

    private static int openPosition = 0;   // requires testing
    private static int closePosition = 1;// requires testing


    public Arm(Telemetry globalTelemetry, HardwareMap hardwareMap) {
        telemetry = globalTelemetry;
        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor"); arm_motor.setDirection(DcMotorEx.Direction.FORWARD);
        pince_servo = hardwareMap.get(Servo.class, "bucket_servo"); pince_servo.setDirection(Servo.Direction.FORWARD);
    }
    public void keepPosition(int ticks, double DeltaT){
        telemetry.addData("delta", DeltaT);
        telemetry.addData("ticks", arm_motor.getCurrentPosition());
        velocity = (int) Math.min(Math.max(((arm_motor.getCurrentPosition()-ticks)/DeltaT),-2e3),2e3);
        telemetry.addData("velocity",velocity);
    }
    public void openGripper(){
        position = openPosition;
    }
    public void closeGripper(){
        position = closePosition;
    }
    public void apply() {

        pince_servo.setPosition(position);
        telemetry.addData("pince position", position);

        arm_motor.setVelocity(velocity);
        telemetry.addData("Arm Velocity", velocity);
        telemetry.addData("Current arm position", arm_motor.getCurrentPosition());
    }
}

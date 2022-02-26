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

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//@Config
public class Movement {
    public static double frontLeftCoeff = 1;
    public static double frontRightCoeff = 1;
    public static double backLeftCoeff = 1;
    public static double backRightCoeff = 1;
    public static double turnSensitivity = 2; // Increasing sensitivity gives more priority to turning
    public static double frontSensitivity = 1;
    public static double sidewaysSensitivity = 1;
    public static double maxDpadSensitivity = 0.4; // How much the robot moves when dpad is used
    public static double timeToDpadFront = 3;
    public static double timeToDpadSideways = 2;
    public static double dpadMarginFront = 0.2;
    public static double dpadMarginSideways = 0.3;

    public static double FL_position_X = 0d;
    public static double FL_position_Y = 0d;
    public static double FR_position_X = 0d;
    public static double FR_position_Y = 0d;
    public static double BL_position_X = 0d;
    public static double BL_position_Y = 0d;
    public static double BR_position_X = 0d;
    public static double BR_position_Y = 0d;

    public double FL_ticks_ratio = 1;
    public double FR_ticks_ratio = 1;
    public double BL_ticks_ratio = 1;
    public double BR_ticks_ratio = 1;

    public static double getAngle(double x,double y){
        if (x*y == 0)
            return 0;
        /*
        hypotenuse: Math.hypot(a,b) = (a,b)=>(a**2+b**2)**.5
        asin(y/hypotenuse) = angle ou -angle, pour le savoir,
            si x>1, angle = asin(y/hypotenuse),
            sinon, angle = pi - asin(y/hypotenuse),
         */
        double angleA = Math.asin(y/Math.hypot(x,y));
        return x<0?Math.PI - angleA : angleA;
        //ou aussi, Math.PI/2 - (x < 0 ? -1 : 1) * (Math.PI/2 - Math.asin(y / Math.hypot(x,y)));
    }

    public static double FL_distance = Math.hypot(FL_position_X,FL_position_Y);
    public static double FR_distance = Math.hypot(FR_position_X,FR_position_Y);
    public static double BL_distance = Math.hypot(BL_position_X,BL_position_Y);
    public static double BR_distance = Math.hypot(BR_position_X,BR_position_Y);
    public static double FL_angle = getAngle(FL_position_X,FL_position_Y);
    public static double FR_angle = getAngle(FR_position_X,FR_position_Y);
    public static double BL_angle = getAngle(BL_position_X,BL_position_Y);
    public static double BR_angle = getAngle(BR_position_X,BR_position_Y);

    public static double Wheels_Circumference = Consts.TractionWheel_circumference;

    private float turn = 0f;

    private ElapsedTime runtime;

    // Initialize motors
    private DcMotor FLmotor, FRmotor, BLmotor, BRmotor, ticksMotor;

    private double FL_ticks = 0;
    private double FR_ticks = 0;
    private double BL_ticks = 0;
    private double BR_ticks = 0;

    private boolean isReduced = false;

    private Telemetry telemetry;

    private double firstTurnTicks;
    private boolean rotateLock;
    private double N_ticks = Consts.N_Ticks;
    private double turnAngle, turnSpeed; // Angle in degrees

    private double dpadTime; // The time the dpad has been pressed, to progressively ramp up speed

    public Movement(Telemetry globalTelemetry, ElapsedTime globalRuntime, DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR, boolean isReducedMain) {

        // INITIALIZE TELEMETRY
        telemetry = globalTelemetry;

        runtime = globalRuntime;

        isReduced = isReducedMain;

        // Assign motors from ManualOpMode
        FLmotor  = FL; FLmotor.setDirection(DcMotor.Direction.FORWARD);
        FRmotor = FR; FRmotor.setDirection(DcMotor.Direction.REVERSE);
        BLmotor  = BL;  BLmotor.setDirection(DcMotor.Direction.FORWARD);
        BRmotor = BR;  BRmotor.setDirection(DcMotor.Direction.REVERSE);

        ticksMotor = BLmotor;
    }

    /* MOVEMENT CALCULATIONS */

    // Move `front` ticks while turing `turn` rad
    @Deprecated
    public void move(double front,double u, double turn) { // d = \alpha r/c_r \times 500
        double s = turn/Wheels_Circumference*N_ticks;
        FL_ticks  = front - s * FL_distance;
        FR_ticks = front + s * FR_distance;
        BL_ticks   = front - s * BL_distance ;
        BR_ticks  = front + s * BR_distance;
    }

    public void move(double front, double turn) { // d = \alpha r/c_r \times 500
        move(front,0,turn);
    }

    /*// Move with an angle and a speed instead
    public void polarMove(double angle, double speed, double turn) {
        move(
                Math.sin(angle) * speed,
                Math.cos(angle) * speed,
                turn
        );
    }*/

    // Add extra rotation
    public void rotate(double angleInDegrees, double speed) {
        rotateLock = true;
        firstTurnTicks = ticksMotor.getCurrentPosition();
        turnAngle = angleInDegrees; turnSpeed = speed;
    }

    // Add rotation lock
    public void keepRotating() {
        if (turnAngle > 0) {
            if (firstTurnTicks + turnAngle / 360 * N_ticks > ticksMotor.getCurrentPosition()) {
                move(0, turnSpeed);
            } else
                rotateLock = false;
        } else {
            if (firstTurnTicks - turnAngle / 360 * N_ticks < ticksMotor.getCurrentPosition()) {
                move(0, -turnSpeed);
            } else
                rotateLock = false;
        }
    }

    // Check if rotation lock is enabled
    public boolean checkRotating() {
        return rotateLock;
    }

    // Turns off all motors, full stop
    public void reset(boolean includeTurn) {
        FL_ticks = 0;
        FR_ticks = 0;
        BL_ticks = 0;
        BR_ticks = 0;
        if (includeTurn)
            turn = 0f;
    }
    public void reset() {
        reset(true);
    }

    /* GAMEPAD CONTROL */

    // Translates the robot with the left joystick
    public void joystickTranslate(Gamepad gamepad) {
        double turn = isReduced ? gamepad.left_stick_x / 2 : gamepad.left_stick_x;
        double front = isReduced ? gamepad.left_stick_y / 2 : gamepad.left_stick_y;
        // Uncomment for gradual speed increase
        if (Math.abs(turn) < 0.1) {
            turn = 0;
        } else if (Math.abs(turn) > 0.1 || Math.abs(turn) < 0.7) {
            turn = 0.83*turn+0.02;
        } else if (Math.abs(turn) >= 0.7 || Math.abs(turn) < 0.9) {
            turn = 2*turn-0.8;
        } else if (Math.abs(turn) >=0.9) {
            turn = 1*Math.signum(turn);
        } else {
            turn = 0;
        }

        if (Math.abs(front) < 0.1) {
            front = 0;
        } else if (Math.abs(front) > 0.1 || Math.abs(front) < 0.7) {
            front = 0.83*front+0.02;
        } else if (Math.abs(front) >= 0.7 || Math.abs(front) < 0.9) {
            front = 2*front-0.8;
        } else if (Math.abs(front) >=0.9) {
            front = 1;
        } else {
            front = 0;
        }
        move(
                front,
                //sideways,
                0
        );
    }

    // Move slowly with the dpad
    public void dpadTranslate(Gamepad gamepad) {
        if (!(gamepad.dpad_up || gamepad.dpad_down || gamepad.dpad_left || gamepad.dpad_right)) {
          dpadTime = runtime.time();
        } // Allows us to get the time the dpad was pressed, so we can progressively augment the speed of the robot
        double time = runtime.time() - dpadTime;
        double sidewaysTime = time/timeToDpadSideways;
        double frontTime = time/timeToDpadFront;
        sidewaysTime += dpadMarginSideways;
        frontTime += dpadMarginFront;
        if (sidewaysTime > maxDpadSensitivity) sidewaysTime = maxDpadSensitivity;
        if (frontTime > maxDpadSensitivity) frontTime = maxDpadSensitivity;
        move(
                (gamepad.dpad_down? frontTime :0) - (gamepad.dpad_up? frontTime :0),
                //(gamepad.dpad_right? sidewaysTime :0) - (gamepad.dpad_left? sidewaysTime :0),
                0
        );
    }

    // Turn with the bumpers and triggers
    public void bumperTurn(Gamepad gamepad) {
        if (gamepad.left_bumper || gamepad.right_bumper) {
            if (gamepad.left_bumper)
                turn -= 0.3;
            if (gamepad.right_bumper)
                turn += 0.3;
        } else {
            telemetry.addData("Right trigger", gamepad.right_trigger);
            telemetry.addData("Left trigger", gamepad.left_trigger);
            turn += gamepad.right_trigger*0.8;
            turn -= gamepad.left_trigger*0.8;
        }
        if (turn != 0) {
            reset(false); // Because Jeremy wanted to
        }
        move(0, turn);
    }

    /* GET INFO */

    public int getEncoder() {
        return FLmotor.getCurrentPosition();
    }

    /* APPLY CHANGES */

    // Apply all changes made before
    public void apply() {
        double FL_power = FL_ticks*FL_ticks_ratio;
        double FR_power = FR_ticks*FR_ticks_ratio;
        double BL_power = BL_ticks*BL_ticks_ratio;
        double BR_power = BR_ticks*BR_ticks_ratio;
        double coef = 1 / Math.max(
                Math.max(
                        Math.max(Math.abs(FL_ticks),Math.abs(FR_ticks)),
                        Math.max(Math.abs(BL_ticks),Math.abs(BR_ticks))
                ),1
            );
        FL_ticks*=coef; FL_power *= coef;
        FR_ticks*=coef; FR_power *= coef;
        BL_ticks*=coef; BL_power *= coef;
        BR_ticks*=coef; BR_power *= coef;
        FLmotor.setPower(Range.clip(FL_power, -1.0, 1.0));
        FRmotor.setPower(Range.clip(FR_power, -1.0, 1.0));
        BLmotor.setPower(Range.clip(BL_power, -1.0, 1.0));
        BRmotor.setPower(Range.clip(BR_power, -1.0, 1.0));

        // telemetry.addData("Front", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower);
        // telemetry.addData("Right", "left (%.2f), right (%.2f)", backLeftPower, backRightPower);
    }
}

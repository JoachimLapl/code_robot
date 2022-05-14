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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//@Config
public class Movement2 {
    public double front = 0d;
    public double turn = 0d;
    private ElapsedTime runtime;
    private WheelsUpdate wheels_update;
    private DcMotor FLmotor, FRmotor, BLmotor, BRmotor;
    private Telemetry telemetry;
    private Vector orientation = new Vector(1,0);
    private Vector motion = new Vector(0,0);
    private double[] powers = { 0, 0 };
    private static float distance_roues = 33.7f;
    private double[] arr = {0,0,0,0};
    private VectorN baseEncoder = new VectorN(arr);
    public VectorN nv = new VectorN(arr);
    public AccelPositioning positioning;

    public Movement2(Telemetry globalTelemetry, ElapsedTime globalRuntime, HardwareMap hardwareMap) {
        telemetry = globalTelemetry;
        runtime = globalRuntime;
        FLmotor = hardwareMap.get(DcMotor.class, "fl_motor"); FLmotor.setDirection(DcMotor.Direction.FORWARD); FLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRmotor = hardwareMap.get(DcMotor.class, "fr_motor"); FRmotor.setDirection(DcMotor.Direction.FORWARD); FRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLmotor = hardwareMap.get(DcMotor.class, "bl_motor"); BLmotor.setDirection(DcMotor.Direction.FORWARD); BLmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRmotor = hardwareMap.get(DcMotor.class, "br_motor"); BRmotor.setDirection(DcMotor.Direction.REVERSE); BRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("movement", "init");
        telemetry.update();
        wheels_update = new WheelsUpdate(telemetry, FLmotor,FRmotor,BLmotor,BRmotor);
        positioning =  new AccelPositioning(hardwareMap, telemetry);
        resetValues();
    }

    @Deprecated
    public void move(double f, double t) { // d = \alpha r/c_r \times 500
        front = f;
        turn = t;
    }

    public void reset() {
        resetValues();
        front = 0d;
        turn = 0d;
    }

    public void resetValues(){
        baseEncoder.setCoordinates(
                FLmotor.getCurrentPosition(),
                FRmotor.getCurrentPosition(),
                BLmotor.getCurrentPosition(),
                BRmotor.getCurrentPosition()
        );
    }

    public void gamepadMoves(Gamepad gamepad){
        double lx = gamepad.left_stick_x+(gamepad.dpad_left?-1:0)+(gamepad.dpad_right?1:0)+(gamepad.x?-.4:0)+(gamepad.b?+.4:0),
                ly = gamepad.left_stick_y+(gamepad.dpad_up?-1:0)+(gamepad.dpad_down?1:0)+(gamepad.a?.4:0)+(gamepad.y?-.4:0);
        double a = Math.abs(ly), b = Math.abs(lx);
        front = a<.1 ? 0 : a<.9 ? (a-.1)*1.25*Math.signum(ly) : Math.signum(ly);
        turn = b<.1 ? 0 : b<.9 ? (b-.1)*1.25*Math.signum(lx) : Math.signum(lx);
        //turn += (gamepad.left_bumper ? .25 : 0) - (gamepad.right_bumper ? .25 : 0);
    }

    public void apply() {
        double[] a = {FLmotor.getCurrentPosition(),FRmotor.getCurrentPosition(),BLmotor.getCurrentPosition(),BRmotor.getCurrentPosition()};
        VectorN nv = new VectorN(a).subtract(baseEncoder);
        telemetry.addData("baseEncoder", String.format("%s, %s, %s, %s", nv.coordinates[0], nv.coordinates[1], nv.coordinates[2], nv.coordinates[3]));
        telemetry.addData("mean", nv.mean());
        telemetry.addData("median", nv.median());
        double[] i = {front + turn,front - turn};
        powers[0] = (powers[0]+(i[0] - powers[0])/5)*(i[0]==0?0:1);
        powers[1] = (powers[1]+(i[1] - powers[1])/5)*(i[1]==0?0:1);
        //telemetry.addData("power1", powers[0]);
        //telemetry.addData("i1", i[0]);
        //telemetry.addData("power2", powers[1]);
        FLmotor.setPower(Range.clip(powers[0], -1.0, 1.0));
       // FLmotor.setPower(0.);
       FRmotor.setPower(Range.clip(powers[1], -1.0, 1.0));
       // FRmotor.setPower(0.);
        BLmotor.setPower(Range.clip(powers[0], -1.0, 1.0));
       BRmotor.setPower(Range.clip(powers[1], -1.0, 1.0));
        // BRmotor.setPower(0.);
    }
    public double rotationIndex(){
        return (nv.coordinates[0]+nv.coordinates[2]-nv.coordinates[1]-nv.coordinates[3])/4; //2748.6
    }

    public double pointTowards(double a){
        a = (a+180)%360-180;
        double currentAngle = positioning.getAngle();
        double offset = a - currentAngle;
        if (offset<-180) offset += 360;
        if (offset > 180) offset -= 360;
        turn = Range.clip(Math.abs(offset)/100,.2,1) * (offset>0 && offset<180 ? -1: 1);
        if (Math.abs(offset) < 2) turn = 0;
        return turn; //offset;
    }

    /*public void moveTowards(Vector point){
        double dist = pointTowards(point);
        front = dist < 10 ? 0 : dist < 100? .1 : 1;
    }*/

    public void update(){
        double[] a = {FLmotor.getCurrentPosition(),FRmotor.getCurrentPosition(),BLmotor.getCurrentPosition(),BRmotor.getCurrentPosition()};
        nv = new VectorN(a).subtract(baseEncoder);
        //wheels_update.update();
        //telemetry.addData("position", wheels_update.position.toStr());
        //telemetry.addData("angle", wheels_update.orientation.getAngle()*180/Math.PI);
    }


}

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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: DadBot Teleop Linear", group="Linear Opmode")
//@Disabled
public class DadBot_Teleop_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor liftMotor = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private DigitalChannel liftLimit = null;


    // Declare variable constants
    public static final double MID_LEFT_SERVO  = 0.5 ;
    public static final double MID_RIGHT_SERVO  = 0.5 ;
    public static final double CLAW_SPEED  = 0.005 ;
    public static final double CLAW_OPEN_LIMIT_DELTA  = .1;
    public static final double CLAW_CLOSE_LIMIT_DELTA = .15;
    public static final double LIFT_POWER = 0.3;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");
        liftLimit = hardwareMap.get(DigitalChannel.class, "lift_limit");

        // Initialize ALL installed motors and servos.
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        liftMotor.setPower(0);
        leftClaw.setPosition(MID_LEFT_SERVO);
        rightClaw.setPosition(MID_RIGHT_SERVO);
        liftLimit.setMode(DigitalChannel.Mode.INPUT);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Setup a variable for each drive wheel to save power level for telemetry
        // Drive motors
        double leftPower;
        double rightPower;

        // Lift
        boolean liftUp;
        boolean liftDown;
        float liftDownFloat;

        // Claw
        boolean clawClose;
        float clawCloseFloat;
        boolean clawOpen;
        double leftClawPosition = MID_LEFT_SERVO;
        double rightClawPosition = MID_RIGHT_SERVO;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            //double drive = -gamepad1.left_stick_y;
            //double turn  =  gamepad1.right_stick_x;
            //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            leftPower  = -gamepad1.left_stick_y ;
            rightPower = -gamepad1.right_stick_y ;

            // Lift controller
            liftUp = gamepad1.right_bumper;
            liftDownFloat = gamepad1.right_trigger;
            if(liftDownFloat > 0){
                liftDown = true;
            } else {
                liftDown = false;
            }

            // Claw Control
            clawCloseFloat = gamepad1.left_trigger;
            if(clawCloseFloat> 0) {
                clawClose = true;
            } else {
                clawClose = false;
            }
            clawOpen = gamepad1.left_bumper;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            //  Send calculated power to lift motor
            if(liftUp){
                liftMotor.setPower(LIFT_POWER);
            } else if (liftDown) {
                if (liftLimit.getState() == false){
                    liftMotor.setPower(0);
                } else {
                    liftMotor.setPower(-LIFT_POWER);
                }
            } else {
                liftMotor.setPower(0);
            }

            // Use gamepad X & B to open and close the claw
            if (clawOpen) {
                leftClawPosition += CLAW_SPEED;
                rightClawPosition -= CLAW_SPEED;
            } else if (clawClose) {
                leftClawPosition -= CLAW_SPEED;
                rightClawPosition += CLAW_SPEED;
            }

            // Move both servos to new position.
            leftClawPosition  = Range.clip(leftClawPosition, MID_LEFT_SERVO-CLAW_OPEN_LIMIT_DELTA, MID_LEFT_SERVO+CLAW_CLOSE_LIMIT_DELTA);
            leftClaw.setPosition(leftClawPosition);
            rightClawPosition = Range.clip(rightClawPosition, MID_RIGHT_SERVO-CLAW_CLOSE_LIMIT_DELTA, MID_RIGHT_SERVO+CLAW_OPEN_LIMIT_DELTA);
            rightClaw.setPosition(rightClawPosition);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}

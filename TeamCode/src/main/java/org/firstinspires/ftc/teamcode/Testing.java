/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Testing", group = "Gobilda")
//@Disabled
public class Testing extends OpMode {
    final double FEED_TIME_SECONDS = 1; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1025;

    // Declare OpMode members.
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    ElapsedTime feederTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftPower;
    double rightPower;

    // --- ADVANCED DRIVE STATE VARIABLES ---
    int gear = 1; // 0 = R, 1 = Gear 1, 2 = Gear 2
    double prevForward = 0.0; // For forward decay
    double prevTurn = 0.0;    // For turn decay
    
    // NEW DEBOUNCE VARIABLES FOR GEAR SHIFTING (FIX 1)
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    // ------------------------------------


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFeeder.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        
        // --- Config ---
        double baseSpeed = 0.6;// Gear 1 speed
        double brakeFactor = 0.8; // Smooth braking factor

        // -----------------------------------------------------------------
        // --- Fix 1: Corrected Gear Shifting Logic with Debouncing ---
        // -----------------------------------------------------------------
        boolean currentDpadUp = gamepad1.dpad_up || gamepad2.dpad_up;
        boolean currentDpadDown = gamepad1.dpad_down || gamepad2.dpad_down;

        if (currentDpadDown && !prevDpadDown) {
            // Shift up: cap at Gear 2 (2)
            gear = Math.min(gear + 1, 2);
        } else if (currentDpadUp && !prevDpadUp) {
            // Shift down: floor at Reverse (0)
            gear = Math.max(gear - 1, 0);
        }
        
        // Update previous state for next loop
        prevDpadUp = currentDpadUp;
        prevDpadDown = currentDpadDown;

        // Determine speed scale based on gear
        double speedScale;
        switch (gear) {
            case 0: // Reverse
                speedScale = baseSpeed; 
                break;
            case 1: // Gear 1
                speedScale = baseSpeed;
                break;
            case 2: // Gear 2
                speedScale = baseSpeed * 2.0;
                break;
            default:
                speedScale = baseSpeed;
        }

        // --- Forward/Backward Inputs (Triggers) ---
        double throttle = gamepad1.right_trigger;
        double brake = gamepad1.left_trigger;
        double throttle2 = gamepad2.right_trigger;
        double brake2 = gamepad2.left_trigger;

        double targetForward;
        double combinedThrottle = Math.max(throttle, throttle2);
        
        if (gear == 0) {
            targetForward = -combinedThrottle * speedScale; // Reverse
        } else {
            targetForward = combinedThrottle * speedScale; // Forward
        }

        // Apply braking (Triggers take priority)
        if ((brake > 0.05) || (brake2 > 0.05)) {
            targetForward = prevForward * brakeFactor; // decay speed
        }
        
        // --- TURNING LOGIC (Direction Corrected) ---
        
        // Read raw turn inputs from both gamepads and combine them
        // The negative sign is applied to reverse the turning direction
        double rawTurn1 = -gamepad1.right_stick_x; 
        double rawTurn2 = -gamepad2.right_stick_x; 
        double combinedRawTurn = rawTurn1 + rawTurn2;

        // Apply speed scale to the combined input
        double targetTurn = combinedRawTurn * speedScale;

        // --- Smooth braking/decay (Forward & Turn) ---
        
        // Forward Smoothing
        double forward = (Math.abs(targetForward) > 0.01) ? targetForward : prevForward * brakeFactor;

        // Turning Smoothing
        double turn;
        if (Math.abs(targetTurn) > 0.01) {
            turn = targetTurn;
        } else {
            // Apply decay (slower decay for turn than forward)
            turn = prevTurn * (brakeFactor / 2.0);
        }

        // Save current values for next loop iteration
        prevForward = forward;
        prevTurn = turn;
        
        // Optional: deadband to prevent drift from tiny stick movements
        if (Math.abs(turn) < 0.05){
            turn = 0;
        }

        // Drive the robot
        arcadeDrive(forward, turn); 


        /*
         * Launcher Manual Controls
         */
        if (gamepad1.y || gamepad2.y) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.b || gamepad2.b) { // stop flywheel
            launcher.setVelocity(STOP_SPEED);
        }

        /*
         * Now we call our "Launch" function.
         */
        // Using standard bumper check (no debouncing)
        launch(gamepad1.right_bumper || gamepad2.right_bumper);

        /*
         * Show the state and motor powers
         */
        String gearLabel;
        switch (gear) {
            case 0: gearLabel = "R"; break;
            case 1: gearLabel = "1"; break;
            case 2: gearLabel = "2"; break;
            default: gearLabel = "?"; break;
        }
        
        telemetry.addData("Gear", gearLabel);
        telemetry.addData("Forward Power", forward);
        telemetry.addData("Turn Power", turn);
        telemetry.addData("State", launchState);
        
        // -----------------------------------------------------------------
        // --- Fix 2: Swapped Telemetry Output (rightPower, leftPower) ---
        // -----------------------------------------------------------------
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", rightPower, leftPower);
        
        telemetry.addData("motorSpeed", launcher.getVelocity());
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void arcadeDrive(double forward, double rotate) {
        leftPower = forward + rotate;
        rightPower = forward - rotate;

        /*
         * Send calculated power to wheels
         */
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                break;
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AUtoCOdeKool", group = "Robot")
public class StarterBotAuto extends LinearOpMode {

    // Drive Motors
    private DcMotor leftDrive, rightDrive, leftDriveBack, rightDriveBack;
    // Attachments
    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;
    private double LAUNCHER_MAX_VELOCITY  = 1200;

    private ElapsedTime runtime = new ElapsedTime();

    // Encoder counts (Adjust these for your specific motor gear ratio)
    static final double TICKS_PER_INCH = 45.0;

    @Override
    public void runOpMode() {
        // --- 1. Hardware Mapping ---
        leftDrive      = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightDrive     = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftDriveBack  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightDriveBack = hardwareMap.get(DcMotor.class, "right_back_drive");

        launcher       = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder     = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder    = hardwareMap.get(CRServo.class, "right_feeder");

        // Directions
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);

        // Reset Encoders
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // --- 2. Step 1: Shoot 3 Times ---
        launcher.setVelocity(LAUNCHER_MAX_VELOCITY); // Spin up
        sleep(4000); // Wait for spin up

        for (int i = 0; i < 3; i++) {
            leftFeeder.setPower(1.0);
            rightFeeder.setPower(-1.0);
            sleep(400); // Time to push ring
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
            sleep(600); // Wait between shots
        }
        launcher.setPower(0); // Stop launcher

        // --- 3. Step 2: Move Backward (24 inches) ---
        moveRobot(24, 0, 0.5);

        // --- 4. Step 3: Move Side/Strafe (24 inches) ---
        // Positive is left, Negative is right
        moveRobot(0, 24, 0.5);

        telemetry.addData("Status", "Complete");
        telemetry.update();
    }

    /**
     * Helper to move the robot using encoders
     * @param forwardInches Positive is forward
     * @param strafeInches Positive is right
     * @param speed Motor power (0 to 1)
     */
    public void moveRobot(double forwardInches, double strafeInches, double speed) {
        int moveTicks = (int)(forwardInches * TICKS_PER_INCH);
        int strafeTicks = (int)(strafeInches * TICKS_PER_INCH);

        int lfTarget = leftDrive.getCurrentPosition() + moveTicks + strafeTicks;
        int rfTarget = rightDrive.getCurrentPosition() + moveTicks - strafeTicks;
        int lbTarget = leftDriveBack.getCurrentPosition() + moveTicks - strafeTicks;
        int rbTarget = rightDriveBack.getCurrentPosition() + moveTicks + strafeTicks;

        leftDrive.setTargetPosition(lfTarget);
        rightDrive.setTargetPosition(rfTarget);
        leftDriveBack.setTargetPosition(lbTarget);
        rightDriveBack.setTargetPosition(rbTarget);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        leftDriveBack.setPower(speed);
        rightDriveBack.setPower(speed);

        while (opModeIsActive() && leftDrive.isBusy()) {
            telemetry.addData("Moving", "To Position");
            telemetry.update();
        }

        stopDrive();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopDrive() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDriveBack.setPower(0);
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        leftDrive.setMode(mode);
        rightDrive.setMode(mode);
        leftDriveBack.setMode(mode);
        rightDriveBack.setMode(mode);
    }
}
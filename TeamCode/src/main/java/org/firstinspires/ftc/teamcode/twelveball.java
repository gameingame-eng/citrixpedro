package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Dash.DashboardDrawingHandler;

@Autonomous(name = "RedGoal12BallAuto", group = "Autonomous")
public class twelveball extends LinearOpMode {

    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();
    private int pathState;

    // Hardware variables
    private DcMotorEx shooterMotor;
    private DcMotor intakeMotor;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private CRServo leftFeeder;
    private CRServo rightFeeder;

    // Define Poses (X, Y, Heading)
    private final Pose startPose = new Pose(8, 56, Math.toRadians(0));
    private final Pose scorePose = new Pose(30, 56, Math.toRadians(0));

    private Path scorePath;

    @Override
    public void runOpMode() {
        // Initialize Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize Hardware using names from bobot.xml
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");


        // Build the path to the goal
        scorePath = new Path(new BezierLine(startPose, scorePose));
        scorePath.setConstantHeadingInterpolation(Math.toRadians(0));

        // Wait for Start
        while (!isStarted() && !isStopRequested()) {
            follower.update();
            telemetry.addData("Status", "Initialized - Check Config Names");
            telemetry.update();
        }

        if (isStopRequested()) return;

        pathState = 0;
        timer.reset();

        while (opModeIsActive()) {
            follower.update();
            autonomousControl();

            // Dashboard and Telemetry
            DashboardDrawingHandler.drawDebug(follower);
            telemetry.addData("Path State", pathState);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.update();
        }
    }

    public void autonomousControl() {
        switch (pathState) {
            case 0: // Move to Goal
                follower.followPath(scorePath);
                setPathState(1);
                break;

            case 1: // Wait until robot reaches the goal
                if (!follower.isBusy()) {
                    timer.reset();
                    setPathState(2);
                }
                break;

            case 2: // Shooting Logic
                // Activate launcher and feed mechanisms
                shooterMotor.setPower(1.0);
                intakeMotor.setPower(0.8);

                // CRServos usually need opposite powers to spin in the same direction relative to the intake
                leftFeeder.setPower(1.0);
                rightFeeder.setPower(-1.0);

                // Wait 4 seconds to ensure all balls are shot
                if (timer.seconds() > 4.0) {
                    shooterMotor.setPower(0);
                    intakeMotor.setPower(0);
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);
                    setPathState(3);
                }
                break;

            case 3: // Done
                telemetry.addData("Auto Status", "Complete");
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
    }
}
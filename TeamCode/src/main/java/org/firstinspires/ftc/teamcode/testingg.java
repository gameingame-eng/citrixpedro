package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Dash.DashboardDrawingHandler;

@Autonomous(name = "BlueGoalCorrectedDirection", group = "Autonomous")
public class testingg extends LinearOpMode {

    private Follower follower;
    private ElapsedTime timer = new ElapsedTime();
    private int pathState;

    private DcMotorEx shooterMotor;
    private DcMotor intakeMotor;
    private CRServo leftFeeder;
    private CRServo rightFeeder;

    // --- ADJUSTED POSES ---
    // If 35 was too far, 10-12 is usually "Up Close" against the wall.
    // Facing the goal usually means Heading is 0.
    private final Pose scorePose = new Pose(10, 114, Math.toRadians(0));

    // To move BACK: We increase X (Move toward the center of the field)
    // To move RIGHT: We decrease Y (Move toward the center of the field)
    private final Pose endPose = new Pose(35, 80, Math.toRadians(0));

    private Path moveBackRight;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(scorePose);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        // Single diagonal line move
        moveBackRight = new Path(new BezierLine(scorePose, endPose));
        moveBackRight.setConstantHeadingInterpolation(Math.toRadians(0));

        while (!isStarted() && !isStopRequested()) {
            follower.update();
            telemetry.addLine("Ready - Blue Goal Up Close");
            telemetry.update();
        }

        pathState = 0;
        timer.reset();

        while (opModeIsActive()) {
            follower.update();
            autonomousControl();

            if (pathState == 2 && !follower.isBusy()) {
                follower.breakFollowing();
            }

            DashboardDrawingHandler.drawDebug(follower);
            telemetry.addData("Path State", pathState);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.update();
        }
    }

    public void autonomousControl() {
        switch (pathState) {
            case 0: // Shoot 3 Balls
                shooterMotor.setPower(0.9);
                if (timer.seconds() > 0.8) { // Slightly longer spinup
                    intakeMotor.setPower(0.9);
                    leftFeeder.setPower(1.0);
                    rightFeeder.setPower(-1.0);
                }

                if (timer.seconds() > 4.5) { // Give it plenty of time
                    shooterMotor.setPower(0);
                    intakeMotor.setPower(0);
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);

                    follower.followPath(moveBackRight);
                    setPathState(1);
                }
                break;

            case 1: // Wait for move
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2:
                telemetry.addLine("Autonomous Complete");
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
    }
}
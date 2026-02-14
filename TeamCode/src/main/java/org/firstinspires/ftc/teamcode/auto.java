package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// Verified Imports
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path; // Use the pathgen Path class yum
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



@Autonomous(name = "Pedro_Auto_Path_Fixed", group = "FinalCodePedro")
public class auto extends LinearOpMode {

    private Follower follower;
    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;

    @Override
    public void runOpMode() {
        // Initialize Follower
        follower = Constants.createFollower(hardwareMap);

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        // 1. Define Poses
        Pose startPose = new Pose(0, 0, 0);
        Pose shootPose = new Pose(-24, 0, 0);

        follower.setStartingPose(startPose);

        // 2. Create the BezierLine as an Array to satisfy the constructor
        // If this still gives an error, we will try the individual coordinates method next.
        BezierLine line = new BezierLine(startPose, shootPose);
        // 3. Wrap it in a Path object
        Path movePath = new Path(line);

        // 4. Build the PathChain
        PathChain moveBackward = follower.pathBuilder()
                .addPath(movePath)
                .setConstantHeadingInterpolation(0)
                .build();

        waitForStart();

        // Start movement
        follower.followPath(moveBackward);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }

        // Shooting Sequence
        launcher.setVelocity(1200);
        sleep(2000);
        leftFeeder.setPower(-1.0);
        rightFeeder.setPower(1.0);
        sleep(2000);
        launcher.setPower(0);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }
}
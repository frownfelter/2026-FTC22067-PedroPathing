package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
public class redClose extends OpMode {

    private Follower follower;
    private Timer pathTimer, OpModeTimer;
    private DcMotor outtake = null;
    private DcMotor intake = null;

    public enum PathState {
        //START POS -> END POS
        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD
    }

    PathState pathState;

    private final Pose startPose = new Pose (104.256, 8.352000000000007, Math.toRadians(90));
    private final Pose shootPose = new Pose (86.68800000000002, 94.17600000000002, Math.toRadians(44));//TURN MORE LEFT NEXT TIME

    private PathChain driveStartPosShootPos;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate () {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(driveStartPosShootPos, true);
                outtake.setPower(.8);
                //pathState = PathState.SHOOT_PRELOAD;
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    telemetry.addLine("Done Path 1");
                }
                break;

            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    public void setPathState (PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {

        outtake = hardwareMap.get(DcMotor.class, "OUT");
        outtake.setDirection(DcMotor.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "IN");
        intake.setDirection(DcMotor.Direction.REVERSE);

        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;
        pathTimer = new Timer();
        //OpModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        //OpModeTimer.resetTimer();
        setPathState(pathState);
    }

    public void loop() {

        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time", pathTimer.getElapsedTimeSeconds());

    }
}

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
public class blueFar extends OpMode {

    private Follower follower;
    private Timer pathTimer, OpModeTimer;
    private DcMotor outtake = null;

    //private DcMotor outtake2 = null;
    private DcMotor intake = null;

    public enum PathState {
        //START POS -> END POS
        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_INPOS
    }

    PathState pathState;

    private final Pose startPose = new Pose (19.295999999999992, 121.76400000000007, Math.toRadians(144));
    private final Pose shootPose = new Pose (48.096, 105.03599999999999, Math.toRadians(148));//TURN MORE LEFT NEXT TIME
    private final Pose inPose = new Pose (52.75199999999999,83.49600000000002, Math.toRadians(180));

    private PathChain driveStartPosShootPos, driveShootPosInPos;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosInPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, inPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), inPose.getHeading())
                .build();
    }

    public void statePathUpdate () {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(driveStartPosShootPos, true);
                outtake.setPower(0.75);
                //outtake2.setPower(.75);
                //pathState = PathState.SHOOT_PRELOAD;
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    intake.setPower(1);
                }
                if (pathTimer.getElapsedTimeSeconds() > 8) {
                    follower.followPath(driveShootPosInPos, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_INPOS);
                    intake.setPower(0);
                    outtake.setPower(0);
                }
                break;
            case DRIVE_SHOOTPOS_INPOS:
                if (!follower.isBusy()) {
                    telemetry.addLine("Done all Paths");
                }

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

        //outtake2 = hardwareMap.get(DcMotor.class, "OUT2");
        outtake2.setDirection(DcMotor.Direction.FORWARD);

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

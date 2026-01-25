package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// You would need to import your specific OTOS library class here, e.g.:
// import com.sparkfun.otos.OTOS_Sensor;
// import com.sparkfun.otos.Pose2D;

/**
 * Mecanum Drive TeleOp Code with SparkFun OTOS Integration Placeholder
 */
@TeleOp(name="Sparkfun Test", group="Linear OpMode")
public class SparkfunTest extends LinearOpMode {

    // --- 1. Declare Hardware ---
    private DcMotor frontL = null;
    private DcMotor frontR = null;
    private DcMotor backL = null;
    private DcMotor backR = null;

    // Placeholder for the OTOS Sensor object
    // You would replace this with the actual OTOS class type: OTOS_Sensor sensor_otos = null;
    private Object sensor_otos = null;

    // Constants for Drive Adjustment
    private static final double DRIVE_SPEED_SCALE = 1.0; // Max speed (1.0 = 100%)

    @Override
    public void runOpMode() {

        // --- 2. Hardware Mapping ---
        frontL = hardwareMap.get(DcMotor.class, "frontL");
        frontR = hardwareMap.get(DcMotor.class, "frontR");
        backL = hardwareMap.get(DcMotor.class, "backL");
        backR = hardwareMap.get(DcMotor.class, "backR");

        // --- 3. Motor Setup ---
        // Reverse one side (usually left) to ensure motors drive forward when set to positive power
        frontL.setDirection(DcMotor.Direction.REVERSE);
        backL.setDirection(DcMotor.Direction.REVERSE);
        frontR.setDirection(DcMotor.Direction.FORWARD);
        backR.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to run without encoders for smoother TeleOp control
        frontL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Ensure motors stop when power is set to 0.0
        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- 4. OTOS Sensor Initialization (Placeholder) ---
        // In a real implementation, you would initialize the sensor here,
        // passing its I2C port reference from the hardware map.
        // Example: sensor_otos = hardwareMap.get(OTOS_Sensor.class, "sensor_otos");
        // sensor_otos.calibrate(); // Typically needed once at the start

        telemetry.addData("Status", "Initialized. Waiting for Start.");
        telemetry.update();

        waitForStart(); // Wait for the game to start (driver presses PLAY)

        // Reset the OTOS system to 0,0,0 at the start of the OpMode
        // sensor_otos.resetPose();

        // --- 5. Main Control Loop ---
        while (opModeIsActive()) {

            // --- A. Get Gamepad Inputs ---
            // Y-axis is inverted on the gamepad, so multiply by -1 for positive = forward
            double drive = -gamepad1.left_stick_y;  // Forward/Reverse movement
            double strafe = gamepad1.left_stick_x;  // Left/Right strafe movement
            double turn = gamepad1.right_stick_x;   // Rotation/Turning movement

            // --- B. Mecanum Kinematics Calculation ---
            // Calculate the power for each motor based on the inputs (vector addition)
            double frontLPower = drive + strafe + turn;
            double frontRPower = drive - strafe - turn;
            double backLPower = drive - strafe + turn;
            double backRPower = drive + strafe - turn;

            // --- C. Normalization and Scaling ---
            // Find the largest motor power (absolute value)
            double maxPower = Math.max(Math.abs(frontLPower), Math.abs(frontRPower));
            maxPower = Math.max(maxPower, Math.abs(backLPower));
            maxPower = Math.max(maxPower, Math.abs(backRPower));

            // Normalize all powers if any is over 1.0, and apply the speed scale
            if (maxPower > 1.0) {
                frontL.setPower((frontLPower / maxPower) * DRIVE_SPEED_SCALE);
                frontR.setPower((frontRPower / maxPower) * DRIVE_SPEED_SCALE);
                backL.setPower((backLPower / maxPower) * DRIVE_SPEED_SCALE);
                backR.setPower((backRPower / maxPower) * DRIVE_SPEED_SCALE);
            } else {
                frontL.setPower(frontLPower * DRIVE_SPEED_SCALE);
                frontR.setPower(frontRPower * DRIVE_SPEED_SCALE);
                backL.setPower(backLPower * DRIVE_SPEED_SCALE);
                backR.setPower(backRPower * DRIVE_SPEED_SCALE);
            }

            // --- D. OTOS Telemetry Update (Placeholder) ---
            // In a real implementation, you would call the OTOS sensor to get the current pose
            // Pose2D pose = sensor_otos.getPose();

            telemetry.addData("Status", "Running");
            telemetry.addData("Front Motors", "L: %.2f R: %.2f", frontL.getPower(), frontR.getPower());
            telemetry.addData("Back Motors", "L: %.2f R: %.2f", backL.getPower(), backR.getPower());

            // Uncomment and replace placeholders when you have the OTOS library imported:
            // telemetry.addData("Robot Pose (X, Y, Heading)", "%.2f, %.2f, %.2f", pose.x, pose.y, Math.toDegrees(pose.heading));

            telemetry.update();
        }
    }
}
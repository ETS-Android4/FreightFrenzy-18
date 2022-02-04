package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


@TeleOp
public class BlueMiniAutoTrial extends LinearOpMode {
    public robotInit robot = new robotInit();
    ElapsedTime runtime = new ElapsedTime();

//    int armPosition; //for arm position detection during teleop
//
//    //OpenCvCamera webcam;
//    webcamtest.DeterminationPipeline pipeline; //pipeline = series of img coming through camera to process


    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
//        boolean spinnyWheely = false;

        boolean freightSnatcher1on = true;
        //boolean freightSnatcher2on = false;

//        robot.freightSnatcher1.resetDeviceConfigurationForOpMode();
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
//
//        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//
//        pipeline = new webcamtest.DeterminationPipeline();
//        webcam.setPipeline(pipeline);
//
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//
//        });


        waitForStart();

        while (opModeIsActive()) {

            double vertical = 0.55 * (gamepad1.left_stick_y); //move forward, backward
            double turn = 0.55 * (gamepad1.right_stick_x); //turn left, right
            double horizontal = 0.55 * (gamepad1.left_stick_x); //move left, right
            double arm_control = 0.5 * (gamepad1.left_trigger - gamepad1.right_trigger); //arm up,down


            //fudge isn't being used
            //double fudge = 0.25; //TODO decrease speed with fudge factor/power curve so movements are not as jerky

            //Driving for mecanum wheels
            robot.motorFL.setPower(vertical - horizontal - turn);
            robot.motorFR.setPower(vertical + horizontal + turn);
            robot.motorBL.setPower(vertical + horizontal - turn);
            robot.motorBR.setPower(vertical - horizontal + turn);

//            //Driving for rhino (plain, flat) wheels
//            robot.motorFL.setPower(vertical - turn);
//            robot.motorFR.setPower(vertical + turn); //if vertical = 0, turn=-1, then vertical - turn = forward
//            robot.motorBL.setPower(vertical - turn);
//            robot.motorBR.setPower(vertical + turn);


            if (gamepad1.left_bumper) {
                robot.freightSnatcher1.setPower(-1); //vacuum takes in freight
            } else if (gamepad1.right_bumper) { //vacuum spews out freight
                robot.freightSnatcher1.setPower(1); //vacuum spews out freight
            } else {
                robot.freightSnatcher1.setPower(0.0); //vacuum stops

            }


//            //arm control for elbow and arm lift motors

            robot.elbowMotor.setPower(arm_control);
            robot.armLift.setPower(arm_control);


            if (gamepad1.x) { //spin carousel wheel
                robot.spinnyThing.setPower(0.75);
            } else {
                robot.spinnyThing.setPower(0);
            }

            //mini auto for high level
            if (gamepad1.dpad_right) {

                raise(-150);
                moveForward(15);
                // place freight on hub
                robot.freightSnatcher1.setPower(-1); //vacuum spews out freight
                runtime.reset();
                while (runtime.seconds() < 2.5) {
                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                    telemetry.update();
                }
                robot.freightSnatcher1.setPower(0); //vacuum stops
                moveBackward(15);
                lower(150);
                turnLeft(19);

            }


            //mini auto for low level and shared hub
            if (gamepad1.dpad_down) {
                raise(-65);
            }
        }
    }

    // FUNCTION TO TURN Right
    public void turnRight(double inches) {
        int newmotorFLTarget;
        int newmotorFRTarget;
        int newmotorBLTarget;
        int newmotorBRTarget;

        // Determine new target position, and pass to motor controller
        newmotorFLTarget = robot.motorFL.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
        newmotorFRTarget = robot.motorFR.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
        newmotorBLTarget = robot.motorBL.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
        newmotorBRTarget = robot.motorBR.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);

        robot.motorFL.setTargetPosition(newmotorFLTarget);
        robot.motorFR.setTargetPosition(newmotorFRTarget);
        robot.motorBL.setTargetPosition(newmotorBLTarget);
        robot.motorBR.setTargetPosition(newmotorBRTarget);

        // Turn On RUN_TO_POSITION
        // robot moves to set position
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
        runtime.reset();
        while (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy()) {
            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
            telemetry.update();
        }
    }


    // FUNCTION TO TURN LEFT
    public void turnLeft(double inches) {
        int newmotorFLTarget;
        int newmotorFRTarget;
        int newmotorBLTarget;
        int newmotorBRTarget;

        // Determine new target position, and pass to motor controller
        newmotorFLTarget = robot.motorFL.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
        newmotorFRTarget = robot.motorFR.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
        newmotorBLTarget = robot.motorBL.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
        newmotorBRTarget = robot.motorBR.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);

        robot.motorFL.setTargetPosition(newmotorFLTarget);
        robot.motorFR.setTargetPosition(newmotorFRTarget);
        robot.motorBL.setTargetPosition(newmotorBLTarget);
        robot.motorBR.setTargetPosition(newmotorBRTarget);

        // Turn On RUN_TO_POSITION
        // robot moves to set position
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
        runtime.reset();
        while (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy()) {
            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
            telemetry.update();
        }
    }


    // FUNCTION TO MOVE BACKWARD
    public void moveBackward(double inches) {
        int newmotorFLTarget;
        int newmotorFRTarget;
        int newmotorBLTarget;
        int newmotorBRTarget;

        // Determine new target position, and pass to motor controller
        newmotorFLTarget = robot.motorFL.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
        newmotorFRTarget = robot.motorFR.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
        newmotorBLTarget = robot.motorBL.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);
        newmotorBRTarget = robot.motorBR.getCurrentPosition() + (int) (inches * robot.COUNTS_PER_INCH);

        robot.motorFL.setTargetPosition(newmotorFLTarget);
        robot.motorFR.setTargetPosition(newmotorFRTarget);
        robot.motorBL.setTargetPosition(newmotorBLTarget);
        robot.motorBR.setTargetPosition(newmotorBRTarget);

        // Turn On RUN_TO_POSITION
        // robot moves to set position
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
        runtime.reset();
        while (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy()) {
            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
            telemetry.update();
        }
    }


    // FUNCTION TO MOVE FORWARD
    public void moveForward(double inches) {
        int newmotorFLTarget;
        int newmotorFRTarget;
        int newmotorBLTarget;
        int newmotorBRTarget;

        // Determine new target position, and pass to motor controller
        newmotorFLTarget = robot.motorFL.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
        newmotorFRTarget = robot.motorFR.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
        newmotorBLTarget = robot.motorBL.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
        newmotorBRTarget = robot.motorBR.getCurrentPosition() - (int) (inches * robot.COUNTS_PER_INCH);
        robot.motorFL.setTargetPosition(newmotorFLTarget);
        robot.motorFR.setTargetPosition(newmotorFRTarget);
        robot.motorBL.setTargetPosition(newmotorBLTarget);
        robot.motorBR.setTargetPosition(newmotorBRTarget);

        // Turn On RUN_TO_POSITION
        // robot moves to set position
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
        runtime.reset();
        while (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy()) {
            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
            telemetry.update();
        }
    }


    // ENCODER FUNCTIONS
    public void resetEncoder() {
        robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void startEncoderMode() {
        //Set Encoder Mode
        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    //RAISE ARM FUNCTION
    public void raise(double count) {

        int Target1;
        int Target2;

        // Determine new target position, and pass to motor controller
        Target1 = robot.elbowMotor.getCurrentPosition() + (int) (count);
        Target2 = robot.armLift.getCurrentPosition() + (int) (count);
        robot.elbowMotor.setTargetPosition(Target1);
        robot.armLift.setTargetPosition(Target2);

        // Turn On RUN_TO_POSITION
        robot.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.elbowMotor.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.armLift.setPower(Math.abs(robot.DRIVE_SPEED));

    }


    //LOWER ARM FUNCTION
    public void lower(double count) {

        int newElbowMotorTarget;

        // Determine new target position, and pass to motor controller
        newElbowMotorTarget = robot.elbowMotor.getCurrentPosition() - (int) (count);
        robot.elbowMotor.setTargetPosition(newElbowMotorTarget);
        robot.armLift.setTargetPosition(newElbowMotorTarget);

        // Turn On RUN_TO_POSITION
        robot.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.elbowMotor.setPower(Math.abs(robot.DRIVE_SPEED));
        robot.armLift.setPower(Math.abs(robot.DRIVE_SPEED));

    }
}

//        if (gamepad1.dpad_right) {
//
//            while (armPosition < BlueMiniAutoTrial.DeterminationPipeline.ArmPosition.Down)
//            robot.elbowMotor.setPower(arm_control);
//            robot.armLift.setPower(arm_control);
//
//            if (pipeline.position == BlueMiniAutoTrial.DeterminationPipeline.ArmPosition.Down) {
//                telemetry.addData("Detected", "level 1!");
//                telemetry.update();
//
//                armPosition
//
//            } else if (BlueMiniAutoTrial.DeterminationPipeline.ArmPosition.Up) {
//                telemetry.addData("Detected", "level 2!");
//                telemetry.update();
//
//                armPosition = 2;
//
//            } else {
//                telemetry.addData("Detected", "level 3!");
//                telemetry.update();
//
//                armPosition = 3;
//
//            }
//        }
//    }
//}
//
//
//    public static class DeterminationPipeline extends OpenCvPipeline {
//        public enum ArmPosition {
//            Down,
//            Up
//        }
//
//        // Some color constants
//        static final Scalar TURQUOISE = new Scalar(163, 210, 202);
//
//        // The core values which define the location and size of the sample regions
//        static final Point BOX1_TOPLEFT_ANCHOR_POINT = new Point(0, 200);
//
//        static final int REGION_WIDTH = 90;
//        static final int REGION_HEIGHT = 90;
//
//        final int UP_ARM_THRESHOLD = 150;
//        final int DOWN_ARM_THRESHOLD = 135;
//
//        Point region1_pointA = new Point(
//                BOX1_TOPLEFT_ANCHOR_POINT.x,
//                BOX1_TOPLEFT_ANCHOR_POINT.y);
//        Point region1_pointB = new Point(
//                BOX1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                BOX1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//
//        // Creating variables
//        Mat YCrCb = new Mat();
//        Mat Cb = new Mat();
//
//        //Box1
//        Mat region1_Cb;
//        int avg1;
//
//        // Volatile since accessed by OpMode thread w/o synchronization
//        public volatile BlueMiniAutoTrial.DeterminationPipeline.ArmPosition position = BlueMiniAutoTrial.DeterminationPipeline.ArmPosition.Down;
//
//        // This function takes the RGB frame, converts to YCrCb, and extracts the Cb channel to the 'Cb' variable
//        void inputToCb(Mat input) {
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//            Core.extractChannel(YCrCb, Cb, 1);
//        }
//
//        @Override
//        public void init(Mat firstFrame) {
//            inputToCb(firstFrame);
//
//            //Figure out how much blue is in each region
//            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
//
//        }
//
//        @Override
//        public Mat processFrame(Mat input) {
//            inputToCb(input);
//
//            avg1 = (int) Core.mean(region1_Cb).val[0];
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    TURQUOISE, // The color the rectangle is drawn in
//                    2); // Thickness of the rectangle lines
//
//
//            position = BlueMiniAutoTrial.DeterminationPipeline.ArmPosition.Down; // Record our analysis
//            //find the box/region with maximum red color
//            if (avg1 < UP_ARM_THRESHOLD) {
//                position = BlueMiniAutoTrial.DeterminationPipeline.ArmPosition.Up;
//            } else {
//                position = BlueMiniAutoTrial.DeterminationPipeline.ArmPosition.Down;
//            }
//
//            return input;
//        }
//
//        public int getAnalysis1() {
//            return avg1;
//        }

//            // Display it for the driver.
//            telemetry.addData("freightSnatcher1", freightSnatcher1on);
//            //telemetry.addData("freightSnatcher2", freightSnatcher2on);
//            telemetry.update();

//           // clamp servo movement
//            if (gamepad1.right_bumper) {
//                //clamps the servo that releases the freight
//                if (freightSnatcher1on) {
//                    robot.freightSnatcher1.setPosition(1);
//                    freightSnatcher1on = false;
//                    telemetry.addLine("servo1 clamped");
//                    telemetry.update();
//                }
//                //releases the servo that clamps the freight
//                else {
//                    robot.freightSnatcher1.setPosition(0.5);
//                    freightSnatcher1on = true;
//                    telemetry.addLine("servo1 released");
//                    telemetry.update();
//                }
//
//                runtime.reset();
//                while (opModeIsActive() && (runtime.seconds() < 1)) {
//                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//                    telemetry.update();
//                }
//            }



//            // wrist servo movement
//            if (gamepad1.dpad_up) {
//                // wrist servo goes up
//                double newPosition = robot.freightSnatcher2.getPosition() + 0.1;
//                robot.freightSnatcher2.setPosition(newPosition);
//
//                runtime.reset();
//                while (opModeIsActive() && (runtime.seconds() < 1)) {
//                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//                    telemetry.update();
//                }
//            }
//
//            if (gamepad1.dpad_down) {
//                // wrist servo goes down
//                double newPosition = robot.freightSnatcher2.getPosition() - 0.1;
//                robot.freightSnatcher2.setPosition(newPosition);
//
//                runtime.reset();
//                while (opModeIsActive() && (runtime.seconds() < 1)) {
//                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//                    telemetry.update();
//                }


//            if (gamepad1.right_bumper) { //clamp freight servo
//                //release the wrist servo
//                if (freightSnatcher2on) {
//                    robot.freightSnatcher2.setPosition(0.5);
//                    freightSnatcher2on = false;
//                    telemetry.addLine("servo2 released");
//                    telemetry.update();
//                }
//                //release the wrist servo
//                else {
//                    robot.freightSnatcher2.setPosition(1);
//                    freightSnatcher2on = true;
//                    telemetry.addLine("servo2 clamped");
//                    telemetry.update();
//                }
//
//                runtime.reset();
//                while (opModeIsActive() && (runtime.seconds() < 1)) {
//                    telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//                    telemetry.update();
//                }
//            }



//            //stop intake
//            if(gamepad1.left_bumper){
//                robot.intakeMotor.setPower(0);
//            }
//
//            //start intake
//            if(gamepad1.right_bumper){
//                robot.intakeMotor.setPower(0.9);
//            }

//        telemetry.addData("Status", "Running");
//        telemetry.addLine();
//        telemetry.update();




        /* FUNCTIONS */

//    public void raise(double count) {
//
//        int newElbowMotorTarget;
//
//        // Determine new target position, and pass to motor controller
//        newElbowMotorTarget = robot.elbowMotor.getCurrentPosition() + (int)(count);
//        robot.elbowMotor.setTargetPosition(newElbowMotorTarget);
//
//        // Turn On RUN_TO_POSITION
//        robot.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.elbowMotor.setPower(0.3);
//
//        runtime.reset();
//        while (opModeIsActive() && robot.elbowMotor.isBusy()) {
//            // Display it for the driver.
//            telemetry.addData("Path1",  "Running to %7d", newElbowMotorTarget);
//            telemetry.update();
//        }
//    }

//    public void lower(double count) {
//
//        int newElbowMotorTarget;
//
//        // Determine new target position, and pass to motor controller
//        newElbowMotorTarget = robot.elbowMotor.getCurrentPosition() - (int) (count);
//        robot.elbowMotor.setTargetPosition(newElbowMotorTarget);
//
//        // Turn On RUN_TO_POSITION
//        robot.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.elbowMotor.setPower(0.3);
//
//        runtime.reset();
//        while (opModeIsActive() && robot.elbowMotor.isBusy()) {
//            // Display it for the driver.
//            telemetry.addData("Path1",  "Running to %7d", newElbowMotorTarget);
//            telemetry.update();
//        }
//
//    }
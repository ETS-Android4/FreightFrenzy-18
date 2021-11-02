package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;



    @Autonomous(name="Auto", group="Pushbot")
    public class autonomous extends LinearOpMode {

//    OpenCvCamera webcam;

        public robotInit robot = new robotInit();
        ElapsedTime runtime = new ElapsedTime();


    /* Telemetry for testing ring detection
    while (opModeIsActive())
    {
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();

        // Don't burn CPU cycles busy-looping in this sample
        sleep(50);
    }
    */

//    SkystoneDeterminationPipeline pipeline; //pipline = series of img coming through camera to process

        @Override
        public void runOpMode() {

            robot.init(hardwareMap);

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
//
//        pipeline = new SkystoneDeterminationPipeline();
//        webcam.setPipeline(pipeline);
//
            resetEncoder();
            startEncoderMode();
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//        });


            // Wait for the game to start (driver presses PLAY)
            telemetry.addLine("Waiting for start");
            telemetry.update();
            waitForStart();

//        // Detect where the customized element is placed on the field
//        public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
//            public enum ElementPosition
//            {
//                Level1,
//                Level2,
//                Level3
//            }
//            // Some color constants
//            static final Scalar GREEN = new Scalar(57, 255, 20);
//
//            // The core values which define the location and size of the sample regions
//            static final Point BOX1_TOPLEFT_ANCHOR_POINT = new Point(85,183);
//            static final Point BOX2_TOPLEFT_ANCHOR_POINT = new Point(85,183);
//            static final Point BOX3_TOPLEFT_ANCHOR_POINT = new Point(85,183);
//
//            static final int REGION_WIDTH = 50;
//            static final int REGION_HEIGHT = 50;
//        }
//
//
//
//            static final int REGION_WIDTH = 50;
//            static final int REGION_HEIGHT = 50;
//
//            final int FOUR_RING_THRESHOLD = 135; //143-148
//            final int ONE_RING_THRESHOLD = 125; //119-131
//
//            Point region1_pointA = new Point(
//                    REGION1_TOPLEFT_ANCHOR_POINT.x,
//                    REGION1_TOPLEFT_ANCHOR_POINT.y);
//            Point region1_pointB = new Point(
//                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//            /*
//             * Working variables
//             */
//            Mat region1_Cb;
//            Mat YCrCb = new Mat();
//            Mat Cb = new Mat();
//            int avg1;
//
//            // Volatile since accessed by OpMode thread w/o synchronization
//            public volatile org.firstinspires.ftc.teamcode.autonomous.SkystoneDeterminationPipeline.RingPosition position = org.firstinspires.ftc.teamcode.autonomous.SkystoneDeterminationPipeline.RingPosition.FOUR;
//
//            /*
//             * This function takes the RGB frame, converts to YCrCb,
//             * and extracts the Cb channel to the 'Cb' variable
//             */
//            void inputToCb(Mat input)
//            {
//                Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//                Core.extractChannel(YCrCb, Cb, 1);
//            }
//
//            @Override
//            public void init(Mat firstFrame)
//            {
//                inputToCb(firstFrame);
//
//                region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
//            }
//
//            @Override
//            public Mat processFrame(Mat input)
//            {
//                inputToCb(input);
//
//                avg1 = (int) Core.mean(region1_Cb).val[0];
//
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region1_pointA, // First point which defines the rectangle
//                        region1_pointB, // Second point which defines the rectangle
//                        BLUE, // The color the rectangle is drawn in
//                        2); // Thickness of the rectangle lines
//
//                position = org.firstinspires.ftc.teamcode.autonomous.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
//                if(avg1 > FOUR_RING_THRESHOLD){
//                    position = org.firstinspires.ftc.teamcode.autonomous.SkystoneDeterminationPipeline.RingPosition.FOUR;
//                }else if (avg1 > ONE_RING_THRESHOLD){
//                    position = org.firstinspires.ftc.teamcode.autonomous.SkystoneDeterminationPipeline.RingPosition.ONE;
//                }else{
//                    position = org.firstinspires.ftc.teamcode.autonomous.SkystoneDeterminationPipeline.RingPosition.NONE;
//                }
//
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region1_pointA, // First point which defines the rectangle
//                        region1_pointB, // Second point which defines the rectangle
//                        GREEN, // The color the rectangle is drawn in
//                        1); // Negative thickness means solid fill
//
//                return input;
//            }
//
//            public int getAnalysis()
//            {
//                return avg1;
//            }

            // Level 1 (left sticker) - corresponds to bottom level on alliance shipping hub
            // Level 2 (middle sticker) - corresponds to middle level on alliance shipping hub
            // Level 3 (right sticker) - corresponds to top level on alliance shipping hub


//        Delivering a duck on the carousel

            // Navigate toward carousel

            // Turn left
            turnLeft(15);
            telemetry.addLine("turning left");
            telemetry.update();

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 2.0)) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Move forward
            moveForward(30);
            telemetry.addLine("moving forward");
            telemetry.update();

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 2.0)) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Turn right
            turnRight(15);
            telemetry.addLine("turning right");
            telemetry.update();

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 2.0)) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Go forward toward carousel
            moveForward(30);
            telemetry.addLine("moving forward");
            telemetry.update();

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 2.0)) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Touch wheel to carousel in order to spin it and spin the duck off
            robot.spinnyThing.setPower(0.25);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 8.0)) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }


            // Preloaded block placed marked level (by custom element) on alliance shipping hub

            // Navigate to alliance hub
            // Move forward toward alliance hub
            // Turn toward hub


            // Place freight on alliance hub

            // Level 1 scenario
            // Lower arm
            // Drop block on bottom level


            // Level 2 scenario
            // Bring arm forward
            // Drop block on middle level

            // Level 3 scenario
            // Bring arm up
            // Drop block on top level

        }

        // FUNCTION TO TURN LEFT
        public void turnLeft(double inches) {
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
            while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
                telemetry.update();
            }
        }



        // FUNCTION TO TURN RIGHT
        public void turnRight(double inches) {
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
            while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
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
            while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
                telemetry.update();
            }
        }


        /* ENCODER FUNCTIONS */
        public void resetEncoder()
        {
            robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        public void startEncoderMode()
        {
            //Set Encoder Mode
            robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

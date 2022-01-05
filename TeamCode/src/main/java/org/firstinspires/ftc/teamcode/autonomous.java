package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


    @Autonomous(name="Auto", group="Pushbot")
    public class autonomous extends LinearOpMode {

        public robotInit robot = new robotInit();
        ElapsedTime runtime = new ElapsedTime();

        OpenCvCamera webcam;
        DeterminationPipeline pipeline; //pipeline = series of img coming through camera to process

        @Override
        public void runOpMode() {

            robot.init(hardwareMap);

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

            pipeline = new DeterminationPipeline();
            webcam.setPipeline(pipeline);

            resetEncoder();


            startEncoderMode();

//            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//                @Override
//                public void onOpened() {
//                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//                }
//            });


            //Telemetry for testing barcode detection
//            while (opModeIsActive()) {
//                telemetry.addData("Analysis1", pipeline.getAnalysis1());
//                telemetry.addData("Analysis2", pipeline.getAnalysis2());
//                telemetry.addData("Analysis3", pipeline.getAnalysis3());
//                telemetry.addData("Position", pipeline.position);
//                telemetry.update();
//
//                // Don't burn CPU cycles busy-looping in this sample
//                sleep(50);
//            }

            // Wait for the game to start (driver presses PLAY)
            telemetry.addLine("Waiting for start");
            telemetry.update();
            waitForStart();


//        STEP 2 - Delivering a duck on the carousel

            // Navigate toward carousel
            turnRight(23);
            moveForward(46);

            // Touch wheel to carousel in order to spin it
            robot.spinnyThing.setPower(0.6);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 14.0)) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }


            // STEP 3 - Preloaded block placed marked level (by custom element) on alliance shipping hub

            // Navigate to alliance hub

            // Turn toward hub
            turnLeft(30);
            // Move forward toward alliance hub
            moveForward(50);


            //find the box/region with maximum red color
//            if (pipeline.position == org.firstinspires.ftc.teamcode.autonomous.DeterminationPipeline.ElementPosition.Level1) {
//
//                // raise arm
//                raise(70);
//                // Drop block on bottom level
//                robot.freightSnatcher1.setPosition(0.3);
//
//            } else if (pipeline.position == org.firstinspires.ftc.teamcode.autonomous.DeterminationPipeline.ElementPosition.Level2) {
//
//                // Bring arm forward
//                raise(150);
//                robot.freightSnatcher2.setPosition(0.7);
//
//                runtime.reset();
//                while (opModeIsActive() && (runtime.seconds() < 1.0)) {
//                    telemetry.addData("Detected:", "Level2");
//                    telemetry.update();
//                }
//                // drop block on bottom level
//                robot.freightSnatcher1.setPosition(0.3);
//
//            } else {
//                // Bring arm forward
//                raise(250);
//                // Drop block on bottom level
//                robot.freightSnatcher2.setPosition(1);
//
//                runtime.reset();
//                while (opModeIsActive() && (runtime.seconds() < 1.0)) {
//                    telemetry.addData("Detected:", "Level3");
//                    telemetry.update();
//                }
//                robot.freightSnatcher1.setPosition(0.3);
//
//            }

            // STEP 4 - Park the robot in the warehouse
            turnLeft(13);
            moveForward(60);

        }



            // STEP 1 - Detect where the customized element is placed on the field
            public static class DeterminationPipeline extends OpenCvPipeline {
                public enum ElementPosition {
                    Level1,
                    Level2,
                    Level3
                }

                // Some color constants
                static final Scalar TURQUOISE = new Scalar(163, 210, 202);
                static final Scalar PINK = new Scalar(224, 119, 125);
                static final Scalar LILAC = new Scalar(130, 106, 237);

                // The core values which define the location and size of the sample regions
                static final Point BOX1_TOPLEFT_ANCHOR_POINT = new Point(30, 183);
                static final Point BOX2_TOPLEFT_ANCHOR_POINT = new Point(100, 183);
                static final Point BOX3_TOPLEFT_ANCHOR_POINT = new Point(170, 183);

                static final int REGION_WIDTH = 50;
                static final int REGION_HEIGHT = 50;

                Point region1_pointA = new Point(
                        BOX1_TOPLEFT_ANCHOR_POINT.x,
                        BOX1_TOPLEFT_ANCHOR_POINT.y);
                Point region1_pointB = new Point(
                        BOX1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                        BOX1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

                Point region2_pointA = new Point(
                        BOX2_TOPLEFT_ANCHOR_POINT.x,
                        BOX2_TOPLEFT_ANCHOR_POINT.y);
                Point region2_pointB = new Point(
                        BOX2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                        BOX2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

                Point region3_pointA = new Point(
                        BOX3_TOPLEFT_ANCHOR_POINT.x,
                        BOX3_TOPLEFT_ANCHOR_POINT.y);
                Point region3_pointB = new Point(
                        BOX3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                        BOX3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

                // Creating variables
                Mat YCrCb = new Mat();
                Mat Cb = new Mat();

                //Box1
                Mat region1_Cb;
                int avg1;

                //Box2
                Mat region2_Cb;
                int avg2;

                //Box3
                Mat region3_Cb;
                int avg3;

                // Volatile since accessed by OpMode thread w/o synchronization
                public volatile org.firstinspires.ftc.teamcode.autonomous.DeterminationPipeline.ElementPosition position = org.firstinspires.ftc.teamcode.autonomous.DeterminationPipeline.ElementPosition.Level1;

                // This function takes the RGB frame, converts to YCrCb, and extracts the Cb channel to the 'Cb' variable
                void inputToCb(Mat input) {
                    Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
                    Core.extractChannel(YCrCb, Cb, 1);
                }

                @Override
                public void init(Mat firstFrame) {
                    inputToCb(firstFrame);

                    //Figure out how much blue is in each region
                    region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
                    region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
                    region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
                }

                @Override
                public Mat processFrame(Mat input) {
                    inputToCb(input);

                    avg1 = (int) Core.mean(region1_Cb).val[0];
                    avg2 = (int) Core.mean(region2_Cb).val[0];
                    avg3 = (int) Core.mean(region3_Cb).val[0];

                    Imgproc.rectangle(
                            input, // Buffer to draw on
                            region1_pointA, // First point which defines the rectangle
                            region1_pointB, // Second point which defines the rectangle
                            TURQUOISE, // The color the rectangle is drawn in
                            2); // Thickness of the rectangle lines

                    Imgproc.rectangle(
                            input, // Buffer to draw on
                            region2_pointA, // First point which defines the rectangle
                            region2_pointB, // Second point which defines the rectangle
                            PINK, // The color the rectangle is drawn in
                            2); // Thickness of the rectangle lines

                    Imgproc.rectangle(
                            input, // Buffer to draw on
                            region3_pointA, // First point which defines the rectangle
                            region3_pointB, // Second point which defines the rectangle
                            LILAC, // The color the rectangle is drawn in
                            2); // Thickness of the rectangle lines

                    position = org.firstinspires.ftc.teamcode.autonomous.DeterminationPipeline.ElementPosition.Level1; // Record our analysis
                    //find the box/region with maximum red color
                    if (avg1 > avg2 && avg1 > avg3) {
                        position = org.firstinspires.ftc.teamcode.autonomous.DeterminationPipeline.ElementPosition.Level1;
                    } else if (avg2 > avg3) {
                        position = org.firstinspires.ftc.teamcode.autonomous.DeterminationPipeline.ElementPosition.Level2;
                    } else {
                        position = org.firstinspires.ftc.teamcode.autonomous.DeterminationPipeline.ElementPosition.Level3;
                    }

                    return input;
                }

                //TODO check to see if this works :)
                public int getAnalysis1() {
                    return avg1;
                }

                public int getAnalysis2() {
                    return avg2;
                }

                public int getAnalysis3() {
                    return avg3;
                }
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



        // FUNCTION TO MOVE BACKWARDS
        public void moveBackwards(double inches) {
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
            while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget);
                telemetry.update();
            }
        }



        // ENCODER FUNCTIONS
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



        //RAISE ARM FUNCTION
        public void raise(double count) {

            int newElbowMotorTarget;

            // Determine new target position, and pass to motor controller
            newElbowMotorTarget = robot.elbowMotor.getCurrentPosition() + (int) (count);
            robot.elbowMotor.setTargetPosition(newElbowMotorTarget);

            // Turn On RUN_TO_POSITION
            robot.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.elbowMotor.setPower(Math.abs(robot.DRIVE_SPEED));

        }



        //LOWER ARM FUNCTION
        public void lower(double count) {

            int newElbowMotorTarget;

            // Determine new target position, and pass to motor controller
            newElbowMotorTarget = robot.elbowMotor.getCurrentPosition() - (int) (count);
            robot.elbowMotor.setTargetPosition(newElbowMotorTarget);

            // Turn On RUN_TO_POSITION
            robot.elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.elbowMotor.setPower(Math.abs(robot.DRIVE_SPEED));

        }

//        public void (){
//            //move motor down
//            robot.elbowMotor.setPower(1);
//            runtime.reset();
//            while (runtime.seconds() < 0.2){
//            }
//
//            //unclamp servo
//            robot.elbowMotor.setPower(0);
//            robot.freightSnatcher1.setPosition(0.6);
//
//            //wait
//            runtime.reset();
//            while (runtime.seconds() < 1){
//            }
//
//            //clamp freightSnatcher
//            robot.freightSnatcher1.setPosition(0.3);
//
//            //move arm back up
//            robot.elbowMotor.setPower(-1);
//            runtime.reset();
//            while (runtime.seconds() < 0.2){
//            }
//            robot.elbowMotor.setPower(0);
//        }
    }
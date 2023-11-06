/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.vision;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AprilTags {
    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    // FOR GETTERS
    int getterId = 0;
    double xGetter = 0.0;
    double yGetter = 0.0;
    double zGetter = 0.0;
    double yawGetter = 0.0;
    double pitchGetter = 0.0;
    double rollGetter = 0.0;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    // UNITS ARE METERS
    double tagsize = 0.166;
    AprilTagDetection tagOfInterest = null;

    int[] ID_TAG_OF_INTEREST = {440,373, 182};

    public void initCamera(OpenCvCamera cam) {
        camera = cam;
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {

            }
        });
    }

    public void updateTags() {
        // Calling getDetectionsUpdate() will only return an object if there was a new frame
        // processed since the last time we called it. Otherwise, it will return null. This
        // enables us to only run logic when there has been a new frame, as opposed to the
        // getLatestDetections() method which will always return an object.
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

        // If there's been a new frame...
        if (detections != null) {
            /*

            // If we don't see any tags
            if (detections.size() == 0) {
                numFramesWithoutDetection++;

                // If we haven't seen a tag for a few frames, lower the decimation
                // so we can hopefully pick one up if we're e.g. far back
                if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                }
            }
            // We do see tags!
            else {
                numFramesWithoutDetection = 0;

                // If the target is within 1 meter, turn on high decimation to
                // increase the frame rate
                if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                }

                for (AprilTagDetection detection : detections) {
                    this.getterId = detection.id;
                    this.xGetter = detection.pose.x;
                    this.yGetter = detection.pose.y;
                    this.zGetter = detection.pose.z;
                    this.yawGetter = Math.toDegrees(detection.pose.yaw);
                    this.pitchGetter = Math.toDegrees(detection.pose.pitch);
                    this.rollGetter = Math.toDegrees(detection.pose.roll);
                }
            } */
            if(detections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : detections) {
                    for (int id : ID_TAG_OF_INTEREST) {
                        if (id == tag.id) {
                            tagOfInterest = tag;
                            this.getterId = tag.id;
                            this.xGetter = tag.pose.x;
                            this.yGetter = tag.pose.y;
                            this.zGetter = tag.pose.z;
                            this.yawGetter = Math.toDegrees(tag.pose.yaw);
                            this.pitchGetter = Math.toDegrees(tag.pose.pitch);
                            this.rollGetter = Math.toDegrees(tag.pose.roll);
                            break;
                        }
                    }
                }
            }
        }
    }

    // GETTERS
    public int idGetter() {
        return getterId;
    }

    public double[] position() {
        return new double[]{xGetter, yGetter, zGetter};
    }

    public double[] rotation() {
        return new double[] {yawGetter, pitchGetter, rollGetter};
    }
}
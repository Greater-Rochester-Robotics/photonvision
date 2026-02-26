/*
 * Copyright (C) Photon Vision.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package org.photonvision.vision.pipe.impl;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.common.logging.LogGroup;
import org.photonvision.common.logging.Logger;
import org.photonvision.estimation.TargetModel;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.vision.calibration.CameraCalibrationCoefficients;
import org.photonvision.vision.pipe.CVPipe;
import org.photonvision.vision.target.TrackedTarget;

/** Estimate the camera pose given multiple Apriltag observations */
public class MultiTargetPNPPipe
        extends CVPipe<
                List<TrackedTarget>,
                Optional<MultiTargetPNPResult>,
                MultiTargetPNPPipe.MultiTargetPNPPipeParams> {
    private static final Logger logger = new Logger(MultiTargetPNPPipe.class, LogGroup.VisionModule);

    private static final Map<Integer, Integer> pairs = new HashMap<>();

    static {
        // Hard-coded 2026 REBUILT AprilTag pairs
        pairs.put(2, 11);
        pairs.put(11, 2);
        pairs.put(3, 4);
        pairs.put(4, 3);
        pairs.put(5, 8);
        pairs.put(8, 5);
        pairs.put(9, 10);
        pairs.put(10, 9);
        pairs.put(13, 14);
        pairs.put(14, 13);
        pairs.put(15, 16);
        pairs.put(16, 15);
        pairs.put(18, 27);
        pairs.put(27, 18);
        pairs.put(19, 20);
        pairs.put(20, 19);
        pairs.put(21, 24);
        pairs.put(24, 21);
        pairs.put(25, 26);
        pairs.put(26, 25);
        pairs.put(29, 30);
        pairs.put(30, 29);
        pairs.put(31, 32);
        pairs.put(32, 31);
    }

    private boolean hasWarned = false;

    @Override
    protected Optional<MultiTargetPNPResult> process(List<TrackedTarget> targetList) {
        if (params == null
                || params.cameraCoefficients() == null
                || params.cameraCoefficients().getCameraIntrinsicsMat() == null
                || params.cameraCoefficients().getDistCoeffsMat() == null) {
            if (!hasWarned) {
                logger.warn(
                        "Cannot perform solvePNP an uncalibrated camera! Please calibrate this resolution...");
                hasWarned = true;
            }
            return Optional.empty();
        }

        return calculateCameraInField(targetList);
    }

    private Optional<MultiTargetPNPResult> calculateCameraInField(List<TrackedTarget> targetList) {
        // This method is modified to preemptively filter for AprilTag "pairs", to
        // ensure multi-target estimates are guaranteed to only include tags that
        // share the same point of interest, with the goal of preventing noise from
        // background tags affecting the accuracy of vision estimates.

        // All estimates returned by this method will only include the closest pair
        // of tags to the camera, with all other tags supplied in the targetList being
        // left unused.

        var targetArray = targetList.toArray(new TrackedTarget[targetList.size()]);
        Arrays.sort(
                targetArray,
                (a, b) -> {
                    double a_d2 = a.getBestCameraToTarget3d().getTranslation().getSquaredNorm();
                    double b_d2 = b.getBestCameraToTarget3d().getTranslation().getSquaredNorm();
                    return a_d2 == b_d2 ? 0 : a_d2 < b_d2 ? -1 : 1;
                });

        for (var target : targetArray) {
            int targetId = target.getFiducialId();
            Integer pairId = pairs.get(targetId);

            if (pairId != null) {
                for (var secondary : targetArray) {
                    if (secondary.getFiducialId() == pairId) {
                        var estimatedPose =
                                VisionEstimation.estimateCamPosePNP(
                                        params.cameraCoefficients().cameraIntrinsics.getAsWpilibMat(),
                                        params.cameraCoefficients().distCoeffs.getAsWpilibMat(),
                                        TrackedTarget.simpleFromTrackedTargets(List.of(target, secondary)),
                                        params.atfl(),
                                        params.targetModel());

                        if (estimatedPose.isPresent()) {
                            return Optional.of(
                                    new MultiTargetPNPResult(
                                            estimatedPose.get(), List.of((short) targetId, pairId.shortValue())));
                        } else {
                            break;
                        }
                    }
                }
            }
        }

        return Optional.empty();
    }

    public static record MultiTargetPNPPipeParams(
            CameraCalibrationCoefficients cameraCoefficients,
            AprilTagFieldLayout atfl,
            TargetModel targetModel) {}
}

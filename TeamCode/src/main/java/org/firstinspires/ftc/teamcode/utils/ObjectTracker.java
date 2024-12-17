package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import team.techtigers.core.paths.Waypoint;
import team.techtigers.core.paths.geometry.Point;
import team.techtigers.core.paths.geometry.Rectangle;

public class ObjectTracker {
    RobotState robotState;
    Rectangle currentBbox;
    Rectangle expectedBbox;
    List<Rectangle> lastDetections;
    Waypoint lastPoseWithDetection;
    double threshold;
    int consecutiveMisses;

    public ObjectTracker(Rectangle initialBbox, RobotState robotState) {
        this.robotState = robotState;
        this.currentBbox = initialBbox;
        lastDetections = new ArrayList<>();
        lastPoseWithDetection = robotState.getRobotCurrentPose();
        threshold = 20;
        consecutiveMisses = 0;
    }

    public void update(List<LLResultTypes.DetectorResult> detections) {
        ArrayList<Rectangle> detectionRects = new ArrayList<>();
        for (LLResultTypes.DetectorResult detection :  detections) {
            List<Double> topRight = detection.getTargetCorners().get(1);
            double trX = topRight.get(0);
            double trY = topRight.get(1);
            List<Double> bottomLeft = detection.getTargetCorners().get(1);
            double blX = topRight.get(0);
            double blY = topRight.get(1);
            detectionRects.add(new Rectangle(new Point(trX, trY), new Point(blX, blY)));
        }
        Point deltaPosition = robotState.getRobotCurrentPose().getPoint().minus(lastPoseWithDetection.getPoint());
//        List<Rectangle> translatedLastDetections = lastDetections.stream().map((rect -> rect.translate(deltaPosition))).collect(Collectors.toList());
        Rectangle translatedCurrentBbox = currentBbox.translate(deltaPosition);
        Optional<Rectangle> optionalClosestToTranslatedCurrentBbox = detectionRects.stream().min((rect1, rect2) -> (int) (compareRects(translatedCurrentBbox, rect1) - compareRects(translatedCurrentBbox, rect2)));
        Rectangle closestToTranslatedCurrentBbox;
        if (optionalClosestToTranslatedCurrentBbox.isPresent()) {
            closestToTranslatedCurrentBbox = optionalClosestToTranslatedCurrentBbox.get();
            if (compareRects(closestToTranslatedCurrentBbox, translatedCurrentBbox) < threshold) {
                currentBbox = closestToTranslatedCurrentBbox;
                lastPoseWithDetection = robotState.getRobotCurrentPose();
                consecutiveMisses = 0;
            } else {
                consecutiveMisses++;
            }
        } else {
            consecutiveMisses++;
        }
        lastDetections = detectionRects;
    }

    public Optional<Rectangle> getDetection() {
        return consecutiveMisses > 10 ? Optional.empty() : Optional.of(currentBbox);
    }

    private static double compareRects(Rectangle translatedCurrentBbox, Rectangle other) {
        return translatedCurrentBbox.getTopRight().dist(other.getTopRight()) + translatedCurrentBbox.getBottomLeft().dist(other.getBottomLeft());
//        double rect2Sum =  translatedCurrentBbox.getTopRight().dist(rect2.getTopRight()) + translatedCurrentBbox.getBottomLeft().dist(rect2.getBottomLeft());
//        return (int) (rect1Sum - rect2Sum);
    }
}


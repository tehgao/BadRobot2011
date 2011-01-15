/*----------------------------------------------------------------------------*
/* Copyright (c) FIRST 2008. All Rights Reserved.                             *
/* Open Source Software - may be modified and shared by FRC teams. The code   *
/* must be accompanied by the FIRST BSD license file in the root directory of *
/* the project.                                                               *
/*----------------------------------------------------------------------------*/
package code.badrobot;

import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;

/**
 *
 * @author dtjones
 */
public class Target {

    private static final double FRC_PARTICLE_TO_IMAGE_PERCENT = .0001;
    private static final double FRC_SIZE_FACTOR = 3.0;
    private static final double FRC_MAX_IMAGE_SEPARATION = 20.0;
    private static final double FRC_ALIGNMENT_SCALE = 3.0;
    final ParticleAnalysisReport firstParticle;
    final ParticleAnalysisReport secondParticle;

    Target(ParticleAnalysisReport firstParticle, ParticleAnalysisReport secondParticle) {
        this.firstParticle = firstParticle;
        this.secondParticle = secondParticle;
    }

    public static class Position {

        public final int value;
        protected static final int above_val = 0;
        protected static final int below_val = 1;
        protected static final int right_val = 2;
        protected static final int left_val = 3;
        public static final Position above = new Position(above_val);
        public static final Position bellow = new Position(below_val);
        public static final Position right = new Position(right_val);
        public static final Position left = new Position(left_val);

        private Position(int value) {
            this.value = value;
        }
    }

    public static class Threshold {

        int plane1Low;
        int plane1High;
        int plane2Low;
        int plane2High;
        int plane3Low;
        int plane3High;

        public Threshold(int plane1Low, int plane1High,
                int plane2Low, int plane2High,
                int plane3Low, int plane3High) {
            this.plane1Low = plane1Low;
            this.plane1High = plane1High;
            this.plane2Low = plane2Low;
            this.plane2High = plane2High;
            this.plane3Low = plane3Low;
            this.plane3High = plane3High;
        }
    }

    private static boolean aligned(int center1, int center2, int dimension1, int dimension2) {
        double averageWidth = (dimension1 + dimension2) / 2.0;
        //scale down width
        averageWidth *= FRC_ALIGNMENT_SCALE;
        int centerDiff = Math.abs(center1 - center2);
        if (centerDiff < averageWidth) {
            return true;
        }
        //dimensions (widths or heights) should be nearly the same. If they are
        //different, most likely there is glare or incorrect color specification
        return false;
    }

    private static boolean adjacent(int value1, int value2) {
        if (Math.abs(value1 - value2) <= FRC_MAX_IMAGE_SEPARATION) {
            return true;
        }
        return false;
    }

    private static boolean sizesRelative(double area1, double area2) {
        if ((area2 < (area1 * FRC_SIZE_FACTOR)) && (area1 < (area2 * FRC_SIZE_FACTOR))) {
            return true;
        }
        return false;
    }

    /**
     * Search for a target in the given image of the two color ranges given and
     * positioned according to the given position.
     * @param image The image to fing the target within.
     * @param position The postion of the two colors realtive to eachother.
     * @param firstThreshold The first color range.
     * @param secondThreshold The second color range.
     * @return A Target object containing information about the target or null
     * if no target was found.
     */
    public static Target getTarget(ColorImage image, Position position,
            Threshold firstThreshold, Threshold secondThreshold) throws NIVisionException{
        BinaryImage firstColor = image.thresholdHSL(
                firstThreshold.plane1Low, firstThreshold.plane1High,
                firstThreshold.plane2Low, firstThreshold.plane2High,
                firstThreshold.plane3Low, firstThreshold.plane3High);
        BinaryImage secondColor = image.thresholdHSL(
                secondThreshold.plane1Low, secondThreshold.plane1High,
                secondThreshold.plane2Low, secondThreshold.plane2High,
                secondThreshold.plane3Low, secondThreshold.plane3High);


        ParticleAnalysisReport[] firstColorHits = firstColor.getOrderedParticleAnalysisReports(3);
        ParticleAnalysisReport[] secondColorHits = secondColor.getOrderedParticleAnalysisReports(3);
        firstColor.free();
        secondColor.free();

        for (int i = 0; i < firstColorHits.length; i++) {
            ParticleAnalysisReport firstTrackReport = firstColorHits[i];
            if (firstTrackReport.particleToImagePercent < FRC_PARTICLE_TO_IMAGE_PERCENT) {
                break;
            }
            for (int j = 0; j < secondColorHits.length; j++) {
                ParticleAnalysisReport secondTrackReport = secondColorHits[j];
                if (secondTrackReport.particleToImagePercent < FRC_PARTICLE_TO_IMAGE_PERCENT) {
                    break;
                }
                switch (position.value) {
                    case Position.above_val:
                        if (secondTrackReport.center_mass_y < firstTrackReport.center_mass_y) {
                            // add in the SizesRelative call if needed -
                            // so far it does not seem necessary
                            if (aligned(firstTrackReport.center_mass_x,
                                    secondTrackReport.center_mass_x,
                                    firstTrackReport.boundingRectWidth,
                                    secondTrackReport.boundingRectWidth) &&
                                    adjacent(firstTrackReport.boundingRectTop,
                                    (secondTrackReport.boundingRectTop +
                                    secondTrackReport.boundingRectHeight)) &&
                                    sizesRelative(firstTrackReport.particleArea,
                                    secondTrackReport.particleArea)) {
                                //return the relevant track report
                                return new Target(firstTrackReport, secondTrackReport);
                            }
                        }
                        break;
                    case Position.below_val:
                        if (secondTrackReport.center_mass_y > firstTrackReport.center_mass_y) {
                            if (aligned(firstTrackReport.center_mass_x,
                                    secondTrackReport.center_mass_x,
                                    firstTrackReport.boundingRectWidth,
                                    secondTrackReport.boundingRectWidth) &&
                                    adjacent((firstTrackReport.boundingRectTop +
                                    firstTrackReport.boundingRectHeight),
                                    secondTrackReport.boundingRectTop)) {
                                return new Target(firstTrackReport, secondTrackReport);
                            }
                        }
                        break;
                    case Position.left_val:
                        if (secondTrackReport.center_mass_x < firstTrackReport.center_mass_x) {
                            if (aligned(firstTrackReport.center_mass_y,
                                    secondTrackReport.center_mass_y,
                                    firstTrackReport.boundingRectWidth,
                                    secondTrackReport.boundingRectWidth) &&
                                    adjacent(firstTrackReport.boundingRectLeft,
                                    (secondTrackReport.boundingRectLeft +
                                    secondTrackReport.boundingRectWidth))) {
                                return new Target(firstTrackReport, secondTrackReport);
                            }
                        }
                        break;
                    case Position.right_val:
                        if (secondTrackReport.center_mass_x > firstTrackReport.center_mass_x) {
                            if (aligned(firstTrackReport.center_mass_y,
                                    secondTrackReport.center_mass_y,
                                    firstTrackReport.boundingRectWidth,
                                    secondTrackReport.boundingRectWidth) &&
                                    adjacent((firstTrackReport.boundingRectLeft +
                                    secondTrackReport.boundingRectWidth),
                                    secondTrackReport.boundingRectLeft)) {
                                return new Target(firstTrackReport, secondTrackReport);
                            }
                        }
                        break;
                }
            }
        }
        return null;
    }

    public double getXPosition() {
        return (firstParticle.center_mass_x_normalized * firstParticle.particleToImagePercent
                + secondParticle.center_mass_x_normalized * secondParticle.particleToImagePercent) /
                getSize();
    }

    public double getYPosition() {
        return (firstParticle.center_mass_y_normalized * firstParticle.particleToImagePercent
                + secondParticle.center_mass_y_normalized * secondParticle.particleToImagePercent) /
                getSize();
    }

    public double getSize() {
        return firstParticle.particleToImagePercent + secondParticle.particleToImagePercent;
    }

    public String toString() {
        return "Target at ( " + getXPosition() + " , " + getYPosition() + " ) of size " + getSize();
    }

}

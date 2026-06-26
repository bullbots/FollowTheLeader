package frc.robot.targeting;

import com.fasterxml.jackson.dataformat.csv.CsvMapper;
import com.fasterxml.jackson.dataformat.csv.CsvSchema;

import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.databind.MappingIterator;

public class InterpolationTables {
    public static class ShooterData {
        public double distance;
        public double hoodAngle;
        public double shooterSpeed;
    }

    private List<ShooterData> leftData = new ArrayList<>();
    private List<ShooterData> rightData = new ArrayList<>();

    public InterpolationTables() {
        // load left shooter data
        try {
            CsvMapper csvMapper = new CsvMapper();
            CsvSchema schema = CsvSchema.emptySchema().withHeader();
            MappingIterator<ShooterData> iterator = csvMapper
                    .readerFor(ShooterData.class)
                    .with(schema)
                    .readValues(new File(Filesystem.getDeployDirectory(), "LeftShooterData.csv"));
            leftData = iterator.readAll();
        } catch (IOException e) {
            System.err.println("Failed to load interpolation data: " + e.getMessage());
        }
        System.out.println("Loaded " + leftData.size() + " left shooter data points.");
        // Load right shooter data
        try {
            CsvMapper csvMapper = new CsvMapper();
            CsvSchema schema = CsvSchema.emptySchema().withHeader();
            MappingIterator<ShooterData> iterator = csvMapper
                    .readerFor(ShooterData.class)
                    .with(schema)
                    .readValues(new File(Filesystem.getDeployDirectory(), "RightShooterData.csv"));
            rightData = iterator.readAll();
        } catch (IOException e) {
            System.err.println("Failed to load interpolation data: " + e.getMessage());
        }
        System.out.println("Loaded " + rightData.size() + " right shooter data points.");
    }

    public List<ShooterData> getLeftData() {
        return leftData;
    }

    public List<ShooterData> getRightData() {
        return rightData;
    }

    public double getLeftInterpolatedHoodAngle(double distance) {
        if (leftData.isEmpty()) {
            return 0;
        }
        ShooterData lower = null, upper = null;
        for (ShooterData d : leftData) {
            if (d.distance <= distance) {
                lower = d;
            }
            if (d.distance >= distance) {
                upper = d;
                break;
            }
        }
        if (lower == null) {
            return leftData.get(0).hoodAngle;
        }
        if (upper == null) {
            return leftData.get(leftData.size() - 1).hoodAngle;
        }
        if (upper.distance == lower.distance) {
            return lower.hoodAngle; // Avoid division by zero if distances are the same
        }
        double ratio = (distance - lower.distance) / (upper.distance - lower.distance);
        return lower.hoodAngle + ratio * (upper.hoodAngle - lower.hoodAngle);
    }

    public double getLeftInterpolatedShooterSpeed(double distance) {
        if (leftData.isEmpty()) {
            return 0;
        }
        ShooterData lower = null, upper = null;
        for (ShooterData d : leftData) {
            if (d.distance <= distance) {
                lower = d;
            }
            if (d.distance >= distance) {
                upper = d;
                break;
            }
        }
        if (lower == null) {
            return leftData.get(0).shooterSpeed;
        }
        if (upper == null) {
            return leftData.get(leftData.size() - 1).shooterSpeed;
        }
        if (upper.distance == lower.distance) {
            return lower.shooterSpeed; // Avoid division by zero if distances are the same
        }
        double ratio = (distance - lower.distance) / (upper.distance - lower.distance);
        return lower.shooterSpeed + ratio * (upper.shooterSpeed - lower.shooterSpeed);
    }

    public double getRightInterpolatedHoodAngle(double distance) {
        if (rightData.isEmpty()) {
            return 0;
        }
        ShooterData lower = null, upper = null;
        for (ShooterData d : rightData) {
            if (d.distance <= distance) {
                lower = d;
            }
            if (d.distance >= distance) {
                upper = d;
                break;
            }
        }
        if (lower == null) {
            return rightData.get(0).hoodAngle;
        }
        if (upper == null) {
            return rightData.get(rightData.size() - 1).hoodAngle;
        }
        if (upper.distance == lower.distance) {
            return lower.hoodAngle; // Avoid division by zero if distances are the same
        }
        double ratio = (distance - lower.distance) / (upper.distance - lower.distance);
        return lower.hoodAngle + ratio * (upper.hoodAngle - lower.hoodAngle);
    }

    public double getRightInterpolatedShooterSpeed(double distance) {
        if (rightData.isEmpty()) {
            return 0;
        }
        ShooterData lower = null, upper = null;
        for (ShooterData d : rightData) {
            if (d.distance <= distance) {
                lower = d;
            }
            if (d.distance >= distance) {
                upper = d;
                break;
            }
        }
        if (lower == null) {
            return rightData.get(0).shooterSpeed;
        }
        if (upper == null) {
            return rightData.get(rightData.size() - 1).shooterSpeed;
        }
        if (upper.distance == lower.distance) {
            return lower.shooterSpeed; // Avoid division by zero if distances are the same
        }
        double ratio = (distance - lower.distance) / (upper.distance - lower.distance);
        return lower.shooterSpeed + ratio * (upper.shooterSpeed - lower.shooterSpeed);
    }
}

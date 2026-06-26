package frc.robot.trench;

import java.io.File;
import java.util.List;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * Singleton for trench zone data.
 * Loads trench zone positions from deploy/trench_warfare/trench_zones.json.
 * Call {@link #initialize()} during robot startup to avoid parsing delays during competition.
 */
public class TrenchZones {

    private static TrenchZones instance;

    private final TrenchZonesData data;

    private TrenchZones() {
        File file = new File(Filesystem.getDeployDirectory(), "trench_warfare/trench_zones.json");
        try {
            ObjectMapper mapper = new ObjectMapper();
            this.data = mapper.readValue(file, TrenchZonesData.class);
        } catch (Exception e) {
            throw new RuntimeException("Failed to load trench zones from " + file.getAbsolutePath(), e);
        }
    }

    /**
     * Initializes the singleton instance eagerly.
     * Call this during robot startup (e.g., in RobotContainer constructor) to
     * avoid parsing delays during competition.
     */
    public static void initialize() {
        if (instance == null) {
            instance = new TrenchZones();
        }
    }

    /**
     * Gets the singleton instance.
     * Must call {@link #initialize()} first during robot startup.
     *
     * @throws IllegalStateException if initialize() was not called
     */
    public static TrenchZones getInstance() {
        if (instance == null) {
            throw new IllegalStateException("TrenchZones not initialized. Call TrenchZones.initialize() during robot startup.");
        }
        return instance;
    }

    /**
     * Gets all trench zones.
     */
    public List<TrenchZone> getZones() {
        return data.trenchZones;
    }

    /**
     * Gets a trench zone by index.
     */
    public TrenchZone getZone(int index) {
        return data.trenchZones.get(index);
    }

    /**
     * Gets the field length in meters.
     */
    public double getFieldLengthMeters() {
        return data.fieldLengthMeters;
    }

    /**
     * Gets the field width in meters.
     */
    public double getFieldWidthMeters() {
        return data.fieldWidthMeters;
    }

    /**
     * Checks if a robot pose is inside any trench's structure bounds.
     *
     * @param pose The robot's current pose
     * @return true if inside any trench structure
     */
    public boolean isInsideAnyTrench(Pose2d pose) {
        for (TrenchZone zone : data.trenchZones) {
            if (zone.isInsideStructure(pose.getTranslation())) {
                return true;
            }
        }
        return false;
    }

    /**
     * Gets the trench zone the robot is currently inside (by structure bounds), or null if not in any.
     *
     * @param pose The robot's current pose
     * @return The trench zone or null
     */
    public TrenchZone getTrenchAt(Pose2d pose) {
        for (TrenchZone zone : data.trenchZones) {
            if (zone.isInsideStructure(pose.getTranslation())) {
                return zone;
            }
        }
        return null;
    }

    // JSON data classes

    public static class TrenchZonesData {
        @JsonProperty("coordinate_system")
        public String coordinateSystem;

        @JsonProperty("field_length_meters")
        public double fieldLengthMeters;

        @JsonProperty("field_width_meters")
        public double fieldWidthMeters;

        @JsonProperty("trench_zones")
        public List<TrenchZone> trenchZones;
    }

    public static class TrenchZone {
        @JsonProperty("name")
        public String name;

        @JsonProperty("index")
        public int index;

        @JsonProperty("structure_bounds")
        public Bounds structureBounds;

        @JsonProperty("passable_opening")
        public PassableOpening passableOpening;

        @JsonProperty("rotation_degrees")
        public double rotationDegrees;

        /**
         * Checks if a translation is inside this trench's structure bounds.
         */
        public boolean isInsideStructure(Translation2d translation) {
            return structureBounds.contains(translation);
        }

        /**
         * Checks if a translation is inside this trench's passable opening.
         */
        public boolean isInsideOpening(Translation2d translation) {
            return passableOpening.contains(translation);
        }

        /**
         * Gets the center of the passable opening as a Translation2d.
         */
        public Translation2d getOpeningCenter() {
            return new Translation2d(passableOpening.centerX, passableOpening.centerY);
        }

        /**
         * Gets the center of the structure bounds as a Translation2d.
         */
        public Translation2d getStructureCenter() {
            return new Translation2d(structureBounds.centerX, structureBounds.centerY);
        }
    }

    public static class Bounds {
        @JsonProperty("min_x")
        public double minX;

        @JsonProperty("min_y")
        public double minY;

        @JsonProperty("max_x")
        public double maxX;

        @JsonProperty("max_y")
        public double maxY;

        @JsonProperty("center_x")
        public double centerX;

        @JsonProperty("center_y")
        public double centerY;

        @JsonProperty("width")
        public double width;

        @JsonProperty("depth")
        public double depth;

        @JsonProperty("height")
        public double height;

        public boolean contains(Translation2d point) {
            return point.getX() >= minX && point.getX() <= maxX
                    && point.getY() >= minY && point.getY() <= maxY;
        }
    }

    public static class PassableOpening {
        @JsonProperty("min_x")
        public double minX;

        @JsonProperty("min_y")
        public double minY;

        @JsonProperty("max_x")
        public double maxX;

        @JsonProperty("max_y")
        public double maxY;

        @JsonProperty("center_x")
        public double centerX;

        @JsonProperty("center_y")
        public double centerY;

        @JsonProperty("width")
        public double width;

        @JsonProperty("depth")
        public double depth;

        @JsonProperty("max_height_meters")
        public double maxHeightMeters;

        public boolean contains(Translation2d point) {
            return point.getX() >= minX && point.getX() <= maxX
                    && point.getY() >= minY && point.getY() <= maxY;
        }
    }
}

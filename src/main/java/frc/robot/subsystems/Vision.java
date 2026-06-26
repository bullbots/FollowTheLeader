
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;


/**
 * PhotonVision class to aid in accurate odometry with AprilTag vision.
 * Adapted from YAGSL-Example.
 */
public class Vision {

  /**
   * April Tag Field Layout of the year.
   */
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  /**
   * Ambiguity defined as a value between (0,1). Used in pose filtering.
   */
  private final double maximumAmbiguity = 0.25;

  /**
   * Photon Vision Simulation
   */
  public VisionSystemSim visionSim;

  /**
   * Count of times that the odom thinks we're more than 10 meters away from the april tag.
   */
  private double longDistancePoseEstimationCount = 0;

  /**
   * Whether the vision system has seen an AprilTag at least once since startup.
   * Pose estimation updates are blocked until this is true.
   */
  private boolean hasSeenAprilTag = false;

  /**
   * Current pose from the pose estimator using wheel odometry.
   */
  private Supplier<Pose2d> currentPose;

  /**
   * Field from {@link swervelib.SwerveDrive#field}
   */
  private Field2d field2d;

  private double YawAngle;


  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link SwerveDrive#field}
   */
  public Vision(Supplier<Pose2d> currentPose, Field2d field) {
    this.currentPose = currentPose;
    this.field2d = field;

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);

      for (Cameras c : Cameras.values()) {
        c.addToVisionSim(visionSim);
      }
    }
  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag    The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to the pose.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent()) {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }
  }

  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the given poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive) {
    if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
      /*
       * In maple-sim, odometry is simulated using encoder values, accounting for factors like skidding.
       * The vision system should provide accurate pose estimation even when odometry is incorrect.
       * Therefore, we must ensure that the actual robot pose is provided in the simulator.
       */
      visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
    }

    for (Cameras camera : Cameras.values()) {
      boolean connected = camera.camera.isConnected();
      boolean hasTargets = !camera.resultsList.isEmpty() && camera.resultsList.get(0).hasTargets();
      SmartDashboard.putBoolean("Vision/" + camera.name() + "/Connected", connected);
      SmartDashboard.putBoolean("Vision/" + camera.name() + "/HasTargets", hasTargets);

      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      SmartDashboard.putBoolean("Vision/" + camera.name() + "/PoseEstimate", poseEst.isPresent());
      if (poseEst.isPresent()) {
        hasSeenAprilTag = true;
        var pose = poseEst.get();
        SmartDashboard.putString("Vision/" + camera.name() + "/EstimatedPose",
            String.format("(%.2f, %.2f, %.1f°)",
                pose.estimatedPose.getX(),
                pose.estimatedPose.getY(),
                pose.estimatedPose.getRotation().toRotation2d().getDegrees()));
        if (Robot.isSimulation()) {
          // Reject vision estimates that are too far from current odometry to prevent seizuring
          // when the simulated robot is dragged to a new location
          double distanceToEstimate = swerveDrive.getPose()
              .getTranslation()
              .getDistance(pose.estimatedPose.toPose2d().getTranslation());
          if (distanceToEstimate >= 1.0) continue;
        }
        swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                                         pose.timestampSeconds,
                                         camera.curStdDevs);
      }
    }
  }

  /**
   * Generates the estimated robot pose.
   *
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
    Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
    if (Robot.isSimulation()) {
      Field2d debugField = visionSim.getDebugField();
      poseEst.ifPresentOrElse(
          est -> debugField.getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
          () -> debugField.getObject("VisionEstimation").setPoses());
    }
    return poseEst;
  }

  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance
   */
  public double getDistanceFromAprilTag(int id) {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
  }

  /**
   * Get tracked target from a camera of AprilTagID
   *
   * @param id     AprilTag ID
   * @param camera Camera to check.
   * @return Tracked target.
   */
  public PhotonTrackedTarget getTargetFromId(int id, Cameras camera) {
    PhotonTrackedTarget target = null;
    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget i : result.getTargets()) {
          if (i.getFiducialId() == id) {
            return i;
          }
        }
      }
    }
    return target;
  }

  /**
   * Returns whether the vision system has seen at least one AprilTag since startup.
   * Use this to gate turret targeting so it doesn't run on unconfirmed odometry.
   */
  public boolean hasSeenAprilTag() {
    return hasSeenAprilTag;
  }

  /**
   * Vision simulation.
   *
   * @return Vision Simulation
   */
  public VisionSystemSim getVisionSim() {
    return visionSim;
  }

  /**
   * Update the {@link Field2d} to include tracked targets.
   */
  public void updateVisionField() {
    List<PhotonTrackedTarget> targets = new ArrayList<>();
    for (Cameras c : Cameras.values()) {
      if (!c.resultsList.isEmpty()) {
        PhotonPipelineResult latest = c.resultsList.get(0);
        if (latest.hasTargets()) {
          targets.addAll(latest.targets);
        }
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets) {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    field2d.getObject("tracked targets").setPoses(poses);
  }

  /**
   * Camera Enum to select each camera.
   * Update these values to match your robot's camera configuration!
   */
  public enum Cameras {
    /**
     * Front Left Camera - update position/rotation to match your robot
     */
    FrontLeft("FrontLeft",
              new Rotation3d(0, Math.toRadians(0), Math.toRadians(30)),
              new Translation3d(Units.inchesToMeters(-2.485),
                                Units.inchesToMeters(13.7148),
                                Units.inchesToMeters(11.244565)),
              VecBuilder.fill(4, 4, 8), VecBuilder.fill(1.0, 1.0, 2.0)),
    /**
     * Front Right Camera - update position/rotation to match your robot
     */
    FrontRight("FrontRight",

               new Rotation3d(0, Math.toRadians(0), Math.toRadians(-30)),
               new Translation3d(Units.inchesToMeters(-2.485),
                                 Units.inchesToMeters(-13.7148),
                                 Units.inchesToMeters(11.244565)),
               VecBuilder.fill(4, 4, 8), VecBuilder.fill(1.0, 1.0, 2.0)),
    /**
     * Back Left Camera - update position/rotation to match your robot
     */
    BackLeft("BackLeft",
             new Rotation3d(0, Math.toRadians(0), Math.toRadians(150)),
             new Translation3d(Units.inchesToMeters(-4.51305),
                               Units.inchesToMeters(13.7148),
                               Units.inchesToMeters(11.244565)),
             VecBuilder.fill(4, 4, 8), VecBuilder.fill(1.0, 1.0, 2.0)),
    /**
     * Back Right Camera - update position/rotation to match your robot
     */
    BackRight("BackRight",
              new Rotation3d(0, Math.toRadians(0), Math.toRadians(-150)),
              new Translation3d(Units.inchesToMeters(-4.51305),
                                Units.inchesToMeters(-13.7148),
                                Units.inchesToMeters(11.244565)),
              VecBuilder.fill(4, 4, 8), VecBuilder.fill(1.0, 1.0, 2.0));

    /**
     * Latency alert to use when high latency is detected.
     */
    public final Alert latencyAlert;

    /**
     * Camera instance for comms.
     */
    public final PhotonCamera camera;

    /**
     * Pose estimator for camera.
     */
    public final PhotonPoseEstimator poseEstimator;

    /**
     * Standard Deviation for single tag readings for pose estimation.
     */
    private final Matrix<N3, N1> singleTagStdDevs;

    /**
     * Standard deviation for multi-tag readings for pose estimation.
     */
    private final Matrix<N3, N1> multiTagStdDevs;

    /**
     * Transform of the camera rotation and translation relative to the center of the robot
     */
    private final Transform3d robotToCamTransform;

    /**
     * Current standard deviations used.
     */
    public Matrix<N3, N1> curStdDevs;

    /**
     * Estimated robot pose.
     */
    public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

    /**
     * Simulated camera instance which only exists during simulations.
     */
    public PhotonCameraSim cameraSim;

    /**
     * Results list to be updated periodically and cached.
     */
    public List<PhotonPipelineResult> resultsList = new ArrayList<>();

    /**
     * Last read from the camera timestamp.
     */
    private double lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

    /**
     * Construct a Photon Camera class.
     *
     * @param name                  Name of the PhotonVision camera found in the PV UI.
     * @param robotToCamRotation    {@link Rotation3d} of the camera.
     * @param robotToCamTranslation {@link Translation3d} relative to the center of the robot.
     * @param singleTagStdDevs      Single AprilTag standard deviations.
     * @param multiTagStdDevsMatrix Multi AprilTag standard deviations.
     */
    Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
            Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix) {
      latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

      camera = new PhotonCamera(name);

      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

      poseEstimator = new PhotonPoseEstimator(Vision.fieldLayout, robotToCamTransform);

      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;

      if (Robot.isSimulation()) {
        SimCameraProperties cameraProp = new SimCameraProperties();
        // A 960 x 720 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate.
        cameraProp.setFPS(30);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
      }
    }

    /**
     * Add camera to {@link VisionSystemSim} for simulated photon vision.
     *
     * @param systemSim {@link VisionSystemSim} to use.
     */
    public void addToVisionSim(VisionSystemSim systemSim) {
      if (Robot.isSimulation()) {
        systemSim.addCamera(cameraSim, robotToCamTransform);
      }
    }

    /**
     * Get the result with the least ambiguity from the best tracked target within the Cache.
     *
     * @return The result in the cache with the least ambiguous best tracked target.
     */
    public Optional<PhotonPipelineResult> getBestResult() {
      if (resultsList.isEmpty()) {
        return Optional.empty();
      }

      PhotonPipelineResult bestResult = resultsList.get(0);
      double ambiguity = bestResult.getBestTarget().getPoseAmbiguity();
      double currentAmbiguity = 0;
      for (PhotonPipelineResult result : resultsList) {
        currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
        if (currentAmbiguity < ambiguity && currentAmbiguity > 0) {
          bestResult = result;
          ambiguity = currentAmbiguity;
        }
      }
      return Optional.of(bestResult);
    }

    /**
     * Get the latest result from the current cache.
     *
     * @return Empty optional if nothing is found. Latest result if something is there.
     */
    public Optional<PhotonPipelineResult> getLatestResult() {
      return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
    }

    /**
     * Get the estimated robot pose. Updates the current robot pose estimation.
     *
     * @return Estimated pose.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      updateUnreadResults();
      return estimatedRobotPose;
    }

    /**
     * Update the latest results, cached. Sorts the list by timestamp.
     */
    private void updateUnreadResults() {
      double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
      double currentTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
      double debounceTime = Milliseconds.of(15).in(Seconds);
      for (PhotonPipelineResult result : resultsList) {
        mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
      }

      resultsList = Robot.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
      lastReadTimestamp = currentTimestamp;
      resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
        return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
      });
      if (!resultsList.isEmpty()) {
        updateEstimatedGlobalPose();
      }
    }

    /**
     * The latest estimated robot pose on the field from vision data.
     */
    private void updateEstimatedGlobalPose() {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : resultsList) {
        for (var target : change.getTargets()) {
          int id = target.getFiducialId();
          boolean inLayout = poseEstimator.getFieldTags().getTagPose(id).isPresent();
          SmartDashboard.putNumber("Vision/" + name() + "/TagID", id);
          SmartDashboard.putBoolean("Vision/" + name() + "/TagInLayout", inLayout);
          SmartDashboard.putNumber("Vision/" + name() + "/Ambiguity", target.getPoseAmbiguity());
        }
        // Try multi-tag first, fall back to lowest ambiguity for single tag
        visionEst = poseEstimator.estimateCoprocMultiTagPose(change);
        if (visionEst.isEmpty()) {
          visionEst = poseEstimator.estimateLowestAmbiguityPose(change);
        }
        updateEstimationStdDevs(visionEst, change.getTargets());
      }
      estimatedRobotPose = visionEst;
    }

    /**
     * Calculates new standard deviations based on number of tags, strategy, and distance.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
      if (estimatedPose.isEmpty()) {
        curStdDevs = singleTagStdDevs;
      } else {
        var estStdDevs = singleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        for (var tgt : targets) {
          var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) {
            continue;
          }
          numTags++;
          avgDist += tagPose.get().toPose2d().getTranslation()
              .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
          curStdDevs = singleTagStdDevs;
        } else {
          avgDist /= numTags;
          if (numTags > 1) {
            estStdDevs = multiTagStdDevs;
          }
          if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          }
          curStdDevs = estStdDevs;
        }
      }
    }
  }
}

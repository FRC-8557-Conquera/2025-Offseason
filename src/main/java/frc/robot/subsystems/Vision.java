package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.awt.Desktop;
import java.lang.module.ResolutionException;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.logging.LogManager;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter_al;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
    
     // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    private double maxAmbiguity = 0.25;
    private double longDistancePoseEstCount = 0;
    private Supplier<Pose2d> currentPose;
    private Field2d field2d;

    public Vision(Supplier<Pose2d> currentPose, Field2d field) {
        // ----- Simulation
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

    public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
        Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
        if(aprilTagPose3d.isPresent()) {
            return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
        } else {
            throw new RuntimeException("april tag");
        }
    }

    public void updatePoseEstimation(SwerveDrive swerveDrive) {
        if(SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
            visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
        }
        for(Cameras c : Cameras.values()) {
            Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(c);
            if(poseEst.isPresent()) {
                var pose = poseEst.get();
                swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, c.curStdDevs);
            }
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras c) {
        Optional<EstimatedRobotPose> poseEst = c.getEstimatedGlobalPose();
        if(Robot.isSimulation()) {
            Field2d debugField = visionSim.getDebugField();
            poseEst.ifPresentOrElse(est -> debugField.getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()), () -> {debugField.getObject("VisionEstimation").setPoses();});
        }
        return poseEst;
    }
    public double getDistanceFromAprilTag(int id) {
        Optional<Pose3d> tag = fieldLayout.getTagPose(id);
        return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
    }

    public PhotonTrackedTarget getTargetFromId(int id, Cameras c) {
        PhotonTrackedTarget target = null;
        for(PhotonPipelineResult result : c.resultsList) {
            if(result.hasTargets()) {
                for(PhotonTrackedTarget i : result.getTargets()) {
                    if(i.getFiducialId() == id) return i;
                }
            }
        }
        return target;
    }
    public VisionSystemSim getVisionSim() {
        return visionSim;
    }

    public void updateVisionField() {
        List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
        for(Cameras c : Cameras.values()) {
            if(!c.resultsList.isEmpty()) {
                PhotonPipelineResult latest = c.resultsList.get(0);
                if(latest.hasTargets()) {
                    targets.addAll(latest.targets);
                }
            }
        }

        List<Pose2d> poses = new ArrayList<>();
        for(PhotonTrackedTarget target : targets) {
            if(fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
                Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
                poses.add(targetPose);
            }
        }

        field2d.getObject("tracked targets").setPoses(poses);
    }



    
    public enum Cameras {
        RAZER("razer", new Rotation3d(0,0,0.52), new Translation3d(0.335,0.325,0.31), VecBuilder.fill(4,4,8), VecBuilder.fill(0.5,0.5,1));
        public final Alert latencyAlert;
        public final PhotonCamera camera;
        public final PhotonPoseEstimator poseEstimator;
        private final Matrix<N3, N1> singleTagStdDevs;
        private final Matrix<N3, N1> multiTagStdDevs;
        private final Transform3d robotToCamTransform;
        public Matrix<N3, N1> curStdDevs;
        public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();
        public PhotonCameraSim cameraSim;
        public List<PhotonPipelineResult> resultsList = new ArrayList<>();
        private double lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
        Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation, Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevs) {
            this.latencyAlert = new Alert("Vision Latency Alert: " + name, AlertType.kWarning);
            this.camera = new PhotonCamera(name);
            this.robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);
            this.poseEstimator = new PhotonPoseEstimator(Vision.fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamTransform);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            this.singleTagStdDevs = singleTagStdDevs;
            this.multiTagStdDevs = multiTagStdDevs;

            if(Robot.isSimulation()) {
                SimCameraProperties cameraProp = new SimCameraProperties();
                cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(70));
                cameraProp.setCalibError(0.25, 0.08);
                cameraProp.setFPS(30);
                cameraProp.setAvgLatencyMs(35);
                cameraProp.setLatencyStdDevMs(5);

                cameraSim = new PhotonCameraSim(camera, cameraProp);
                cameraSim.enableDrawWireframe(true);
            }
        }
        
        public void addToVisionSim(VisionSystemSim visionSim) {
            if(Robot.isSimulation()) {
                visionSim.addCamera(cameraSim, robotToCamTransform);
            }
        }

        public Optional<PhotonPipelineResult> getBestResult() {
            if(resultsList.isEmpty()) {
                return Optional.empty();
            }

            PhotonPipelineResult bestResult = resultsList.get(0);
            double ambiguity = bestResult.getBestTarget().getPoseAmbiguity();
            double currentAmbiguity = 0;

            for(PhotonPipelineResult result : resultsList) {
                if(currentAmbiguity < ambiguity && currentAmbiguity > 0) {
                    bestResult = result;
                    ambiguity = currentAmbiguity;
                }
            }
            return Optional.of(bestResult);
        }
        public Optional<PhotonPipelineResult> getLatestResult() {
            return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
        }
        public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
            updateUnreadResults();
            return estimatedRobotPose;
        }
        private void updateUnreadResults() {
            double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
            double currentTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
            double debounceTime = Milliseconds.of(15).in(Seconds);
            for(PhotonPipelineResult result : resultsList) {
                mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
            }
            if((resultsList.isEmpty() || (currentTimestamp - mostRecentTimestamp >= debounceTime)) && (currentTimestamp - lastReadTimestamp) >= debounceTime) {
                resultsList = Robot.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
                lastReadTimestamp = currentTimestamp;
                resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
                    return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
                });
                if(!resultsList.isEmpty()) {
                    updateEstimatedGlobalPose();
                }
            }
        }
        private void updateEstimatedGlobalPose() {
            Optional<EstimatedRobotPose> visionEst = Optional.empty();
            for(var change : resultsList) {
                visionEst = poseEstimator.update(change);
                updateEstimationStdDevs(visionEst, change.getTargets());
            }
            estimatedRobotPose = visionEst;
        }
        private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
            if(estimatedPose.isEmpty()) {
                curStdDevs = singleTagStdDevs;
            } else {
                var estStdDevs = singleTagStdDevs;
                int numTags = 0;
                double avgDist = 0;

                for(var tgt : targets) {
                    var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                    if(tagPose.isEmpty()) {
                        continue;
                    }
                    numTags++;
                    avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
                }
                if(numTags == 0) {
                    curStdDevs = singleTagStdDevs;
                } else {
                    avgDist /= numTags;
                    if(numTags > 1) {
                        estStdDevs = multiTagStdDevs;
                    }
                    if(numTags == 1 && avgDist > 4) {
                        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                    } else {
                        estStdDevs = estStdDevs.times(1+(avgDist * avgDist / 30));
                    }
                    curStdDevs = estStdDevs;
                }
            }
        }
    }
    
}
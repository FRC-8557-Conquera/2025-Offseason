package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

public class AlignWithAprilTag extends Command {
    private final Swerve swerve;
    private final PhotonCamera camera;
    private final int targetAprilTagID; // Hedef AprilTag ID

    // Aşamalar için konfigürasyon parametreleri
    private static final double VISION_DES_ANGLE_deg = 0.0;
    private static final double VISION_TURN_kP = 0.008;
    private static final double VISION_STRAFE_kP = 0.01;
    private static final double ANGLE_TOLERANCE = 1.0;   // Açısal tolerans (derece)
    private static final double POSITION_TOLERANCE = 0.05; // Konum toleransı (metre)

    private boolean isAligned = false; // **Yeni:** Hizalamanın tamamlanıp tamamlanmadığını takip eder

    public AlignWithAprilTag(Swerve swerve, PhotonCamera camera, int targetAprilTagID) {
        this.swerve = swerve;
        this.camera = camera;
        this.targetAprilTagID = targetAprilTagID;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double strafe = 0.0;
        double turn = 0.0;

        boolean targetVisible = false;
        double targetYaw = 0.0;
        double targetXOffset = 0.0;

        // Tüm okunmamış sonuçları al
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            PhotonPipelineResult result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == targetAprilTagID) { 
                        targetYaw = target.getYaw() - 10.0;
                        targetXOffset = target.getBestCameraToTarget().getX(); // AprilTag'in yatay konumu
                        targetVisible = true;
                        break;
                    }
                }
            }
        }

        SmartDashboard.putBoolean("Target Visible", targetVisible);
        SmartDashboard.putNumber("Target Yaw (deg)", targetYaw);
        SmartDashboard.putNumber("Target X Offset (m)", targetXOffset);

        if (targetVisible) {
            if (Math.abs(targetYaw) > ANGLE_TOLERANCE) {
                turn = (VISION_DES_ANGLE_deg - targetYaw) * VISION_TURN_kP * Constants.Swerve.maxAngularVelocity;
            } else {
                turn = 0.0;
            }

            if (Math.abs(targetXOffset) > POSITION_TOLERANCE) {
                strafe = -targetXOffset * VISION_STRAFE_kP * Constants.Swerve.maxSpeed;
            } else {
                strafe = 0.0;
            }

            if (turn == 0.0 && strafe == 0.0) {
                isAligned = true;
            }
        }

        // Robotu hareket ettir
        swerve.getSwerveDrive().drive(new ChassisSpeeds(0, strafe, turn), false, new Translation2d());
    }

    @Override
    public boolean isFinished() {
        return isAligned; // **Hizalama tamamlandıysa komut çıkacak**
    }

    @Override
    public void end(boolean interrupted) {
        swerve.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0), false, new Translation2d());
    }
}

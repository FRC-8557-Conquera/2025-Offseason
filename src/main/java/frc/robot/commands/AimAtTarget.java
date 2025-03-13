package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;

public class AimAtTarget extends Command {
    private final Swerve swerve;
    private final PhotonCamera camera;

    // Aşamalar için konfigürasyon parametreleri
    // İstenilen açısal sapma (0° ise, etiketin tam ortasında olacak)
    private static final double VISION_DES_ANGLE_deg = 0.0;
    // Dönüş için orantısal kazanç (örneğin 0.01, gerekirse ayarlayın)
    private static final double VISION_TURN_kP = 0.01;
    // İstenilen hedef mesafe (örneğin 0.5 m)
    private static final double VISION_DES_RANGE_m = 0.5;
    // İleri/geri komut için orantısal kazanç (örneğin 0.01)
    private static final double VISION_STRAFE_kP = 0.01;

    public AimAtTarget(Swerve swerve, PhotonCamera camera) {
        this.swerve = swerve;
        this.camera = camera;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double forward = 0.0;
        double strafe = 0.0;
        double turn = 0.0;

        boolean targetVisible = false;
        double targetYaw = 0.0;
        double targetRange = 0.0;

        // Tüm okunmamış sonuçları al
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // En güncel sonucu kullanıyoruz (listede en son eklenen)
            PhotonPipelineResult result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 21) { // AprilTag ID 21, Reef’deki hedef
                        targetYaw = target.getYaw();
                        // Mesafe hesaplaması:
                        // Kamera yüksekliği: 0.31 m (örnek)
                        // Hedef yüksekliği: 8.75 inç ≈ 0.22225 m (etiket alt kenarı yüksekliği)
                        // Kamera pitch açısı: -30° (radyan cinsine çevriliyor)
                        // Ölçülen etiketin pitch açısı: target.getPitch(), derece → radyan
                        targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                                0.31,
                                0.22225,
                                Units.degreesToRadians(-30.0),
                                Units.degreesToRadians(target.getPitch())
                        );
                        targetVisible = true;
                        break;
                    }
                }
            }
        }

        // Debug bilgilerini SmartDashboard'e gönder
        SmartDashboard.putBoolean("Target Visible", targetVisible);
        SmartDashboard.putNumber("Target Yaw (deg)", targetYaw);
        SmartDashboard.putNumber("Target Range (m)", targetRange);

        // Eğer hedef görünürse, komutları hesapla
        if (targetVisible) {
            // Dönüş komutu: Hedefin açısal sapması 0°'ye (VISION_DES_ANGLE_deg) göre hesaplanır.
            turn = (VISION_DES_ANGLE_deg - targetYaw) * VISION_TURN_kP * Constants.Swerve.maxAngularVelocity;
            // İleri/geri komutu: Hedef mesafe sapması, istenen mesafe (VISION_DES_RANGE_m) ile ölçülen mesafe (targetRange) arasındaki fark
            forward = (VISION_DES_RANGE_m - targetRange) * VISION_STRAFE_kP * Constants.Swerve.maxSpeed;
        }

        // Hedef bulunamazsa, komut değişmez (robot sürücü kontrolüne devam edebilir veya sıfırlanabilir)
        ChassisSpeeds speeds = new ChassisSpeeds(forward, strafe, turn);
        swerve.getSwerveDrive().drive(speeds, false, new Translation2d());
    }

    @Override
    public boolean isFinished() {
        // Buton basılı olduğu sürece bu komut çalışır; komutu buton bırakıldığında sonlandırabilirsiniz.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Komut sona erdiğinde robotu durdur
        swerve.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0), false, new Translation2d());
    }
}

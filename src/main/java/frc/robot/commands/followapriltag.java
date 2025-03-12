package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Optional;

public class followapriltag extends Command {
    private final Swerve swerve;
    private final Vision vision;
    private final PIDController strafeController;

    private final int targetTagID;  // Takip edilecek AprilTag ID
    private final double targetX = 0.0;  // Görüntünün ortasında kalmasını istiyoruz

    public followapriltag(Swerve swerve, int targetTagID) {
        this.swerve = swerve;
        this.vision = swerve.vision;
        this.targetTagID = targetTagID;
        this.strafeController = new PIDController(0.1, 0, 0); // PID kontrolü (isteğe bağlı)

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        Optional<Pose2d> tagPose = vision.getEstimatedGlobalPose(Vision.Cameras.RAZER).map(pose -> pose.estimatedPose.toPose2d());

        if (tagPose.isPresent()) {
            double tagX = tagPose.get().getX();  // AprilTag'in X konumu

            // X pozisyon farkına göre sola veya sağa hareket ettir
            double strafeSpeed = strafeController.calculate(tagX, targetX);  // PID ile düzeltme

            // Strafe yönü belirleme (Swerve için)
            swerve.driveFieldOriented(new ChassisSpeeds(strafeSpeed, 0, 0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.driveFieldOriented(new ChassisSpeeds(0, 0, 0)); // Durdur
    }

    @Override
    public boolean isFinished() {
        return false;  
    }
}

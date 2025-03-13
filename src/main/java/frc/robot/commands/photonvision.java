package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.Cameras;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotContainer;
import java.util.Optional;

import org.photonvision.PhotonTargetSortMode;

public class photonvision extends Command {
    private final Joystick driver = new Joystick(0); 
    private final Swerve swerve;
    //private final PIDController strafeController;

    public photonvision(Swerve swerve, int targetTagID) {
        this.swerve = swerve;
        //this.strafeController = new PIDController(0.1, 0, 0); // PID kontrolü (isteğe bağlı)
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double forward = 0 * Constants.MAX_SPEED;
        double strafe = 0 * Constants.MAX_SPEED;
        double turn = 0 * Constants.Swerve.maxAngularVelocity;
        SmartDashboard.putString("vision","Sonuç yok");
        // Read in relevant data from the Camera
        boolean targetVisible = false;
        double targetYaw = 0.0;
        var results = Cameras.RAZER.getLatestResult();
        if (results.isPresent()) {
            // Camera processed a new frame since last
            // Get the result from the Optional.
            SmartDashboard.putString("vision","Sonuç var");
            var result = results.get();
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                SmartDashboard.putString("vision","AprilTag var");
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 21) {
                        SmartDashboard.putString("vision","Yedinci AprilTag var");
                        targetYaw = target.getYaw();
                        targetVisible = true;
                    }
                }
            }
        }

        // Auto-align when requested
        if (driver.getRawButtonPressed(5) && targetVisible) {
            // Driver wants auto-alignment to tag 7
            // And, tag 7 is in sight, so we can turn toward it.
            // Override the driver's turn command with an automatic one that turns toward the tag.
            turn = -1.0 * targetYaw * 0.1 * Constants.Swerve.maxAngularVelocity;
        }

        // Command drivetrain motors based on target speeds
        swerve.driveFieldOriented(new ChassisSpeeds(forward, strafe, turn));

        // Put debug information to the dashboard
        SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
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

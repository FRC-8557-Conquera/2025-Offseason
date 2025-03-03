package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Peripheral;
import edu.wpi.first.wpilibj.Timer;

public class shooter_saniye_tukur extends Command {
    private final Peripheral shooter;
    private final Timer timer = new Timer();
    private final double duration; // Çalışma süresi

    public shooter_saniye_tukur(Peripheral shooter, double duration) {
        this.shooter = shooter;
        this.duration = duration;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        shooter.shooterTukur();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.shooterDurdur();
        timer.stop();
    }
}

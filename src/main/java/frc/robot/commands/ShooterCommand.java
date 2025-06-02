package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ShooterCommand extends Command {
    private final IntakeSubsystem intake;
    private final Timer timer = new Timer();
    private final double duration;
    private final boolean dir;

    public ShooterCommand(IntakeSubsystem intake, double duration, boolean dir) {
        // 0 = içeri, 1 = dışarı
        this.intake = intake;
        this.duration = duration;
        this.dir = dir;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (!dir) {
            intake.setShooter(0.8);
        } else {
            intake.setShooter(-0.8);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setShooter(0);
        timer.stop();
    }
}

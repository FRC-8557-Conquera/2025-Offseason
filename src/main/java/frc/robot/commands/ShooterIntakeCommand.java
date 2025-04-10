package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Peripheral;

public class ShooterIntakeCommand extends Command {
    private final Peripheral peripheral;
    private final Timer timer = new Timer();
    private final double duration;
    private final boolean dir;

    public ShooterIntakeCommand(Peripheral shooter, double duration, boolean dir) {
        // 0 = içeri, 1 = dışarı
        this.peripheral = shooter;
        this.duration = duration;
        this.dir = dir;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (dir) {
            peripheral.shooterIcineal();
        } else {
            peripheral.shooterTukur();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        peripheral.shooterDurdur();
        timer.stop();
    }
}

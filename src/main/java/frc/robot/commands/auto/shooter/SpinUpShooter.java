package frc.robot.commands.auto.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.ShooterSubsystem;

public class SpinUpShooter extends Command{
    private final ShooterSubsystem shooter;

    public SpinUpShooter(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.spinUpShooter(); // Set desired speed
    }

    @Override
    public boolean isFinished() {
        return false; // Return true when the shooter reaches the desired speed
    }

    @Override
    public void end(boolean interrupted) {
    }
}
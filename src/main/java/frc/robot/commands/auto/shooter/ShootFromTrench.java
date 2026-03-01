package frc.robot.commands.auto.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterRPMConstants;
import frc.robot.subsystems.topdeck.AdvancerSubsystem;
import frc.robot.subsystems.topdeck.BeamBreak;
import frc.robot.subsystems.topdeck.ShooterSubsystem;

public class ShootFromTrench extends Command{
    private final ShooterSubsystem shooter;
    private final AdvancerSubsystem advancer;
    private final BeamBreak beamBreak;
    private final Timer timer = new Timer();

    public ShootFromTrench(ShooterSubsystem shooterSubsystem, AdvancerSubsystem advancerSubsystem, BeamBreak beamBreakSubsystem) {
        this.shooter = shooterSubsystem;
        this.advancer = advancerSubsystem;
        this.beamBreak = beamBreakSubsystem;
        addRequirements(shooterSubsystem);
        addRequirements(advancerSubsystem);
        addRequirements(beamBreakSubsystem);
    }

    @Override
    public void initialize() {
        shooter.runShooterAtVelocity(ShooterRPMConstants.INSIDE_TRENCH);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (shooter.runShooterAtVelocity(ShooterRPMConstants.INSIDE_TRENCH)) {
            advancer.advance();
        }else {
            advancer.stopAdvancer();
        }
    }

    @Override
    public boolean isFinished() {
        return beamBreak.getHopper() && timer.hasElapsed(4.0); // Finish when the beam break detects a ball in the hopper and 1 second has passed
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
        advancer.stopAdvancer();
    }
}

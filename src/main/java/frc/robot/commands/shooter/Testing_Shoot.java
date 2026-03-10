package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.AdvancerSubsystem;
import frc.robot.subsystems.topdeck.BeamBreak;
import frc.robot.subsystems.topdeck.ShooterSubsystem;

public class Testing_Shoot extends Command {
    private final ShooterSubsystem shooter;
    private final BeamBreak beamBreak;

    public Testing_Shoot(ShooterSubsystem shooterSubsystem,  BeamBreak beamBreak) {
        this.shooter = shooterSubsystem;
        // this.advancer = advancerSubsystem;
        this.beamBreak = beamBreak;
        addRequirements(shooterSubsystem,  beamBreak);
    }

    private boolean firstRun = true;


    @Override
    public void initialize() {
        firstRun = true;
    }

    @Override
    public void execute() {
        shooter.runShooterAtVelocity(1);
        // if (shooter.getVelocity() > 5000 && firstRun) {
        //     advancer.advance();
        //     firstRun = false;
        // } else {
        //     advancer.stopAdvancer();
        // }

        // if(!firstRun){
        //     advancer.advance();
        // }
        
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
        // advancer.stopAdvancer();
    }

    @Override
    public boolean isFinished() {
        return /*beamBreak.getShooter() ==*/ false;
    }
}

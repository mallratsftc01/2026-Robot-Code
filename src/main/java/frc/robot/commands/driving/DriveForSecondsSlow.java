package frc.robot.commands.driving;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class DriveForSecondsSlow extends Command {
    DrivetrainIO d;
    boolean isFinished;
    Timer timer;
    double seconds;

    public DriveForSecondsSlow(DrivetrainIO d, double seconds) {
        this.d = d;
        this.seconds = seconds;
        addRequirements(d);
    }

    @Override
    public void initialize() {
        timer = new Timer();
        isFinished = false;
        timer.start();
    }

    @Override
    public void execute() {
        d.drive(0.1, 0.0, 0.0, false);
        if (timer.advanceIfElapsed(seconds)) {
            d.drive(0, 0, 0, false);
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}

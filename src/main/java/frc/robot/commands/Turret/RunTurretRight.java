package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RunTurretRight extends Command {
    //TODO: nothing saying "turn left or turn right", just says go at a speed of 1

    public RunTurretRight() {
        addRequirements(RobotContainer.turret);
    }

	// Called just before this Command runs the first time
	public void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
        RobotContainer.turret.setSpeed(1.0);
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		RobotContainer.turret.setSpeed(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}



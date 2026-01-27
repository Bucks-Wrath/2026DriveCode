package frc.robot.commands.IntakePivot;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class SetIntakePivot extends Command {
	private double intakePivotPosition = 0;

	public SetIntakePivot(double intakePivotPosition) {
		this.intakePivotPosition = intakePivotPosition;

		addRequirements(RobotContainer.intakePivot);
	}

	// Called just before this Command runs the first time
	public void initialize() {
		RobotContainer.intakePivot.setTargetPosition(intakePivotPosition);
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		RobotContainer.intakePivot.motionMagicControl();
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
			return RobotContainer.intakePivot.isInPosition(intakePivotPosition);
	}

	// Called once after isFinished returns true
	protected void end() {

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {

	}
}
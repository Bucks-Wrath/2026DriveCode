package frc.robot.commands.IntakePivot;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class ToggleIntakePivotPosition extends Command {
	private double intakePivotPosition = 0;
	private double intakePivotTargetPosition = 0;
	private double intakePivotHomePosition = 0;
	private double intakePivotExtendedPosition = -10;

	public ToggleIntakePivotPosition() {
		addRequirements(RobotContainer.intakePivot);
	}

	// Called just before this Command runs the first time
	public void initialize() {
		RobotContainer.intakePivot.setTargetPosition(0);
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		// if the intake is in, extend it
		if(this.intakePivotPosition == 0) {
			RobotContainer.intakePivot.setTargetPosition(this.intakePivotExtendedPosition);
			this.intakePivotTargetPosition = this.intakePivotExtendedPosition;
		}
		else {
			RobotContainer.intakePivot.setTargetPosition(this.intakePivotHomePosition);
			this.intakePivotTargetPosition = this.intakePivotHomePosition;
		}

		RobotContainer.intakePivot.motionMagicControl();
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
			return RobotContainer.intakePivot.isInPosition(this.intakePivotTargetPosition);
	}

	// Called once after isFinished returns true
	protected void end() {
		this.intakePivotPosition = this.intakePivotTargetPosition;
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {

	}
}

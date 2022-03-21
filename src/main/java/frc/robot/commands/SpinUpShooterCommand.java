package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SpinUpShooterCommand extends CommandBase {

  private BangBangController bangbang;
  private SimpleMotorFeedforward feedforward;
  double targetVelocity;
  ShooterSubsystem shooter;

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public SpinUpShooterCommand(double targetVelocity, ShooterSubsystem shooter) {
    bangbang = new BangBangController();
    feedforward = new SimpleMotorFeedforward(Constants.ShooterConstants.kS, Constants.ShooterConstants.kV, Constants.ShooterConstants.kA);
    this.targetVelocity = targetVelocity;
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    //TODO SET SHOOTER MOTORS TO COAST MODE, BANGBANG CONTROLLER WILL DAMAGE MOTORS OTHERWISE
    //Setting voltage because feedforward units are in volts
    //shooter.setMotorsVoltage(MathUtil.clamp( (bangbang.calculate(shooterMetersPerSec, targetVelocity) * 12 ) + 0.9 * feedforward.calculate(targetVelocity), 0, 12));
    double shooterMetersPerSec = ( ( shooter.getAverageShooterVelocity() / 2048 ) * 0.3048 ) * 10;
    shooter.setMotors(bangbang.calculate(shooterMetersPerSec, targetVelocity));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setMotors(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
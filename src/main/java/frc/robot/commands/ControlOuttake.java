package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OuttakeSubsystem;


public class ControlOuttake extends Command {
    private final DoubleSupplier forward;
    private final DoubleSupplier reverse;
    private final OuttakeSubsystem outtake;

    public ControlOuttake(DoubleSupplier forward, DoubleSupplier reverse, OuttakeSubsystem outtake) {
        this.forward = forward;
        this.reverse = reverse;
        this.outtake = outtake;
        addRequirements(this.outtake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
      // Run the roller motor at the desired speed
      outtake.runMotor(forward.getAsDouble(), reverse.getAsDouble());
    }

    @Override
    public void end(boolean isInterrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

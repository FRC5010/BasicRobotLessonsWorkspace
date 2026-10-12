package first.robot.commands;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;

import first.robot.subsystems.Drivetrain;

public final class Autos {
    private Autos() {}

    public static Command driveTurnDriveCoroutine(Drivetrain drivetrain, double distance) {
        return Command.noRequirements(coroutine -> {
            coroutine.await(drivetrain.driveDistance(distance));
            coroutine.wait(Seconds.of(1.0));
            coroutine.await(drivetrain.turnToHeading(90));
            coroutine.wait(Seconds.of(1.0));
            coroutine.await(drivetrain.driveDistance(distance));
        }
        ).named("Drive Turn Drive");
    }
    public static Command boxPattern(Drivetrain drivetrain) {
        return Command.noRequirements(coroutine -> {
            for (int i = 0; i < 2; i++) {
                coroutine.await(driveTurnDriveCoroutine(drivetrain, 1.0));
            }
        }).named("Box Pattern");
    }
    public static Command turnDegrees(Drivetrain drivetrain, double degrees) {
        return Command.noRequirements(coroutine -> {
            coroutine.await(drivetrain.turnToHeading(degrees));
        }).named("Turn Degrees");
    }
}

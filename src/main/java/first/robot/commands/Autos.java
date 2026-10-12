package first.robot.commands;

public final class Autos {
    private Autos() {} //utility class, never instantiated

    public static Command driveTurnDrive(Drivetrain drivetrain) {
        return Command.sequence(
            drivetrain.driveDistance(1.0),
            drivetrain.turnToHeading(90),
            drivetrain.driveDistance(1.0))
            .named("Drive Turn Drive");

    }

    //different way of writing the same command above, using a coroutine instead of a sequence. The coroutine version is more flexible and can be easier to read.
    public static Command driveTurnDriveWithCoroutine(Drivetrain drivetrain) {
        return Command.noRequirements(coroutine -> {
            coroutine.await(drivetrain.driveDistance(1.0));
            coroutine.wait(Seconds.of(1.0));
            coroutine.await(drivetrain.turnToHeading(90));
            coroutine.await(drivetrain.driveDistance(1.0));
        })
        .named("Drive Turn Drive");
    }


}

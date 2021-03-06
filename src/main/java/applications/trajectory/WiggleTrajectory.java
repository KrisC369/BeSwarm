package applications.trajectory;

import applications.trajectory.geom.point.Point4D;
import choreo.Choreography;
import control.FiniteTrajectory4d;

/**
 * Trajectory that performs a wiggle effect in place. Note that this trajectory does not keep to the
 * usual velocity restraints.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
public class WiggleTrajectory extends BasicTrajectory implements FiniteTrajectory4d {
  private static final double WIGGLE_DISTANCE = 0.5d;
  private static final double TIME_TO_REST_AT_ORIGIN = 0.001d;
  private final FiniteTrajectory4d target;

  WiggleTrajectory(Point4D centerPoint, int wiggles, double timeToStayAtEdges) {
    Choreography.Builder builder = Choreography.builder();
    double orientation = centerPoint.getAngle();
    Point4D endRight =
        Point4D.create(
            centerPoint.getX() + StrictMath.cos(orientation) * WIGGLE_DISTANCE,
            centerPoint.getY() + StrictMath.sin(orientation) * WIGGLE_DISTANCE,
            centerPoint.getZ(),
            orientation);
    Point4D endLeft =
        Point4D.create(
            centerPoint.getX() - StrictMath.cos(orientation) * WIGGLE_DISTANCE,
            centerPoint.getY() - StrictMath.sin(orientation) * WIGGLE_DISTANCE,
            centerPoint.getZ(),
            orientation);

    for (int i = 0; i < wiggles; i++) {
      builder
          .withTrajectory(Trajectories.newHoldPositionTrajectory(endRight))
          .forTime(timeToStayAtEdges);
      builder
          .withTrajectory(Trajectories.newHoldPositionTrajectory(endLeft))
          .forTime(2 * timeToStayAtEdges);
      builder
          .withTrajectory(Trajectories.newHoldPositionTrajectory(centerPoint))
          .forTime(TIME_TO_REST_AT_ORIGIN);
    }
    target = builder.build();
  }

  @Override
  public double getTrajectoryDuration() {
    return target.getTrajectoryDuration();
  }

  @Override
  public double getDesiredPositionX(double timeInSeconds) {
    return target.getDesiredPositionX(timeInSeconds);
  }

  @Override
  public double getDesiredPositionY(double timeInSeconds) {
    return target.getDesiredPositionY(timeInSeconds);
  }

  @Override
  public double getDesiredPositionZ(double timeInSeconds) {
    return target.getDesiredPositionZ(timeInSeconds);
  }

  @Override
  public double getDesiredAngleZ(double timeInSeconds) {
    return target.getDesiredAngleZ(timeInSeconds);
  }
}

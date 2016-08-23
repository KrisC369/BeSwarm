package applications.trajectory;

import applications.trajectory.geom.point.Point3D;
import applications.trajectory.geom.point.Point4D;
import com.google.auto.value.AutoValue;
import control.FiniteTrajectory4d;
import control.Trajectory2d;
import utils.math.RotationOrder;
import utils.math.Transformations;

import static com.google.common.base.Preconditions.checkArgument;
import static com.google.common.base.Preconditions.checkNotNull;

/**
 * Corkscrew motion around a straight line trajectory defined by an origin and destination point, a
 * radius as perpendicular distance to the straight line (origin-destination) and a frequency to
 * specify the number of revolutions.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
public final class CorkscrewTrajectory4D extends PeriodicTrajectory implements FiniteTrajectory4d {

  private static final double EPSILON = 0.00000001d;
  private static final String VELOCITY_ERROR_MESSAGE =
      "velocity component is higher than 1 for the given origin-destination points, "
          + "velocity, radius and frequency values.";
  private final UnitTrajectory unitTrajectory;
  private final double aroundX;
  private final double aroundY;
  private final Point4D origin;
  private final Point3D destination;

  private Point4DCache cache;

  private CorkscrewTrajectory4D(
      Point4D origin,
      Point3D destination,
      double speed,
      double radius,
      double frequency,
      double phase) {
    super(phase, Point4D.origin(), radius, frequency);
    this.origin = origin;
    this.destination = destination;
    checkArgument(!origin.equals(destination), "Origin should not be the same as destination");
    Point4D destinationProjection = Point4D.from(destination, 0);
    double distance = Point4D.distance(origin, destinationProjection);
    unitTrajectory =
        new UnitTrajectory(
            CircleTrajectory2D.builder()
                .setRadius(radius)
                .setFrequency(frequency)
                .setPhase(phase)
                .build(),
            speed,
            distance);

    //translate origin to get angles to unit vectors.
    Point4D translated = destinationProjection.minus(origin);

    //find angles to unit trajectory and check components for excessive speeds.
    Point3D speedComponent = Point3D.scale(Point3D.project(translated), speed / distance);

    checkArgument(
        isValidVelocity(speedComponent.getX(), speed, radius, frequency),
        "X " + VELOCITY_ERROR_MESSAGE);
    checkArgument(
        isValidVelocity(speedComponent.getY(), speed, radius, frequency),
        "Y " + VELOCITY_ERROR_MESSAGE);
    checkArgument(
        isValidVelocity(speedComponent.getZ(), speed, radius, frequency),
        "Z " + VELOCITY_ERROR_MESSAGE);

    double zyNorm = Math.sqrt(Math.pow(translated.getY(), 2) + Math.pow(translated.getZ(), 2));
    this.aroundX = (Math.PI / 2) - Math.acos(translated.getY() / zyNorm);
    this.aroundY =
        Math.acos(
            translated.getX() / Math.sqrt(Math.pow(translated.getX(), 2) + Math.pow(zyNorm, 2)));

    //set initial cache
    this.cache = newCache(Point4D.origin(), -1);
  }

  /**
   * Calculates whether for a given velocity vector component, magnitude and perpendicular velocity
   * magnitude from the circle movement parameters, the velocity component is still within
   * acceptable bounds. The velocity component for x, Vx = speedX + circleX where speedX is the x
   * component of the vector with magnitude |v| (specified by speed) and direction specified by
   * destination-origin and where circleX represents the x component of the velocity vector of the
   * circle movement at the moment where this vector attains its highest magnitude in the x
   * component. The circle movement velocity vector is always perpendicular to the speed vector
   * (destination-origin with |v|) and the magnitude of the circle velocity vector depends on the
   * circle movement parameters (radius, frequency).
   *
   * @param speedcomp the component of the velocity. (eg. Vx as speed*cos(phi) with phi the angle of
   *     the vector with regards to the component unit vector.)
   * @param speed the speed or magnitude of the velocity vector |v|.
   * @param radius the radius of the circle that attains greatest magnitude perpendicular to the
   *     current component.
   * @param frequencythe frequency of the circle that attains greatest magnitude perpendicular to
   *     the current component.
   * @return false if for the given arguments, the component velocity is >1.
   */
  private static boolean isValidVelocity(
      double speedcomp, double speed, double radius, double frequency) {
    double rfreq2pi = radius * frequency * TWOPI;
    double vcomp = speedcomp + rfreq2pi * Math.sqrt(1 - Math.pow(speedcomp / speed, 2));
    if (vcomp > MAX_ABSOLUTE_VELOCITY) {
      return false;
    }
    return true;
  }

  static Builder builder() {
    return new Builder();
  }

  static Point4DCache newCache(Point4D point, double timeMark) {
    return new AutoValue_CorkscrewTrajectory4D_Point4DCache(point, timeMark);
  }

  private static boolean isEqual(double a, double b) {
    return Math.abs(a - b) < EPSILON;
  }

  private static Point4D rotationTransform(Point4D toTrans, double aroundX, double aroundY) {
    return Point4D.from(
        Transformations.reverseRotation(
            Point3D.project(toTrans), aroundX, aroundY, 0, RotationOrder.XYZ),
        0);
  }

  private void refreshCache(double time) {
    if (!isEqual(cache.getTimeMark(), time)) {
      Point4D beforeTransPoint =
          Point4D.create(
              unitTrajectory.getDesiredPositionX(time),
              unitTrajectory.getDesiredPositionY(time),
              unitTrajectory.getDesiredPositionZ(time),
              unitTrajectory.getDesiredAngleZ(time));
      setCache(beforeTransPoint, time);
    }
  }

  private void setCache(Point4D beforeTransPoint, double time) {
    this.cache = newCache(beforeTransPoint, time);
  }

  private Point4D getCachePoint() {
    return this.cache.getDestinationPoint();
  }

  private Point4D translationTransform(Point4D toTrans) {
    return rotationTransform(toTrans, aroundX, aroundY).plus(origin);
  }

  @Override
  public double getTrajectoryDuration() {
    return unitTrajectory.getTrajectoryDuration();
  }

  @Override
  public double getDesiredPositionX(double timeInSeconds) {
    final double currentTime = getRelativeTime(timeInSeconds);
    refreshCache(currentTime);
    return translationTransform(getCachePoint()).getX();
  }

  @Override
  public double getDesiredPositionY(double timeInSeconds) {
    final double currentTime = getRelativeTime(timeInSeconds);
    refreshCache(currentTime);
    return translationTransform(getCachePoint()).getY();
  }

  @Override
  public double getDesiredPositionZ(double timeInSeconds) {
    final double currentTime = getRelativeTime(timeInSeconds);
    refreshCache(currentTime);
    return translationTransform(getCachePoint()).getZ();
  }

  @Override
  public double getDesiredAngleZ(double timeInSeconds) {
    final double currentTime = getRelativeTime(timeInSeconds);
    refreshCache(currentTime);
    return translationTransform(getCachePoint()).getAngle();
  }

  private Point4D getOrigin() {
    return origin;
  }

  private Point3D getDestination() {
    return destination;
  }

  @Override
  public String toString() {
    return "CorkscrewTrajectory4D{"
        + "velocity="
        + unitTrajectory.getSpeed()
        + ", origin point="
        + getOrigin()
        + ", destination point="
        + getDestination()
        + ", radius="
        + unitTrajectory.getRadius()
        + ", frequency="
            + unitTrajectory.getFrequency()
        + '}';
  }

  @AutoValue
  abstract static class Point4DCache {

    public abstract Point4D getDestinationPoint();

    public abstract double getTimeMark();
  }

  /** Builder for corkscrew trajectories. */
  public static final class Builder {
    private Point4D origin = Point4D.origin();
    private Point3D destination = Point3D.origin();
    private double speed = 1;
    private double radius = 0.5;
    private double frequency = 0.3;
    private double phase = 0;

    private Builder() {
      origin = Point4D.origin();
      destination = Point3D.origin();
      speed = 1;
      radius = 0.5;
      frequency = 0.3;
    }

    /**
     * Default value is Point4D.origin()
     *
     * @param location The origin of the trajectory.
     * @return this builder
     */
    public Builder setOrigin(Point4D origin) {
      this.origin = origin;
      return this;
    }

    /**
     * Default value is Point3D.origin()
     *
     * @param location The destination of the trajectory.
     * @return this builder
     */
    public Builder setDestination(Point3D destination) {
      this.destination = destination;
      return this;
    }

    /**
     * Default value = 1.
     *
     * @param speed The linear velocity between origin and destination.
     * @return this builder
     */
    public Builder setSpeed(double speed) {
      this.speed = speed;
      return this;
    }

    /**
     * Default radius = 0.5.
     *
     * @param radius The radius of the circle movement.
     * @return this builder
     */
    public Builder setRadius(double radius) {
      this.radius = radius;
      return this;
    }
    /**
     * Default frequency = 0.3.
     *
     * @param frequency The frequency of the circle movement.
     * @return this builder
     */
    public Builder setFrequency(double frequency) {
      this.frequency = frequency;
      return this;
    }

    /**
     * Default value = 0.
     *
     * @param phase the phase displacement of the movement.
     * @return this builder
     */
    public Builder setPhase(double phase) {
      this.phase = phase;
      return this;
    }

    /** @return a new Corkscrew trajectory instance. */
    public CorkscrewTrajectory4D build() {
      checkNotNull(this.origin, "You have to Supply an origin with setOrigin()");
      checkNotNull(this.destination, "You have to Supply a destination with setOrigin()");
      return new CorkscrewTrajectory4D(origin, destination, speed, radius, frequency, phase);
    }
  }

  private final class UnitTrajectory implements FiniteTrajectory4d {
    private final LinearTrajectory1D linear;
    private final double endPoint;
    private final double speed;
    private final double frequency;
    private final double radius;
    private Trajectory2d circlePlane;
    private boolean atEnd;

    private UnitTrajectory(CircleTrajectory2D circlePlane, double speed, double endPoint) {
      this.linear = new LinearTrajectory1D(0, speed);
      this.circlePlane = circlePlane;
      this.endPoint = endPoint;
      this.atEnd = false;
      this.speed = speed;
      this.frequency = circlePlane.getFrequency();
      this.radius = circlePlane.getRadius();
    }

    @Override
    public double getDesiredPositionX(double timeInSeconds) {
      if (atEnd) {
        return endPoint;
      }
      if (linear.getDesiredPosition(timeInSeconds) > endPoint) {
        markEnd();
        return endPoint;
      }
      return linear.getDesiredPosition(timeInSeconds);
    }

    private void markEnd() {
      this.atEnd = true;
      this.circlePlane = new NoMovement2DTrajectory();
    }

    double getSpeed() {
      return speed;
    }

    private double getRadius() {
      return radius;
    }

    double getFrequency() {
      return frequency;
    }

    @Override
    public double getDesiredPositionY(double timeInSeconds) {
      return circlePlane.getDesiredPositionOrdinate(timeInSeconds);
    }

    @Override
    public double getDesiredPositionZ(double timeInSeconds) {
      return circlePlane.getDesiredPositionAbscissa(timeInSeconds);
    }

    @Override
    public double getDesiredAngleZ(double timeInSeconds) {
      return 0;
    }

    @Override
    public double getTrajectoryDuration() {
      return endPoint / speed;
    }

    private class NoMovement2DTrajectory implements Trajectory2d {

      @Override
      public double getDesiredPositionAbscissa(double timeInSeconds) {
        return 0;
      }

      @Override
      public double getDesiredPositionOrdinate(double timeInSeconds) {
        return 0;
      }
    }
  }
}

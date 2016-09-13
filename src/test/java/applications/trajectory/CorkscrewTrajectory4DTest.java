package applications.trajectory;

import applications.trajectory.geom.point.Point3D;
import applications.trajectory.geom.point.Point4D;
import com.google.common.collect.Lists;
import control.FiniteTrajectory4d;
import org.junit.Before;
import org.junit.Test;

import java.util.List;

import static applications.trajectory.CorkscrewTrajectory4D.newCache;
import static applications.trajectory.TestUtils.EPSILON;
import static applications.trajectory.TestUtils.assertBounds;
import static applications.trajectory.TestUtils.getAngularVelocity;
import static applications.trajectory.TestUtils.getVelocityX;
import static applications.trajectory.TestUtils.getVelocityY;
import static applications.trajectory.TestUtils.getVelocityZ;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

/** @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be> */
public class CorkscrewTrajectory4DTest {
  private final double speed = 1;
  private final double radius = 0.5;
  private final double frequency = 0.3;
  private final double startOrientation = Math.PI;
  private final int phase = 0;
  private final double zDistance = 10;
  private final double startDistance = 10;
  private final double x = 0;
  private final double y = 0;
  private FiniteTrajectory4d trajectory;

  @Before
  public void setUp() throws Exception {
    this.trajectory =
        CorkscrewTrajectory4D.builder()
            .setOrigin(Point4D.create(x, y, startDistance, startOrientation))
            .setDestination(Point3D.create(x, y, startDistance + zDistance))
            .setSpeed(speed)
            .setRadius(radius)
            .setFrequency(frequency)
            .setPhase(phase)
            .build();
    initialize();
  }

  private void initialize() {
    trajectory.getDesiredPositionX(0);
    trajectory.getDesiredPositionY(0);
    trajectory.getDesiredPositionZ(0);
    trajectory.getDesiredAngleZ(0);
  }

  @Test
  public void testTrajectoryBoundsSimple() {
    verifyBounds(
        trajectory,
        1000,
        x - radius,
        x + radius,
        y - radius,
        y + radius,
        startDistance,
        startDistance + zDistance,
        startOrientation - EPSILON,
        startOrientation + EPSILON);
  }

  private void verifyBounds(
      FiniteTrajectory4d trajectory,
      double end,
      double minBoundX,
      double maxBoundX,
      double minBoundY,
      double maxBoundY,
      double minBoundZ,
      double maxBoundZ,
      double minAngle,
      double maxAngle) {
    List<Double> lx = Lists.newArrayList();
    List<Double> ly = Lists.newArrayList();
    List<Double> lz = Lists.newArrayList();
    List<Double> la = Lists.newArrayList();

    for (int i = 0; i < end; i++) {
      lx.add(trajectory.getDesiredPositionX(i / 10d));
      ly.add(trajectory.getDesiredPositionY(i / 10d));
      lz.add(trajectory.getDesiredPositionZ(i / 10d));
      la.add(trajectory.getDesiredAngleZ(i / 10d));
    }
    assertBounds(lx, minBoundX, maxBoundX);
    assertBounds(ly, minBoundY, maxBoundY);
    assertBounds(lz, minBoundZ, maxBoundZ);
    assertBounds(la, minAngle, maxAngle);
  }

  @Test
  public void testTrajectoryZ() {
    double time = 5;
    assertEquals(startDistance + time, trajectory.getDesiredPositionZ(time), EPSILON);
  }

  @Test
  public void testTrajectoryBoundsZVelocity() {
    List<Double> l = Lists.newArrayList();
    for (int i = 0; i < trajectory.getTrajectoryDuration() * 10; i++) {
      //            assertEquals(0.01 * i, trajectory.getDesiredPositionZ(i / 10d),
      // TestUtils.EPSILON);
      l.add(getVelocityZ(trajectory, i / 10d));
    }
    assertBounds(l, speed, speed);
  }

  @Test
  public void testAutoValueCache() {
    CorkscrewTrajectory4D.Point4DCache c = newCache(Point4D.create(2, 2, 2, 2), 1);
    assertEquals(2, c.getDestinationPoint().getX(), 0);
  }

  @Test
  public void testDefault() {
    trajectory = CorkscrewTrajectory4D.builder().build();
    assertEquals(0, trajectory.getTrajectoryDuration(), 0);
  }

  @Test
  public void testAngularMovement() {
    List<Double> l = Lists.newArrayList();
    for (int i = 0; i < trajectory.getTrajectoryDuration() * 10; i++) {
      //            assertEquals(0.01 * i, trajectory.getDesiredPositionZ(i / 10d),
      // TestUtils.EPSILON);
      l.add(trajectory.getDesiredAngleZ(i / 10d));
    }
    assertBounds(l, Math.PI, Math.PI);
  }

  @Test
  public void testTrajectoryVelocityBoundsSimple() {
    verifyVelocity(this.trajectory);
  }

  private void verifyVelocity(FiniteTrajectory4d trajectory) {
    List<Double> lx = Lists.newArrayList();
    List<Double> ly = Lists.newArrayList();
    List<Double> lz = Lists.newArrayList();
    for (int i = 0; i < 1000; i++) {
      lx.add(getVelocityX(trajectory, i / 10d));
      ly.add(getVelocityY(trajectory, i / 10d));
      lz.add(getVelocityZ(trajectory, i / 10d));
    }
    assertBounds(lx, -1, 1);
    assertBounds(ly, -1, 1);
    assertBounds(lz, -1, 1);
  }

  @Test(expected = AssertionError.class)
  public void testTrajectoryVelocityBoundsComplexTrajectory1() {
    this.trajectory =
        CorkscrewTrajectory4D.builder()
            .setOrigin(Point4D.create(x, y, startDistance, Math.PI))
            .setDestination(Point3D.create(x, y + 15, startDistance + zDistance))
            .setSpeed(speed)
            .setRadius(radius)
            .setFrequency(frequency)
            .setPhase(phase)
            .build();
    verifyVelocity(this.trajectory);
  }

  @Test(expected = AssertionError.class)
  public void testTrajectoryVelocityBoundsComplexTrajectory2() {
    this.trajectory =
        CorkscrewTrajectory4D.builder()
            .setOrigin(Point4D.create(0, 0, 0, Math.PI))
            .setDestination(Point3D.create(15, 15, 15))
            .setSpeed(speed)
            .setRadius(radius)
            .setFrequency(frequency)
            .setPhase(phase)
            .build();
    verifyVelocity(this.trajectory);
  }

  @Test
  public void testCorrectTrajectoryInit() {
    Point4D start = Point4D.create(0, 0, 10, 0);
    Point3D end = Point3D.create(0, 15, 25);
    trajectory =
        Trajectories.corkscrewTrajectoryBuilder()
            .setOrigin(start)
            .setDestination(end)
            .setRadius(0.5)
            .setFrequency(0.25)
            .setSpeed(0.6)
            .build();
    assertNotEquals(0, trajectory.getTrajectoryDuration());
  }

  @Test
  public void testTrajectoryVelocityBoundsAngle() {
    List<Double> l = Lists.newArrayList();
    for (int i = 0; i < 1000; i++) {
      l.add(getAngularVelocity(trajectory, i / 10d));
    }
    assertBounds(l, 0, 0);
  }

  /**
   * These tests are coded to represent the desired direction of movement in specific dimensions.
   * P(ositive)X or N(egative)X P(ositive)Y or N(egative)Y P(ositive)Z or N(egative)Z
   */
  @Test
  public void testBoundsXYZComplexPXNYPZ() {
    double orientation = -Math.PI / 2;
    Point4D start = Point4D.create(0, 0, 1, orientation);
    Point3D end = Point3D.create(1.5, -3.0, 1.5);
    double radius = 0.5;
    double frequency = 0.1;
    double velocity = 0.1;
    this.trajectory =
        Trajectories.newCorkscrewTrajectory(start, end, velocity, radius, frequency, 0);
    verifyBounds(trajectory, 1000, radius, start, Point4D.from(end, orientation));
  }

  private void verifyBounds(
      FiniteTrajectory4d trajectory, double end, double radius, Point4D startP, Point4D endP) {
    double minx =
        multiMin(
            startP.getX() - radius,
            startP.getX() + radius,
            endP.getX() + radius,
            endP.getX() - radius);
    double maxx =
        multiMax(
            startP.getX() - radius,
            startP.getX() + radius,
            endP.getX() + radius,
            endP.getX() - radius);

    double miny =
        multiMin(
            startP.getY() - radius,
            startP.getY() + radius,
            endP.getY() + radius,
            endP.getY() - radius);
    double maxy =
        multiMax(
            startP.getY() - radius,
            startP.getY() + radius,
            endP.getY() + radius,
            endP.getY() - radius);

    double minz =
        multiMin(
            startP.getZ() - radius,
            startP.getZ() + radius,
            endP.getZ() + radius,
            endP.getZ() - radius);
    double maxz =
        multiMax(
            startP.getZ() - radius,
            startP.getZ() + radius,
            endP.getZ() + radius,
            endP.getZ() - radius);

    double mina =
        multiMin(
            startP.getAngle() - EPSILON,
            startP.getAngle() + EPSILON,
            endP.getAngle() + EPSILON,
            endP.getAngle() - EPSILON);
    double maxa =
        multiMax(
            startP.getAngle() - EPSILON,
            startP.getAngle() + EPSILON,
            endP.getAngle() + EPSILON,
            endP.getAngle() - EPSILON);
    verifyBounds(trajectory, end, minx, maxx, miny, maxy, minz, maxz, mina, maxz);
  }

  private double multiMin(double p1, double p2, double p3, double p4) {
    return Math.min(Math.min(p1, p2), Math.min(p3, p4));
  }

  private double multiMax(double p1, double p2, double p3, double p4) {
    return Math.max(Math.max(p1, p2), Math.max(p3, p4));
  }

  @Test
  public void testBoundsXYZComplexNYNZ() {
    double orientation = -Math.PI / 2;
    Point4D start = Point4D.create(0, 0, 5, orientation);
    Point3D end = Point3D.create(0, -5.0, 1.5);
    double radius = 0.5;
    double frequency = 0.1;
    double velocity = 0.1;
    this.trajectory =
        Trajectories.newCorkscrewTrajectory(start, end, velocity, radius, frequency, 0);
    verifyBounds(trajectory, 1000, radius, start, Point4D.from(end, orientation));
  }

  @Test
  public void testBoundsXYZComplexNP() {
    double orientation = -Math.PI / 2;
    Point4D start = Point4D.create(0, 0, 5, orientation);
    Point3D end = Point3D.create(0, 0, 1.5);
    double radius = 0.5;
    double frequency = 0.1;
    double velocity = 0.1;
    this.trajectory =
        Trajectories.newCorkscrewTrajectory(start, end, velocity, radius, frequency, 0);
    verifyBounds(trajectory, 1000, radius, start, Point4D.from(end, orientation));
  }

  @Test
  public void testBoundsXYZComplexNY() {
    double orientation = -Math.PI / 2;
    Point4D start = Point4D.create(0, 0, 5, orientation);
    Point3D end = Point3D.create(0, -5, 5);
    double radius = 0.5;
    double frequency = 0.1;
    double velocity = 0.1;
    this.trajectory =
        Trajectories.newCorkscrewTrajectory(start, end, velocity, radius, frequency, 0);
    verifyBounds(trajectory, 1000, radius, start, Point4D.from(end, orientation));
  }

  @Test
  public void testBoundsXYZComplexNX() {
    double orientation = -Math.PI / 2;
    Point4D start = Point4D.create(0, 0, 5, orientation);
    Point3D end = Point3D.create(-5, 0, 5);
    double radius = 0.5;
    double frequency = 0.1;
    double velocity = 0.1;
    this.trajectory =
        Trajectories.newCorkscrewTrajectory(start, end, velocity, radius, frequency, 0);
    verifyBounds(trajectory, 1000, radius, start, Point4D.from(end, orientation));
  }

  @Test
  public void testBoundsXYZComplexPX() {
    double orientation = -Math.PI / 2;
    Point4D start = Point4D.create(0, 0, 5, orientation);
    Point3D end = Point3D.create(5, 0, 5);
    double radius = 0.5;
    double frequency = 0.1;
    double velocity = 0.1;
    this.trajectory =
        Trajectories.newCorkscrewTrajectory(start, end, velocity, radius, frequency, 0);
    verifyBounds(trajectory, 1000, radius, start, Point4D.from(end, orientation));
  }

  @Test
  public void testBoundsXYZComplexPY() {
    double orientation = -Math.PI / 2;
    Point4D start = Point4D.create(0, 0, 5, orientation);
    Point3D end = Point3D.create(0, 5, 5);
    double radius = 0.5;
    double frequency = 0.1;
    double velocity = 0.1;
    this.trajectory =
        Trajectories.newCorkscrewTrajectory(start, end, velocity, radius, frequency, 0);
    verifyBounds(trajectory, 1000, radius, start, Point4D.from(end, orientation));
  }

  @Test
  public void testBoundsXYZComplexPZ() {
    double orientation = -Math.PI / 2;
    Point4D start = Point4D.create(0, 0, 1, orientation);
    Point3D end = Point3D.create(0, 0, 15);
    double radius = 0.5;
    double frequency = 0.1;
    double velocity = 0.1;
    this.trajectory =
        Trajectories.newCorkscrewTrajectory(start, end, velocity, radius, frequency, 0);
    verifyBounds(trajectory, 1000, radius, start, Point4D.from(end, orientation));
  }
}

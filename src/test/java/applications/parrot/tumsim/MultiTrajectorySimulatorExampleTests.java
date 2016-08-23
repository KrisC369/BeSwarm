package applications.parrot.tumsim;

import applications.trajectory.MultiTrajectoryServer;
import applications.trajectory.TestUtils;
import applications.trajectory.TrajectoryServer;
import com.google.common.collect.Lists;
import control.FiniteTrajectory4d;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.Parameterized;
import org.reflections.Reflections;
import org.slf4j.LoggerFactory;

import java.util.Collection;
import java.util.List;

import static org.junit.Assert.fail;

/**
 * Initialization tests for TumSim example files. This test uses reflection to scan the tumsim
 * package for implementations of the MultiTrajectoryServer and runs initialization tests on
 * instances of the found classes. These tests validate correct parameters passed to trajectory
 * creators before attempting to use such trajectories in simulation or real world experiments.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
@RunWith(Parameterized.class)
public class MultiTrajectorySimulatorExampleTests {
  private static final String PACKAGE_FQN = "applications.parrot.tumsim"; //TODO decide.
  private static final Class TEST_PARENT = MultiTrajectoryServer.class;

  private MultiTrajectoryServer server;

  public MultiTrajectorySimulatorExampleTests(Class cl) {
    try {
      this.server = (MultiTrajectoryServer) cl.getDeclaredConstructor().newInstance();
    } catch (Exception e) {
      fail(
          "Instantiation with default constructor should be possible for "
              + "TumSimulatorExamples");
    }
  }

  @Parameterized.Parameters
  public static Collection<? extends Class> getData() {
    Reflections reflections = new Reflections(PACKAGE_FQN);
    return reflections.getSubTypesOf(TEST_PARENT);
  }

  @Test
  public void testTrajectoryInitialization() {
    LoggerFactory.getLogger(MultiTrajectorySimulatorExampleTests.class)
        .info(
            "Running example initialization test for instances of " + server.getClass().getName());
    List<TrajectoryServer> allDifferentTrajectories = server.getAllDifferentTrajectories();
    List<FiniteTrajectory4d> trajectories = Lists.newArrayList();
    for (TrajectoryServer s : allDifferentTrajectories) {
      trajectories.add(s.getConcreteTrajectory());
    }
    TestUtils.testTrajectoryCollisions(trajectories);
  }
}

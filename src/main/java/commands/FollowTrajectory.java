package commands;

import com.google.common.base.Optional;
import commands.schedulers.PeriodicTaskRunner;
import control.DefaultPidParameters;
import control.PidController4d;
import control.PidParameters;
import control.Trajectory4d;
import control.dto.DroneState;
import control.dto.InertialFrameVelocity;
import control.localization.StateEstimator;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.VelocityService;

/**
 * Command for moving to a predefined pose. It is a facade which uses {@link Move}.
 *
 * @author Hoang Tung Dinh
 */
public final class FollowTrajectory implements Command {

    private static final Logger logger = LoggerFactory.getLogger(FollowTrajectory.class);

    private final VelocityService velocityService;
    private final StateEstimator stateEstimator;
    private final PidParameters pidLinearXParameters;
    private final PidParameters pidLinearYParameters;
    private final PidParameters pidLinearZParameters;
    private final PidParameters pidAngularZParameters;
    private final Trajectory4d trajectory4d;
    private final double durationInSeconds;
    private final double controlRateInSeconds;

    private static final double DEFAULT_CONTROL_RATE_IN_SECONDS = 0.05;

    private FollowTrajectory(Builder builder) {
        velocityService = builder.velocityService;
        stateEstimator = builder.stateEstimator;
        pidLinearXParameters = builder.pidLinearXParameters;
        pidLinearYParameters = builder.pidLinearYParameters;
        pidLinearZParameters = builder.pidLinearZParameters;
        pidAngularZParameters = builder.pidAngularZParameters;
        trajectory4d = builder.trajectory4d;
        durationInSeconds = builder.durationInSeconds;
        controlRateInSeconds = builder.controlRateInSeconds;
    }

    public static Builder builder() {
        return new Builder().controlRateInSeconds(DEFAULT_CONTROL_RATE_IN_SECONDS)
                .pidLinearXParameters(DefaultPidParameters.LINEAR_X.getParameters())
                .pidLinearYParameters(DefaultPidParameters.LINEAR_Y.getParameters())
                .pidLinearZParameters(DefaultPidParameters.LINEAR_Z.getParameters())
                .pidAngularZParameters(DefaultPidParameters.ANGULAR_Z.getParameters());
    }

    @Override
    public void execute() {
        logger.debug("Execute follow trajectory command.");

        final PidController4d pidController4d = PidController4d.builder()
                .linearXParameters(pidLinearXParameters)
                .linearYParameters(pidLinearYParameters)
                .linearZParameters(pidLinearZParameters)
                .angularZParameters(pidAngularZParameters)
                .trajectory4d(trajectory4d)
                .build();

        final Runnable computeNextResponse = new ComputeNextResponse(pidController4d);
        PeriodicTaskRunner.run(computeNextResponse, controlRateInSeconds, durationInSeconds);
    }

    private final class ComputeNextResponse implements Runnable {
        private final PidController4d pidController4d;

        private ComputeNextResponse(PidController4d pidController4d) {
            this.pidController4d = pidController4d;
        }

        @Override
        public void run() {
            logger.trace("Start a control loop.");
            final Optional<DroneState> currentState = stateEstimator.getCurrentState();
            if (!currentState.isPresent()) {
                logger.trace("Cannot get state.");
                return;
            }

            logger.debug("Got pose and velocity. Start computing the next velocity response.");
            final InertialFrameVelocity nextVelocity = pidController4d.compute(currentState.get().pose(),
                    currentState.get().inertialFrameVelocity());
            velocityService.sendVelocityMessage(nextVelocity);
        }
    }

    /**
     * {@code FollowTrajectory} builder static inner class.
     */
    public static final class Builder {
        private VelocityService velocityService;
        private StateEstimator stateEstimator;
        private PidParameters pidLinearXParameters;
        private PidParameters pidLinearYParameters;
        private PidParameters pidLinearZParameters;
        private PidParameters pidAngularZParameters;
        private Trajectory4d trajectory4d;
        private double durationInSeconds;
        private double controlRateInSeconds;

        private Builder() {}

        /**
         * Sets the {@code velocityService} and returns a reference to this Builder so that the methods can be
         * chained together.
         *
         * @param val the {@code velocityService} to set
         * @return a reference to this Builder
         */
        public Builder velocityService(VelocityService val) {
            velocityService = val;
            return this;
        }

        /**
         * Sets the {@code stateEstimator} and returns a reference to this Builder so that the methods can be chained
         * together.
         *
         * @param val the {@code stateEstimator} to set
         * @return a reference to this Builder
         */
        public Builder stateEstimator(StateEstimator val) {
            stateEstimator = val;
            return this;
        }

        /**
         * Sets the {@code pidLinearXParameters} and returns a reference to this Builder so that the methods can be
         * chained together.
         *
         * @param val the {@code pidLinearXParameters} to set
         * @return a reference to this Builder
         */
        public Builder pidLinearXParameters(PidParameters val) {
            pidLinearXParameters = val;
            return this;
        }

        /**
         * Sets the {@code pidLinearYParameters} and returns a reference to this Builder so that the methods can be
         * chained together.
         *
         * @param val the {@code pidLinearYParameters} to set
         * @return a reference to this Builder
         */
        public Builder pidLinearYParameters(PidParameters val) {
            pidLinearYParameters = val;
            return this;
        }

        /**
         * Sets the {@code pidLinearZParameters} and returns a reference to this Builder so that the methods can be
         * chained together.
         *
         * @param val the {@code pidLinearZParameters} to set
         * @return a reference to this Builder
         */
        public Builder pidLinearZParameters(PidParameters val) {
            pidLinearZParameters = val;
            return this;
        }

        /**
         * Sets the {@code pidAngularZParameters} and returns a reference to this Builder so that the methods can be
         * chained together.
         *
         * @param val the {@code pidAngularZParameters} to set
         * @return a reference to this Builder
         */
        public Builder pidAngularZParameters(PidParameters val) {
            pidAngularZParameters = val;
            return this;
        }

        /**
         * Sets the {@code trajectory4d} and returns a reference to this Builder so that the methods can be chained
         * together.
         *
         * @param val the {@code trajectory4d} to set
         * @return a reference to this Builder
         */
        public Builder trajectory4d(Trajectory4d val) {
            trajectory4d = val;
            return this;
        }

        /**
         * Sets the {@code durationInSeconds} and returns a reference to this Builder so that the methods can be
         * chained together.
         *
         * @param val the {@code durationInSeconds} to set
         * @return a reference to this Builder
         */
        public Builder durationInSeconds(double val) {
            durationInSeconds = val;
            return this;
        }

        /**
         * Sets the {@code controlRateInSeconds} and returns a reference to this Builder so that the methods can be
         * chained together.
         *
         * @param val the {@code controlRateInSeconds} to set
         * @return a reference to this Builder
         */
        public Builder controlRateInSeconds(double val) {
            controlRateInSeconds = val;
            return this;
        }

        /**
         * Returns a {@code FollowTrajectory} built from the parameters previously set.
         *
         * @return a {@code FollowTrajectory} built with parameters of this {@code FollowTrajectory.Builder}
         */
        public FollowTrajectory build() {return new FollowTrajectory(this);}
    }
}

package control;

import control.dto.InertialFrameVelocity;
import control.dto.Pose;
import control.dto.Velocity;
import utils.math.EulerAngle;

import static com.google.common.base.Preconditions.checkNotNull;

/**
 * A four-dimensional PID controller for the drone. It is the composition of 4 one-dimensional PID controller
 * {@link PidController1d} (three controllers for the three linear velocities, one controller for the
 * angular velocity).
 *
 * @author Hoang Tung Dinh
 */
public final class PidController4d {

    private final PidController1d pidLinearX;
    private final PidController1d pidLinearY;
    private final PidController1d pidLinearZ;
    private final PidController1d pidAngularZ;
    private final Trajectory1d angularTrajectoryZ;

    private PidController4d(Builder builder) {

        angularTrajectoryZ = builder.getTrajectoryAngularZ();
        pidLinearX = PidController1d.create(builder.linearXParameters, builder.getLinearTrajectoryX());
        pidLinearY = PidController1d.create(builder.linearYParameters, builder.getTrajectoryLinearY());
        pidLinearZ = PidController1d.create(builder.linearZParameters, builder.getTrajectoryLinearZ());
        pidAngularZ = PidController1d.create(builder.angularZParameters, angularTrajectoryZ);
    }

    public static Builder builder() {
        return new Builder();
    }

    /**
     * Compute the next velocity (response) of the control loop.
     *
     * @param currentPose     the current pose of the drone
     * @param currentVelocity the current velocity of the drone
     * @return the next velocity (response) of the drone
     */
    public InertialFrameVelocity compute(Pose currentPose, InertialFrameVelocity currentVelocity,
            double currentTimeInSeconds) {
        final double linearX = pidLinearX.compute(currentPose.x(), currentVelocity.linearX(), currentTimeInSeconds);
        final double linearY = pidLinearY.compute(currentPose.y(), currentVelocity.linearY(), currentTimeInSeconds);
        final double linearZ = pidLinearZ.compute(currentPose.z(), currentVelocity.linearZ(), currentTimeInSeconds);

        final double desiredYaw = angularTrajectoryZ.getDesiredPosition(currentTimeInSeconds);
        final double angularError = EulerAngle.computeAngleDistance(currentPose.yaw(), desiredYaw);
        final double adaptedCurrentYaw = desiredYaw - angularError;
        final double angularZ = pidAngularZ.compute(adaptedCurrentYaw, currentVelocity.angularZ(),
                currentTimeInSeconds);

        return Velocity.builder().linearX(linearX).linearY(linearY).linearZ(linearZ).angularZ(angularZ).build();
    }

    /**
     * {@code PidController4d} builder static inner class.
     */
    public static final class Builder {
        private PidParameters linearXParameters;
        private PidParameters linearYParameters;
        private PidParameters linearZParameters;
        private PidParameters angularZParameters;
        private Trajectory1d linearTrajectoryX;
        private Trajectory1d linearTrajectoryY;
        private Trajectory1d linearTrajectoryZ;
        private Trajectory1d angularTrajectoryZ;

        private Builder() {}

        public Trajectory1d getLinearTrajectoryX() {
            return linearTrajectoryX;
        }

        public Trajectory1d getTrajectoryLinearY() {
            return linearTrajectoryY;
        }

        public Trajectory1d getTrajectoryLinearZ() {
            return linearTrajectoryZ;
        }

        public Trajectory1d getTrajectoryAngularZ() {
            return angularTrajectoryZ;
        }

        /**
         * Sets the {@code trajectory4d} and returns a reference to this Builder so that the methods can be chained
         * together.
         *
         * @param val the {@code trajectory4d} to set
         * @return a reference to this Builder
         */
        public Builder trajectory4d(final Trajectory4d val) {

            linearTrajectoryX = new Trajectory1d() {

                @Override
                public double getDesiredVelocity(double timeInSeconds) {
                    return val.getDesiredVelocityX(timeInSeconds);
                }

                @Override
                public double getDesiredPosition(double timeInSeconds) {
                    return val.getDesiredPositionX(timeInSeconds);
                }
            };

            linearTrajectoryY = new Trajectory1d() {

                @Override
                public double getDesiredVelocity(double timeInSeconds) {
                    return val.getDesiredVelocityY(timeInSeconds);
                }

                @Override
                public double getDesiredPosition(double timeInSeconds) {
                    return val.getDesiredPositionY(timeInSeconds);
                }
            };

            linearTrajectoryZ = new Trajectory1d() {

                @Override
                public double getDesiredVelocity(double timeInSeconds) {
                    return val.getDesiredVelocityZ(timeInSeconds);
                }

                @Override
                public double getDesiredPosition(double timeInSeconds) {
                    return val.getDesiredPositionZ(timeInSeconds);
                }
            };

            angularTrajectoryZ = new Trajectory1d() {

                @Override
                public double getDesiredVelocity(double timeInSeconds) {
                    return val.getDesiredAngularVelocityZ(timeInSeconds);
                }

                @Override
                public double getDesiredPosition(double timeInSeconds) {
                    return val.getDesiredAngleZ(timeInSeconds);
                }
            };

            return this;
        }

        /**
         * Sets the {@code linearXParameters} and returns a reference to this Builder so that the methods can be
         * chained together.
         *
         * @param val the {@code linearXParameters} to set
         * @return a reference to this Builder
         */
        public Builder linearXParameters(PidParameters val) {
            linearXParameters = val;
            return this;
        }

        /**
         * Sets the {@code linearYParameters} and returns a reference to this Builder so that the methods can be
         * chained together.
         *
         * @param val the {@code linearYParameters} to set
         * @return a reference to this Builder
         */
        public Builder linearYParameters(PidParameters val) {
            linearYParameters = val;
            return this;
        }

        /**
         * Sets the {@code linearZParameters} and returns a reference to this Builder so that the methods can be
         * chained together.
         *
         * @param val the {@code linearZParameters} to set
         * @return a reference to this Builder
         */
        public Builder linearZParameters(PidParameters val) {
            linearZParameters = val;
            return this;
        }

        /**
         * Sets the {@code angularZParameters} and returns a reference to this Builder so that the methods can be
         * chained together.
         *
         * @param val the {@code angularZParameters} to set
         * @return a reference to this Builder
         */
        public Builder angularZParameters(PidParameters val) {
            angularZParameters = val;
            return this;
        }

        /**
         * Returns a {@code PidController4d} built from the parameters previously set.
         *
         * @return a {@code PidController4d} built with parameters of this {@code PidController4d.Builder}
         */
        public PidController4d build() {
            checkNotNull(linearXParameters, "missing linearXParameters");
            checkNotNull(linearYParameters, "missing linearYParameters");
            checkNotNull(linearZParameters, "missing linearZParameters");
            checkNotNull(angularZParameters, "missing angularZParameters");
            checkNotNull(linearTrajectoryX, "missing linearTrajectoryX");
            checkNotNull(linearTrajectoryY, "missing linearTrajectoryY");
            checkNotNull(linearTrajectoryZ, "missing linearTrajectoryZ");
            checkNotNull(angularTrajectoryZ, "missing angularTrajectoryZ");
            return new PidController4d(this);
        }
    }
}

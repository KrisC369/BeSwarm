package simulation.trajectory;

import control.Trajectory1d;
import control.Trajectory4d;

import static com.google.common.base.Preconditions.checkNotNull;

/**
 * Decorator for transforming the origin points of trajectories to transpose
 * them in space to form the trajectory using a new point as origin.
 *
 * @author Kristof Coninx <kristof.coninx AT cs.kuleuven.be>
 */
public class OriginTransformation extends Trajectory4DForwardingDecorator {

    private final double xOrigin;
    private final double yOrigin;
    private final double zOrigin;
    private final double angularZOrigin;

    private OriginTransformation(Trajectory4d target, double x,
            double y, double z, double angularZ) {
        super(target);
        this.xOrigin = x;
        this.yOrigin = y;
        this.zOrigin = z;
        this.angularZOrigin = angularZ;
    }

    public static OriginTransformationBuilder builder() {
        return new OriginTransformationBuilder();
    }

    @Override
    public Trajectory1d getTrajectoryLinearX() {
        return new Trajectory1d() {
            @Override public double getDesiredPosition(double timeInSeconds) {
                return xOrigin + getTrajectory4DDelegate()
                        .getTrajectoryLinearX()
                        .getDesiredPosition(timeInSeconds);
            }

            @Override public double getDesiredVelocity(double timeInSeconds) {
                return getTrajectory4DDelegate()
                        .getTrajectoryLinearX()
                        .getDesiredVelocity(timeInSeconds);
            }
        };
    }

    @Override
    public Trajectory1d getTrajectoryLinearY() {
        return new Trajectory1d() {
            @Override public double getDesiredPosition(double timeInSeconds) {
                return yOrigin + getTrajectory4DDelegate()
                        .getTrajectoryLinearY()
                        .getDesiredPosition(timeInSeconds);
            }

            @Override public double getDesiredVelocity(double timeInSeconds) {
                return getTrajectory4DDelegate()
                        .getTrajectoryLinearY()
                        .getDesiredVelocity(timeInSeconds);
            }
        };
    }

    @Override
    public Trajectory1d getTrajectoryLinearZ() {
        return new Trajectory1d() {
            @Override public double getDesiredPosition(double timeInSeconds) {
                return zOrigin + getTrajectory4DDelegate()
                        .getTrajectoryLinearZ()
                        .getDesiredPosition(timeInSeconds);
            }

            @Override public double getDesiredVelocity(double timeInSeconds) {
                return getTrajectory4DDelegate()
                        .getTrajectoryLinearZ()
                        .getDesiredVelocity(timeInSeconds);
            }
        };
    }

    @Override
    public Trajectory1d getTrajectoryAngularZ() {
        return new Trajectory1d() {
            @Override public double getDesiredPosition(double timeInSeconds) {
                return angularZOrigin + getTrajectory4DDelegate()
                        .getTrajectoryAngularZ()
                        .getDesiredPosition(timeInSeconds);
            }

            @Override public double getDesiredVelocity(double timeInSeconds) {
                return getTrajectory4DDelegate()
                        .getTrajectoryAngularZ()
                        .getDesiredVelocity(timeInSeconds);
            }
        };
    }

    public static class OriginTransformationBuilder {

        private double x;
        private double y;
        private double z;
        private double zAng;
        private Trajectory4d delegate;

        public OriginTransformationBuilder() {
            this.x = 0;
            this.y = 0;
            this.z = 0;
            this.zAng = 0;
        }

        public OriginTransformationBuilder forTrajectory(Trajectory4d target) {
            this.delegate = target;
            return this;
        }

        public OriginTransformationBuilder setXOrigin(double xOrigin) {
            this.x = xOrigin;
            return this;
        }

        public OriginTransformationBuilder setYOrigin(double yOrigin) {
            this.y = yOrigin;
            return this;
        }

        public OriginTransformationBuilder setZOrigin(double zOrigin) {
            this.z = zOrigin;
            return this;
        }

        public OriginTransformationBuilder setAngularZOrigin(double zOrigin) {
            this.zAng = zOrigin;
            return this;
        }

        public OriginTransformation build() {
            checkNotNull(this.delegate,
                    "Target for this decorator has not been set!");
            return new OriginTransformation(this.delegate, this.x, this.y,
                    this.z, this.zAng);
        }

    }
}

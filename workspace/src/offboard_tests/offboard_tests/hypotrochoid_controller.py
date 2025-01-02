"""Path controller for flying a spirograph
"""
import pdb

import numpy
import scipy.optimize

from geodesy.conversions import lla_to_ned
from geodesy.conversions import ned_to_lla
from geodesy.conversions import ned_to_enu
from geodesy.conversions import enu_to_ned
from geometry.lines import point_line_distance
from geometry.lines import get_point_on_line
import geometry.conversions
from helpers.arrays import ring_index

from robot_control.path_following import ParameterizedParkController

class SpirographController(ParameterizedParkController):
    """A parameterized path controller that flies spirographs
    """
    def __init__(
        self, X0, max_size, n_petals, direction, L, is_ned=True, theta_0=0.0):
        """Constructor

        Note that this will make a spirograph that always passes through the
        center point. By default it will start at the max bound on the x axis,
        initially going tangent to the bound in the -y direction (for a
        positive direction)

        Arguments:
            X0: the circle center point numpy (1,3) array (lla or ned depending
                on the argument is_ned)
            max_size: the maximum distance the aircraft will get from the center
                point. (m)
            n_petals: the number of "petals" on the spirograph to make
            direction: turn direction, sign of the yaw rate
            L: the lookahead distance on the path. (m)
            is_ned: optional flag indicating if the path is given in north,
                east, down coordinates. if false then lla is expected.
            theta_0: initial theta, defaults to zero

        Returns:
            class instance
        """
        assert isinstance(X0, numpy.ndarray), 'X0 must be numpy array'
        assert X0.shape == (1,3), 'X0 must be numpy (3,) array'

        super(SpirographController, self).__init__(
            self._spirograph,
            self._spirograph_accel,
            L,
            is_ned)
        self._X0 = X0

        self._n = n_petals
        self._d = max_size / 2.0
        self._r = max_size / 2.0 / (n_petals - 1)
        self._R = max_size + self._r - self._d
        self._theta = theta_0
        self._direction = numpy.sign(direction)

        if is_ned:
            shape_X0 = numpy.squeeze(self._X0)[:2]
        else:
            shape_X0 = numpy.zeros((2,))
        self._hypotrochoid = geometry.shapes.Hypotrochoid(
            self._R, self._r, self._d, self._direction, shape_X0)

    def _spirograph(self, state, params=None):
        """Compute the path location and direction for the controller

        A helper function, this will be called by an instance of
        ParameterizedParkController. It figures out where the nearest path
        location is relative to the aircraft and computes a vector that is
        along the path in the desired direction

        Arguments:
            state - (X,V):
                X: position
                V: inertial velocity of vehicle
            params: unused, here for compatibility

        Returns:
            (path, path_dot)
            path: closest point on the path
            path_dot: tangent to the path at that point
        """

        # REMEMBER: X given for the aircraft will be in LLA / NED but the
        # hypotrochoid is all in ENU!

        if not self._is_ned:
            X = lla_to_ned(state[0], numpy.array(self._X0, ndmin=2))[0]
        else:
            X = state[0]
        self._theta = self._compute_theta(X, self._theta)

        # compute the closest point on the path
        Xi = self._hypotrochoid.X(self._theta)
        Xi = enu_to_ned(numpy.hstack((Xi, 0.0)))

        if not self._is_ned:
            X_ac = lla_to_ned(state[0], self._X0)[0]
            Xi -= X_ac
            Xi[2] = 0.0

        # compute the path derivative at our desird point
        dX_dtheta = self._hypotrochoid.dX_dtheta(self._theta)
        ds_dtheta = self._hypotrochoid.ds_dtheta(self._theta)
        dX_ds = dX_dtheta / ds_dtheta
        dX_ds = enu_to_ned(numpy.hstack((dX_ds, 0.0)))

        return (Xi, dX_ds / numpy.linalg.norm(dX_ds))

    def _spirograph_accel(self, state, params=None):
        """ generate the acceleration required to stay parallel to this path

        A helper function, this will be called by an instance of
        ParameterizedParkController. It computes the acceleration required
        by the vehicle to remain parallel to the desired orbit (note that
        this is the acceleration required _At_its_distance_from_the_center_
        The error will be flown out by the park controller.

        Arguments:
            state - (X,V):
                X: numpy (3,) position
                V: numpy (3,) inertial velocity of vehicle
            params: unused, here for compatibility

        Returns:
            accel = numpy (3,) array indicating the required inertial
                acceleration
        """
        # compute the closest point on the path and the tangent to the path.
        # this will also update theta...
        Xi, T = self._spirograph(state, params)

        if not self._is_ned:
            X_ac = lla_to_ned(state[0], self._X0)[0]
            Xi += X_ac

        # find unit vector in direction of path center of curvature
        N_hat = (
            numpy.cross(T, numpy.array([0.0, 0.0, 1.0])) / numpy.linalg.norm(T))

        # compute the location of the center of curvature
        r = self._hypotrochoid.path_curvature(self._theta)
        X_r = Xi + N_hat * r * self._direction

        # project that out to the aircraft location and compute the required
        # acceleation
        if self._is_ned:
            R_ac = state[0] - X_r
        else:
            R_ac = X_ac - X_r
        v_tangent = T.dot(state[1])
        a_norm = numpy.power(v_tangent, 2.0) / numpy.linalg.norm(R_ac[:2])

        accel = N_hat * a_norm
        return accel

    def _compute_theta(self, X, theta_0):
        """Compute the parameter for the nearest point on the path

        Finds the parameter, theta, which computes the point on the path
        nearest to the specified position. The search for theta is limited to
            theta_0 - pi / n_petals < theta < theta_0 + pi / n_petals
        this ensures that we don't end up on the wrong branch. It does however
        require that we're reasonably close to the path.

        Arguments:
            X: position to search
            theta_0: initial parameter

        Returns:
            theta: best fit path parameter
        """
        theta = self._hypotrochoid.theta(
            numpy.squeeze(ned_to_enu(X))[:2], theta_0)
        return theta

    @property
    def theta(self):
        """get the current value for theta

        Arguments:
            no arguments

        Return:
            theta: path parameter
        """
        return self._theta

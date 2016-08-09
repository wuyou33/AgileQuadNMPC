/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_traj_control_main.cpp
 * Multicopter trajectory controller.
 *
 * Controller based on combining work from Mellinger and Kumar, and Lee, 
 * Leok, McClamroch. Uses continuous spline trajectories to perform a 
 * feedforward/feedback control system.
 *
 * @author Ross Allen <ross.allen@stanford.edu>
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/trajectory_nominal_values.h>
#include <uORB/topics/trajectory_spline.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/obstacle_repulsive_force_ned.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <mavlink/mavlink_log.h>
#include "mc_traj_control_params.h"
#include <vector>
#include <numeric>  // partial_sum
#include <limits>   // for inf

// parameters explicitly for work at ASL
#define ASL_LAB_CENTER_X    1.6283f
#define ASL_LAB_CENTER_Y    2.0630f
#define ASL_LAB_CENTER_Z    -1.5f
#define ASL_LAB_CENTER_YAW  -1.68f

// Spline initiation (positive value gives a delay for testing safety, 
// negative values retroactively sets start time to account for MPC iteration time
//~ #define SPLINE_START_DELAY 5000000
//#define SPLINE_START_DELAY 1000000
#define SPLINE_START_DELAY_T_SEC_REL 1.0f

// Transition time for smooth transition between two trajectories
// NOTE: this should be less than MPC time horizon in planner for proper behavior
#define SPLINE_TRANS_T_SEC_REL 0.5f

#define N_POLY_COEFS    10
#define _MICRO_ 0.000001f
#define GRAV 	9.81f

#define ZERO_GAIN_THRESHOLD 0.000001f

// TODO remove these later when I have an estimator for m and inertia
#define MASS_TEMP 0.9574f
//~ #define XY_INERTIA_TEMP 0.0018f
//~ #define XY_INERTIA_TEMP 0.0018f
//~ #define Z_INERTIA_TEMP 0.0037f
#define X_INERTIA_EST 0.00484f
#define Y_INERTIA_EST 0.00565f
#define Z_INERTIA_EST 0.1185f

// Trim bias for F330 converted from Sumeet's work
//#define ATTC_ROLL_BIAS 0.025f
#define ATTC_ROLL_BIAS 0.015f
#define ATTC_PITCH_BIAS -0.013f
#define ATTC_YAW_BIAS 0.06f

/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_traj_control_main(int argc, char *argv[]);

class MulticopterTrajectoryControl
{
public:
    /**
     * Constructor
     */
    MulticopterTrajectoryControl();

    /**
     * Destructor, also kills task.
     */
    ~MulticopterTrajectoryControl();

    /**
     * Start task.
     *
     * @return		OK on success.
     */
    int		start();

private:
    const float alt_ctl_dz = 0.2f;

    bool	_task_should_exit;		/**< if true, task should exit */
    int		_control_task;			/**< task handle for task */
    int		_mavlink_fd;			/**< mavlink fd */

    int		_att_sub;				/**< vehicle attitude subscription */
    int		_control_mode_sub;		/**< vehicle control mode subscription */
    int		_arming_sub;			/**< arming status of outputs */
    int		_local_pos_sub;			/**< vehicle local position */
    int     _traj_spline_sub;       /**< trajectory spline */
    int		_sensor_combined_sub;	/**< sensor data */
    int		_obs_force_sub;			/**< repulsive force from obstacles in world coords*/

    orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
    orb_advert_t	_att_rates_sp_pub;		/**< attitude rates setpoint publication */
    orb_advert_t	_local_pos_nom_pub;		/**< vehicle local position setpoint publication */
    orb_advert_t	_global_vel_sp_pub;		/**< vehicle global velocity setpoint publication */
    orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */
    orb_advert_t	_traj_nom_pub;			/**< nominal trajectory state and input values publication*/

    struct vehicle_attitude_s			_att;			/**< vehicle attitude */
    struct vehicle_attitude_setpoint_s	_att_sp;		/**< vehicle attitude setpoint */
    struct vehicle_rates_setpoint_s		_att_rates_sp;	/**< vehicle attitude rates setpoint */
    struct vehicle_control_mode_s		_control_mode;	/**< vehicle control mode */
    struct actuator_armed_s				_arming;		/**< actuator arming status */
    struct vehicle_local_position_s		_local_pos;		/**< vehicle local position */
    struct vehicle_local_position_setpoint_s	_local_pos_nom;		/**< vehicle local position setpoint */
    struct vehicle_global_velocity_setpoint_s	_global_vel_sp;	/**< vehicle global velocity setpoint */
    struct trajectory_spline_s  		_traj_spline;   /**< trajectory spline */
    struct actuator_controls_s			_actuators;		/**< actuator controls */
    struct trajectory_nominal_values_s	_traj_nom;		/**< nominal trajectory values */
    struct obstacle_repulsive_force_ned_s	_obs_force;		/**< repulsive force imposed due to obstacle proximity and relative velocity */
    struct sensor_combined_s 			_sensor;		/**< sensor data */ 


    struct {
        float thrust_min;
        float thrust_max;
        float tilt_max_air;
        math::Vector<3> vel_max;
        math::Vector<3> sp_offs_max;
        float gim_lock;
        float freefall_thresh;
        math::Vector<3> ang_int_limit;
        math::Vector<3> pos_int_limit;
    }		_safe_params;
    
    struct {
		math::Vector<3> pos_int;
        math::Vector<3> pos;
        math::Vector<3> vel;
        math::Vector<3> vel_der;
        math::Vector<3> ang_int;
        math::Vector<3> ang;
        math::Vector<3> omg;
        math::Vector<3> omg_der;
    }		_gains;

    struct map_projection_reference_s _ref_pos;
    float _ref_alt;
    hrt_abstime _ref_timestamp;
    //~ hrt_abstime _t_last_obs_force;

    bool _reset_pos_nom;
    bool _reset_alt_nom;
    bool _reset_psi_nom;
    bool _control_trajectory_started;
    bool _entered_trajectory_feedback_controller;
    bool _started_feedback;
    //~ bool _obs_force_timed_out;
    
    //NOTE: ELIMINATE VARIABLES THAT ARE PASSED AROUND 
    
    /* Actual properties and states */
    math::Vector<3> _pos;		/**< position of body frame origin in world coords */
    math::Vector<3> _vel;		/**< velocity of body frame origin wrt world, in world coords */
    math::Matrix<3, 3> _R_B2W;	/**< rotation matrix from body to world coords (axes aligned with quad arms)*/
    //~ math::Matrix<3, 3> _R_P2W;	/**< rotation matrix from PX4 body frame to world coords (axes bisect quad arms)*/
    //~ math::Matrix<3, 3> _R_B2P;	/**< rotation matrix from body frame to PX4 body frame (const)*/
    math::Vector<3> _x_body;	/**< x-axis of body frame in world coords */
    math::Vector<3> _y_body;	/**< y-axis of body frame in world coords */
    math::Vector<3> _z_body;	/**< z-axis of body frame in world coords */
    math::Vector<3> _Omg_body;	/**< angular velocity of body frame wrt world, in body frame */
    math::Vector<3> _Euler_angles;
    math::Vector<3> _Euler_rates;
    
    /* error terms for integral and derivative control */
    math::Vector<3> _omg_err_prev;  /**< angular velocity previous error */
    math::Vector<3> _pos_err_int;	/**< position error integral */
    math::Vector<3> _ang_err_int;	/**< angular error integral*/
    math::Vector<3> _vel_err_prev;  /**< velocity previous error */
    
    /* Nominal states, properties, and inputs */
    math::Vector<3> _pos_nom;		/**< nominal position */
    math::Vector<3> _vel_nom;		/**< nominal velocity */
    math::Vector<3> _acc_nom;       /**< nominal acceleration */
    math::Vector<3> _jerk_nom;      /**< nominal jerk (3rd derivative) */
    math::Vector<3> _snap_nom;   	/**< nominal snap (4rd derivative) */
    float _psi_nom;		/**< nominal yaw */
    float _psi1_nom;		/**< nominal yaw 1st derivative*/
    float _psi2_nom;		/**< nominal yaw 2nd derivative*/
    math::Matrix<3, 3> _R_N2W;		/**< rotation matrix from nominal to world frame */
    math::Vector<3> _Omg_nom;		/**< nominal angular velocity wrt to world, expressed in nominal */
    math::Vector<3> _Euler_des;
    math::Vector<3> _Euler_rate_des;

    math::Vector<3> _F_nom;			/**< nominal force expressed in world coords */
    math::Vector<3> _F_cor;			/**< corrective force in world coords */
    float			_uT_nom;		/**< nominal thrust setpoint */
    float			_uT_sp;		/**< thrust setpoint */
    math::Vector<3> _M_nom;			/**< nominal moment expressed in body coords */
    math::Vector<3> _M_cor;			/**< corrective moment expressed in body coords */
    math::Vector<3> _M_sp;			/**< moment setpoint in body coords */
    math::Vector<3>	_att_control;	/**< attitude control vector */

    /* Used in trajectory feedback controller */
    math::Vector<3> _u_prev;
    math::Vector<3> _Euler_des_prev;
    math::Vector<3> _om_des_prev;

    float _kx;
    float _kv;
    
    math::Matrix<6,6> _Mq;
    math::Matrix<6,3> _B_rot;

    //int _n_spline_seg;      /** < number of segments in spline (not max number, necassarily) */
    
    /* Dynamical properties of quadrotor*/
    float _mass;					/**< mass of quadrotor (kg) */
    math::Matrix<3, 3> _J_B;	/**< inertia matrix, fixed to body (kg*m^2) */
    
    /* Filter properties to estimate throttle mapping */
    float _k_thr;			/**< scaling factor for F/m = _k_thr*throttle */
    float _alpha;			/**< exponential decay weight for low pass filter */
    float _thr_prev;		/**< previous value of throttle input */
    
    // time vectors
    std::vector<float> _spline_delt_sec; // time step sizes for each segment
    std::vector<float> _spline_cumt_sec; // cumulative time markers for each segment
    float _spline_start_t_sec_abs;
    float _spline_term_t_sec_rel;
    std::vector<float> _prev_spline_delt_sec;
    std::vector<float> _prev_spline_cumt_sec;
    float _prev_spline_start_t_sec_abs;
    float _prev_spline_term_t_sec_rel;

    /* define vector of appropriate size for trajectory spline polynomials */
    // position 
    std::vector< std::vector<float> > _x_coefs;
    std::vector< std::vector<float> > _y_coefs;
    std::vector< std::vector<float> > _z_coefs;
    std::vector< std::vector<float> > _prev_x_coefs;
    std::vector< std::vector<float> > _prev_y_coefs;
    std::vector< std::vector<float> > _prev_z_coefs;
    
    // velocity
    std::vector< std::vector<float> > _xv_coefs;
    std::vector< std::vector<float> > _yv_coefs;
    std::vector< std::vector<float> > _zv_coefs;
    std::vector< std::vector<float> > _prev_xv_coefs;
    std::vector< std::vector<float> > _prev_yv_coefs;
    std::vector< std::vector<float> > _prev_zv_coefs;
    
    // acceleration
    std::vector< std::vector<float> > _xa_coefs;
    std::vector< std::vector<float> > _ya_coefs;
    std::vector< std::vector<float> > _za_coefs;
    std::vector< std::vector<float> > _prev_xa_coefs;
    std::vector< std::vector<float> > _prev_ya_coefs;
    std::vector< std::vector<float> > _prev_za_coefs;
    
    // jerk
    std::vector< std::vector<float> > _xj_coefs;
    std::vector< std::vector<float> > _yj_coefs;
    std::vector< std::vector<float> > _zj_coefs;
    std::vector< std::vector<float> > _prev_xj_coefs;
    std::vector< std::vector<float> > _prev_yj_coefs;
    std::vector< std::vector<float> > _prev_zj_coefs;
    
    // snap
    std::vector< std::vector<float> > _xs_coefs;
    std::vector< std::vector<float> > _ys_coefs;
    std::vector< std::vector<float> > _zs_coefs;
    std::vector< std::vector<float> > _prev_xs_coefs;
    std::vector< std::vector<float> > _prev_ys_coefs;
    std::vector< std::vector<float> > _prev_zs_coefs;
    
    // yaw and yaw derivatives
    std::vector< std::vector<float> > _yaw_coefs;
    std::vector< std::vector<float> > _yaw1_coefs;
    std::vector< std::vector<float> > _yaw2_coefs;
    std::vector< std::vector<float> > _prev_yaw_coefs;
    std::vector< std::vector<float> > _prev_yaw1_coefs;
    std::vector< std::vector<float> > _prev_yaw2_coefs;
    

    /**
     * Update control outputs
     */
    void		control_update();

    /**
     * Check for changes in subscribed topics.
     */
    void		poll_subscriptions(hrt_abstime t);

    /**
     * Update reference for local position projection
     */
    void		update_ref();
    
    /**
     * Reset position setpoint to current position
     */
    void		reset_pos_nom();

    /**
     * Reset altitude setpoint to current altitude
     */
    void		reset_alt_nom();
    
    /**
     * Reset yaw setpoint to current heading
     */
    void		reset_psi_nom();

    /**
     * Check if position setpoint is too far from current position and adjust it if needed.
     */
    void		limit_pos_nom_offset();

    /**
     * Cross and dot product between two vectors
     */
    math::Vector<3>	cross(const math::Vector<3>& v1, const math::Vector<3>& v2);
    float	dot(const math::Vector<3>& v1, const math::Vector<3>& v2);
    
    /**
     * convert SO(3) matrix to R3 vector. Note it does not check in M is element of SO(3)
     */
    math::Vector<3> vee_map(const math::Matrix<3, 3>& M);
    
    /**
     * set columns in matrix
     */
    void 	set_column(math::Matrix<3, 3>& R, unsigned int col, math::Vector<3>& v);

    /**
     * Set position setpoint using offboard control
     */

    bool		cross_sphere_line(const math::Vector<3>& sphere_c, float sphere_r,
                    const math::Vector<3> line_a, const math::Vector<3> line_b, math::Vector<3>& res);

    /**
     * Set position setpoint using trajectory control - Ross Allen
     */
    void        trajectory_nominal_state(float cur_spline_t, float spline_term_t, int cur_seg, float cur_poly_t, float poly_term_t, float dt);
    void		trajectory_transition_pos_vel(float t_trans, const std::vector<float>& x_trans_coefs, const std::vector<float>& y_trans_coefs, const std::vector<float>& z_trans_coefs,
					const std::vector<float>& vx_trans_coefs, const std::vector<float>& vy_trans_coefs, const std::vector<float>& vz_trans_coefs,
					const float dt);
    //void		dual_trajectory_transition_nominal_state( float cur_spline_t, int cur_seg, float cur_poly_t,
    //				float prev_spline_t, int prev_seg, float prev_poly_t);
    void		trajectory_feedback_controller();


    // One-step prediction for feedback controller initial step
    void 		feedback_fwd(math::Vector<3> pos_nom_next, math::Vector<3> vel_nom_next, math::Vector<3> acc_nom_next, float psi_nom_next,
    			const float dt, math::Vector<3>& Euler_rate_s, math::Vector<3>& Om_s);

    math::Matrix<3, 3>              R_om(math::Vector<3> x);
    float                           bq_1(math::Vector<3> x);
    float                           bq_2(math::Vector<3> x);
    float                           bq_3(math::Vector<3> x);
    math::Matrix<6, 6>              phi_d(math::Vector<6> x);
    math::Matrix<6, 6>              M(math::Vector<6> x);
    math::Vector<6>                 f_rot(math::Vector<6>);

    void        reset_trajectory();
    void		hold_position(const float dt);
    void		force_orientation_mapping(math::Matrix<3,3>& R_S2W, math::Vector<3>& Euler_s, math::Vector<3>& x_s, math::Vector<3>& y_s, math::Vector<3>& z_s,
                    float& uT_s, float& uT1_s, math::Vector<3>& Om_s, math::Vector<3>& Euler_rate_s,
                    math::Vector<3>& h_Omega,const math::Vector<3>& F_s, const float psi_s, const math::Vector<3>& Euler_s_prev, const float dt);
    
    /**
     * Evaluate polynomials
     * */
    float       poly_eval(const float coefs[N_POLY_COEFS], float t);
    float       poly_eval(const std::vector<float>& coefs, float t);
    void        vector_cum_sum(const std::vector<float>& vec, float initval, std::vector<float>& vecsum);
    void        poly_deriv(const std::vector< std::vector<float> >& poly, std::vector< std::vector<float> >& deriv);
    void        poly_deriv(const std::vector<float>& poly, std::vector<float>& deriv);
    

    /**
     * Shim for calling task_main from task_create.
     */
    static void	task_main_trampoline(int argc, char *argv[]);

    /**
     * Main sensor collection task.
     */
    void		task_main();
};

namespace traj_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

MulticopterTrajectoryControl	*g_control;
}

MulticopterTrajectoryControl::MulticopterTrajectoryControl() :

    _task_should_exit(false),
    _control_task(-1),
    _mavlink_fd(-1),

/* subscriptions */
    _att_sub(-1),
    _control_mode_sub(-1),
    _arming_sub(-1),
    _local_pos_sub(-1),
    _traj_spline_sub(-1),
    _sensor_combined_sub(-1),
    _obs_force_sub(-1),

/* publications */
    _att_sp_pub(-1),
    _att_rates_sp_pub(-1),
    _local_pos_nom_pub(-1),
    _global_vel_sp_pub(-1),
    _actuators_0_pub(-1),
    _traj_nom_pub(-1),

    _ref_alt(0.0f),
    _ref_timestamp(0),
    //~ _t_last_obs_force(0),

    _reset_pos_nom(true),
    _reset_alt_nom(true),
    _reset_psi_nom(true)
{
    memset(&_att, 0, sizeof(_att));
    memset(&_att_sp, 0, sizeof(_att_sp));
    memset(&_att_rates_sp, 0, sizeof(_att_rates_sp));
    memset(&_control_mode, 0, sizeof(_control_mode));
    memset(&_arming, 0, sizeof(_arming));
    memset(&_local_pos, 0, sizeof(_local_pos));
    memset(&_local_pos_nom, 0, sizeof(_local_pos_nom));
    memset(&_global_vel_sp, 0, sizeof(_global_vel_sp));
    memset(&_traj_spline, 0, sizeof(_traj_spline));
    memset(&_actuators, 0, sizeof(_actuators));
    memset(&_traj_nom, 0, sizeof(_traj_nom));
    memset(&_sensor, 0, sizeof(_sensor));
    memset(&_obs_force, 0, sizeof(_obs_force));

    memset(&_ref_pos, 0, sizeof(_ref_pos));
    
    /* retrieve control gains */
    _gains.pos_int(0) = TRAJ_GAINS_XY_POS_INT;
    _gains.pos_int(1) = TRAJ_GAINS_XY_POS_INT;
    _gains.pos_int(2) = TRAJ_GAINS_Z_POS_INT;
    _gains.pos(0) = TRAJ_GAINS_XY_POS;
    _gains.pos(1) = TRAJ_GAINS_XY_POS;
    _gains.pos(2) = TRAJ_GAINS_Z_POS;
    _gains.vel(0) = TRAJ_GAINS_XY_VEL;
    _gains.vel(1) = TRAJ_GAINS_XY_VEL;
    _gains.vel(2) = TRAJ_GAINS_Z_VEL;
    _gains.vel_der(0) = TRAJ_GAINS_XY_VEL_DER;
    _gains.vel_der(1) = TRAJ_GAINS_XY_VEL_DER;
    _gains.vel_der(2) = TRAJ_GAINS_Z_VEL_DER;
    _gains.ang_int(0) = TRAJ_GAINS_RP_ANG_INT;
    _gains.ang_int(1) = TRAJ_GAINS_RP_ANG_INT;
    _gains.ang_int(2) = TRAJ_GAINS_YAW_ANG_INT;
    _gains.ang(0) = TRAJ_GAINS_RP_ANG;
    _gains.ang(1) = TRAJ_GAINS_RP_ANG;
    _gains.ang(2) = TRAJ_GAINS_YAW_ANG;
    _gains.omg(0) = TRAJ_GAINS_RP_OMG;
    _gains.omg(1) = TRAJ_GAINS_RP_OMG;
    _gains.omg(2) = TRAJ_GAINS_YAW_OMG;
    _gains.omg_der(0) = TRAJ_GAINS_RP_OMG_DER;
    _gains.omg_der(1) = TRAJ_GAINS_RP_OMG_DER;
    _gains.omg_der(2) = TRAJ_GAINS_YAW_OMG_DER;

    /* retrieve safety parameters */
    _safe_params.thrust_min = 0.0f;
    _safe_params.thrust_max = 0.0f;
    _safe_params.tilt_max_air = TRAJ_PARAMS_TILTMAX_AIR;
    _safe_params.vel_max(0) = TRAJ_PARAMS_XY_VEL_MAX;
    _safe_params.vel_max(1) = TRAJ_PARAMS_XY_VEL_MAX;
    _safe_params.vel_max(2) = TRAJ_PARAMS_Z_VEL_MAX;
    if (_gains.pos.length() > ZERO_GAIN_THRESHOLD) {
        _safe_params.sp_offs_max = _safe_params.vel_max.edivide(_gains.pos) * 2.0f;
    } else {
        _safe_params.sp_offs_max = _safe_params.vel_max*1.0f;
    }
    _safe_params.gim_lock = TRAJ_PARAMS_GIMBAL_LOCK;
    _safe_params.freefall_thresh = TRAJ_PARAMS_FREEFALL_THRESHOLD;
    _safe_params.pos_int_limit(0) = TRAJ_PARAMS_XY_INT_LIMIT;
    _safe_params.pos_int_limit(1) = TRAJ_PARAMS_XY_INT_LIMIT;
    _safe_params.pos_int_limit(2) = TRAJ_PARAMS_Z_INT_LIMIT;
    _safe_params.ang_int_limit(0) = TRAJ_PARAMS_RP_INT_LIMIT;
    _safe_params.ang_int_limit(1) = TRAJ_PARAMS_RP_INT_LIMIT;
    _safe_params.ang_int_limit(2) = TRAJ_PARAMS_YAW_INT_LIMIT;
    
    _entered_trajectory_feedback_controller = false;
    _started_feedback = false;

    _pos.zero();
    _vel.zero();
    _R_B2W.identity();
    _x_body.zero();	_x_body(0) = 1.0f;
    _y_body.zero();	_y_body(1) = 1.0f;
    _z_body.zero(); _z_body(2) = 1.0f;
    _Omg_body.zero();
    _Euler_angles.zero();
    _Euler_rates.zero();

    //~ _R_P2W.identity();
    //~ _R_B2P.identity();
    //~ _R_B2P(0,0) = (float)cos(M_PI_4);
    //~ _R_B2P(0,1) = (float)sin(M_PI_4);
    //~ _R_B2P(1,0) = -(float)sin(M_PI_4);
    //~ _R_B2P(1,1) = (float)cos(M_PI_4);
    
    _omg_err_prev.zero();
    _ang_err_int.zero();
    _pos_err_int.zero();
    _vel_err_prev.zero();

    _pos_nom.zero();
    _vel_nom.zero();
    _acc_nom.zero();
    _jerk_nom.zero();
    _snap_nom.zero();
    _psi_nom = 0.0f;
    _psi1_nom = 0.0f;
    _psi2_nom = 0.0f;
    _R_N2W.identity();

    _Omg_nom.zero();
    _Euler_des.zero();
    _Euler_rate_des.zero();

    _u_prev.zero();
    _Euler_des_prev.zero();
    _om_des_prev.zero();

    _kx = 0.0f;
    _kv = 0.0f;
    _Mq.identity();
    _B_rot.zero();

    _F_nom.zero();
    _F_cor.zero();
    _uT_nom = 0.0f;
    _uT_sp = 0.0f;
    _M_nom.zero();
    _M_cor.zero();
    _M_sp.zero();
    _att_control.zero();

    //_n_spline_seg = 0;
    
    _mass = 0.0f;
    _J_B.identity();
    _alpha = THROTTLE_FILTER_SMOOTHING;
    _k_thr = 0.0f;
    _thr_prev = 0.0f;
    
    _spline_start_t_sec_abs = 0.0f;
    _spline_term_t_sec_rel = 0.0f;
    _prev_spline_start_t_sec_abs = 0.0f;
    _prev_spline_term_t_sec_rel = 0.0f;

}

MulticopterTrajectoryControl::~MulticopterTrajectoryControl()
{
    if (_control_task != -1) {
        /* task wakes up stanford outdoors trip sign upevery 100ms or so at the longest */
        _task_should_exit = true;

        /* wait for a second for the task to quit at our request */
        unsigned i = 0;

        do {
            /* wait 20ms */
            usleep(20000);

            /* if we have given up, kill it */
            if (++i > 50) {
                task_delete(_control_task);
                break;
            }
        } while (_control_task != -1);
    }

    traj_control::g_control = nullptr;
}

void
MulticopterTrajectoryControl::poll_subscriptions(hrt_abstime t)
{
    bool updated;

    orb_check(_att_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);
        
        // access current body rotation and account for px4 angular offset from body frame
        //~ _R_P2W.set(_att.R);	// PX4 body frame to world
        //~ _R_B2W = _R_P2W*_R_B2P;	// body frame to world accounting for px4 body frame
        //~ _R_B2W.set(_att.R);
        _R_B2W.from_euler(_att.roll, _att.pitch, _att.yaw);
        for (int i = 0; i < 3; i++) {
            _x_body(i) = _R_B2W(i,0);
            _y_body(i) = _R_B2W(i,1);
            _z_body(i) = _R_B2W(i,2);
        }

        _Euler_angles(0) = _att.roll;
        _Euler_angles(1) = _att.pitch;
        _Euler_angles(2) = _att.yaw;

        _Euler_rates(0) = _att.rollspeed;
        _Euler_rates(1) = _att.pitchspeed;
        _Euler_rates(2) = _att.yawspeed;

        _Omg_body = R_om(_Euler_angles)*_Euler_rates;
    }

    orb_check(_control_mode_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
    }


    orb_check(_arming_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(actuator_armed), _arming_sub, &_arming);
    }

    orb_check(_local_pos_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
    }
    
    orb_check(_traj_spline_sub, &updated);
    
    if (updated) {
        orb_copy(ORB_ID(trajectory_spline), _traj_spline_sub, &_traj_spline);
        _control_trajectory_started = false;
        _entered_trajectory_feedback_controller = false;
    }
    
    orb_check(_obs_force_sub, &updated);
    
    if (updated) {
		orb_copy(ORB_ID(obstacle_repulsive_force_ned), _obs_force_sub, &_obs_force);
		//~ _t_last_obs_force = t;
		//~ _obs_force_timed_out = false;
	}
	//~ } else {
		//~ // Check for topic timeout
		//~ if (!_obs_force_timed_out && t > _t_last_obs_force + OBS_FORCE_TIMEOUT) {
			//~ memset(&_obs_force, 0, sizeof(_obs_force));
			//~ _obs_force_timed_out = true;
		//~ }
	//~ }
    
    orb_check(_sensor_combined_sub, &updated);
    
    if (updated) {
		orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor);
	}
    
}

void
MulticopterTrajectoryControl::task_main_trampoline(int argc, char *argv[])
{
    traj_control::g_control->task_main();
}

void
MulticopterTrajectoryControl::update_ref()
{
    if (_local_pos.ref_timestamp != _ref_timestamp) {
        double lat_sp, lon_sp;
        float alt_nom = 0.0f;

        if (_ref_timestamp != 0) {
            /* calculate current position setpoint in global frame */
            map_projection_reproject(&_ref_pos, _pos_nom(0), _pos_nom(1), &lat_sp, &lon_sp);
            alt_nom = _ref_alt - _pos_nom(2);
        }

        /* update local projection reference */
        map_projection_init(&_ref_pos, _local_pos.ref_lat, _local_pos.ref_lon);
        _ref_alt = _local_pos.ref_alt;

        if (_ref_timestamp != 0) {
            /* reproject position setpoint to new reference */
            map_projection_project(&_ref_pos, lat_sp, lon_sp, &_pos_nom.data[0], &_pos_nom.data[1]);
            _pos_nom(2) = -(alt_nom - _ref_alt);
        }

        _ref_timestamp = _local_pos.ref_timestamp;
    }
}

void
MulticopterTrajectoryControl::reset_pos_nom()
{
    if (_reset_pos_nom) {
        _reset_pos_nom = false;
        if (_gains.pos.length() > ZERO_GAIN_THRESHOLD && _gains.vel.length() > ZERO_GAIN_THRESHOLD){
            /* shift position setpoint to make attitude setpoint continuous */
            _pos_nom(0) = _pos(0) + (_vel(0) - _vel_nom(0) - 
                    _R_N2W(0,2) * _uT_sp / _gains.vel(0)) / _gains.pos(0);
            _pos_nom(1) = _pos(1) + (_vel(1) - _vel_nom(1) - 
                    _R_N2W(0,1) * _uT_sp / _gains.vel(1)) / _gains.pos(1);
        } else {
            _pos_nom(0) = _pos(0);
            _pos_nom(1) = _pos(1);
        }
        mavlink_log_info(_mavlink_fd, "[mtc] reset pos sp: %.2f, %.2f", (double)_pos_nom(0), (double)_pos_nom(1));
    }
}

void
MulticopterTrajectoryControl::reset_alt_nom()
{
    if (_reset_alt_nom) {
        _reset_alt_nom = false;
        if (_gains.pos.length() > ZERO_GAIN_THRESHOLD) {
            _pos_nom(2) = _pos(2) + (_vel(2) - _vel_nom(2)) / _gains.pos(2);
        } else {
            _pos_nom(2) = _pos(2);
        }
        mavlink_log_info(_mavlink_fd, "[mtc] reset alt sp: %.2f", -(double)_pos_nom(2));
    }
}

void
MulticopterTrajectoryControl::reset_psi_nom()
{
    if (_reset_psi_nom) {
        _reset_psi_nom = false;
        _psi_nom = _att.yaw;
    }
}

void
MulticopterTrajectoryControl::limit_pos_nom_offset()
{
    math::Vector<3> pos_sp_offs;
    pos_sp_offs.zero();


    pos_sp_offs(0) = (_pos_nom(0) - _pos(0)) / _safe_params.sp_offs_max(0);
    pos_sp_offs(1) = (_pos_nom(1) - _pos(1)) / _safe_params.sp_offs_max(1);

    pos_sp_offs(2) = (_pos_nom(2) - _pos(2)) / _safe_params.sp_offs_max(2);


    float pos_sp_offs_norm = pos_sp_offs.length();

    if (pos_sp_offs_norm > 1.0f) {
        pos_sp_offs /= pos_sp_offs_norm;
        _pos_nom = _pos + pos_sp_offs.emult(_safe_params.sp_offs_max);
    }
}

math::Vector<3> 
MulticopterTrajectoryControl::cross(const math::Vector<3>& v1, const math::Vector<3>& v2)
{
    /* cross product between two vectors*/
    math::Vector<3> res;
    
    res(0) = v1(1)*v2(2) - v2(1)*v1(2);
    res(1) = v2(0)*v1(2) - v1(0)*v2(2);
    res(2) = v1(0)*v2(1) - v2(0)*v1(1);
    
    return res;

}

float
MulticopterTrajectoryControl::dot(const math::Vector<3>& v1,
        const math::Vector<3>& v2)
{
    /* dot product between two vectors*/
    float res;
    
    res = v1*v2;
    
    return res;
    
}

math::Vector<3>
MulticopterTrajectoryControl::vee_map(const math::Matrix<3, 3>& M)
{
    /* convert SO(3) matrix to R3 */
    math::Vector<3> res;	res.zero();
    res(0) = M(2, 1);
    res(1) = M(0, 2);
    res(2) = M(1, 0);
    
    return res;
}


bool
MulticopterTrajectoryControl::cross_sphere_line(const math::Vector<3>& sphere_c, float sphere_r,
        const math::Vector<3> line_a, const math::Vector<3> line_b, math::Vector<3>& res)
{
    /* project center of sphere on line */
    /* normalized AB */
    math::Vector<3> ab_norm = line_b - line_a;
    ab_norm.normalize();
    math::Vector<3> d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
    float cd_len = (sphere_c - d).length();

    /* we have triangle CDX with known CD and CX = R, find DX */
    if (sphere_r > cd_len) {
        /* have two roots, select one in A->B direction from D */
        float dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);
        res = d + ab_norm * dx_len;
        return true;

    } else {
        /* have no roots, return D */
        res = d;
        return false;
    }
}

/* set a column in a matrix */
void
MulticopterTrajectoryControl::set_column(math::Matrix<3, 3>& R, unsigned int col, math::Vector<3>& v)
{
    for (unsigned i = 0; i < 3; i++) {
        R(i, col) = v(i);
    }
    
}


/* Added by Ross Allen */
void
MulticopterTrajectoryControl::reset_trajectory()
{
        
    memset(&_traj_spline, 0, sizeof(_traj_spline));
    _control_trajectory_started = false;
    _entered_trajectory_feedback_controller = false;
    _reset_pos_nom = true;
    _reset_alt_nom = true;
    _reset_psi_nom = true;
    _pos.zero();
    _vel.zero();
    _R_B2W.identity();
    _x_body.zero();	_x_body(0) = 1.0f;
    _y_body.zero();	_y_body(1) = 1.0f;
    _z_body.zero(); _z_body(2) = 1.0f;

    _Omg_body.zero();
    _Euler_angles.zero();
    _Euler_rates.zero();

    _pos_nom.zero();
    _vel_nom.zero();
    _acc_nom.zero();
    _jerk_nom.zero();
    _snap_nom.zero();
    _psi_nom = 0.0f;
    _psi1_nom = 0.0f;
    _psi2_nom = 0.0f;
    _R_N2W.identity();

    _Omg_nom.zero();
    _Euler_des.zero();
    _Euler_rate_des.zero();

    _entered_trajectory_feedback_controller = false;
    _started_feedback = false;

    _u_prev.zero();
    _Euler_des_prev.zero();
    _om_des_prev.zero();

    _F_nom.zero();
    _F_cor.zero();
    _uT_nom = 0.0f;
    _uT_sp = 0.0f;
    _M_nom.zero();
    _M_cor.zero();
    _M_sp.zero();
    _att_control.zero();
    _k_thr = 1.0f/(TRAJ_PARAMS_THROTTLE_PER_THRUST*_mass);
    _thr_prev = TRAJ_PARAMS_THROTTLE_PER_THRUST*_mass*GRAV;
    memset(&_obs_force, 0, sizeof(_obs_force));
    
    // reset angular and position error integrals
    _omg_err_prev.zero();
    _ang_err_int.zero();
    _pos_err_int.zero();
    _vel_err_prev.zero();
        
}

float
MulticopterTrajectoryControl::poly_eval(const std::vector<float>& coefs, float t)
{
    
    typedef std::vector<float>::size_type vecf_sz;
    vecf_sz deg = coefs.size()-1;

    
    // initialize return value
    float p = coefs.at(deg);
    
    for(vecf_sz i = deg-1; ; --i){
        
        // Calculate with Horner's Rule
        p = p*t + coefs.at(i);
        
        // Check break condition.
        // This clunky for a for loop but is necessary because vecf_sz
        // in some form of unsigned int. If the condition is left as
        // i >= 0, then the condition is always true
        if (i == 0) break;
    }

    return p;
}

float
MulticopterTrajectoryControl::poly_eval(const float coefs_arr[N_POLY_COEFS], float t)
{

    // degree of polynomial
    unsigned deg = N_POLY_COEFS-1;
    
    // initialize return value
    float p = coefs_arr[deg];
    
    for(unsigned i = deg-1; ; --i){
        
        // Calculate with Horner's Rule
        p = p*t + coefs_arr[i];
        
        // Check break condition.
        // This clunky for a for loop but is necessary because vecf_sz
        // in some form of unsigned int. If the condition is left as
        // i >= 0, then the condition is always true
        if (i == 0) break;
    }

    return p;
}

/* Cumulative sum over vector with initial value */
void
MulticopterTrajectoryControl::vector_cum_sum(
const std::vector<float>& vec, float initval, std::vector<float>& vecsum){
    
    vecsum = vec;   // overwrite anything currently in vecsum
    vecsum.at(0) += initval;
    std::partial_sum(vecsum.begin(), vecsum.end(), vecsum.begin());
}

/* Evaluate derivatives of a polynomial */
void
MulticopterTrajectoryControl::poly_deriv(
const std::vector< std::vector<float> >& poly,
std::vector< std::vector<float> >& deriv) {

    
    typedef std::vector<float>::size_type vecf_sz;
    typedef std::vector< std::vector<float> >::size_type vecf2d_sz;

    
    vecf2d_sz numrows = poly.size();
    deriv.resize(numrows);      // resize number rows

    
    for (vecf2d_sz row = 0; row != numrows; ++row) {
        
        vecf_sz numcols = poly.at(row).size();
        deriv.at(row).resize(numcols);
        
        for (vecf_sz col = 0; col != numcols; ++col) {
            
            deriv.at(row).at(col) = ((float)col)*poly.at(row).at(col);
        }
        
        
        // remove first element
        deriv.at(row).erase(deriv.at(row).begin());
    }
    
}

/* Evaluate derivatives of a polynomial */
void
MulticopterTrajectoryControl::poly_deriv(
const std::vector<float>& poly, std::vector<float>& deriv) {

    
    typedef std::vector<float>::size_type vecf_sz;

	vecf_sz numcols = poly.size();
	deriv.resize(numcols);
	
	for (vecf_sz col = 0; col != numcols; ++col) {
		deriv.at(col) = ((float)col)*poly.at(col);
	}
	
	
	// remove first element
	deriv.erase(deriv.begin());
    
}

void
MulticopterTrajectoryControl::feedback_fwd(math::Vector<3> pos_nom_next, math::Vector<3> vel_nom_next, math::Vector<3> acc_nom_next, float psi_nom_next,
		const float dt, math::Vector<3>& Euler_rate_s, math::Vector<3>& Om_s)
{

	math::Vector<3> z_W;	z_W.zero();
	z_W(2) = 1.0f;

	math::Vector<3> bq_temp({ bq_1(_Euler_angles), bq_2(_Euler_angles), bq_3(_Euler_angles) });
	math::Vector<3> a_pred = z_W*GRAV + bq_temp*_uT_nom*((float)-1.0 / _mass);
	math::Vector<3> x_pred = _pos + _vel * dt;
	math::Vector<3> v_pred = _vel + a_pred * dt;

	math::Vector<3> err_x_pred = x_pred - pos_nom_next;
	math::Vector<3> err_v_pred = v_pred - vel_nom_next;

	math::Vector<3> a_des = acc_nom_next;

	math::Vector<3> F_next = -err_x_pred *_kx - err_v_pred * _kv - z_W*_mass*GRAV + a_des*_mass;

	math::Vector<3> Euler_des_next = _Euler_des; // just for initialization

	math::Vector<3> z_s_next = -F_next.normalized();	// (eqn 15)

	float yaw_des_next = psi_nom_next;
	float pitch_des_next = atan2((z_s_next(0) * (float)cos(yaw_des_next) + z_s_next(1) * (float)sin(yaw_des_next)), (z_s_next(2)));
	float roll_des_next = atan2(z_s_next(0) * (float)sin(yaw_des_next) - z_s_next(1) * (float)cos(yaw_des_next), z_s_next(2) / (float)cos(pitch_des_next));

	Euler_des_next(0) = roll_des_next;
	Euler_des_next(1) = pitch_des_next;
	Euler_des_next(2) = yaw_des_next;

	Euler_rate_s = (Euler_des_next-_Euler_des)/dt;
	Om_s = R_om(_Euler_des)*Euler_rate_s;
}

/* hold position */
void
MulticopterTrajectoryControl::hold_position(const float dt)
{
    // reset position and alt if necessary
    reset_pos_nom();
    reset_alt_nom();
    reset_psi_nom();
    
    // set position derivatives to zero
    _vel_nom.zero();
    _acc_nom.zero();
    _jerk_nom.zero();
    _snap_nom.zero();
    _psi1_nom = 0.0f;
    _psi2_nom = 0.0f;
    
    // Force to just counteract gravity
    _F_nom.zero();
    _F_nom(2) = -_mass*GRAV;
    math::Vector<3> x_nom;	x_nom.zero();
    math::Vector<3> y_nom;	y_nom.zero();
    math::Vector<3> z_nom;	z_nom.zero();
    float uT1_nom = 0.0f;
    math::Vector<3> h_Omega; 	h_Omega.zero();
    force_orientation_mapping(_R_N2W,_Euler_des, x_nom, y_nom, z_nom,
                    _uT_nom, uT1_nom, _Omg_nom, _Euler_rate_des, h_Omega,
                    _F_nom, _psi_nom, _Euler_des_prev, dt);
   
    math::Vector<3> ang_a; ang_a.zero();
    if (_started_feedback || _entered_trajectory_feedback_controller){
    	// can trust _Omg_nom & _Euler_rate_des from orientation mapping
		ang_a = (_Omg_nom - _om_des_prev)/dt;
    } else {
    	_started_feedback = true;

    	math::Vector<3> pos_nom_next = _pos_nom;
    	math::Vector<3> vel_nom_next = _vel_nom;
    	math::Vector<3> acc_nom_next = _acc_nom;
    	float psi_nom_next = _psi_nom;
    	feedback_fwd(pos_nom_next,vel_nom_next,acc_nom_next,psi_nom_next,dt,_Euler_rate_des,_Omg_nom);
    	ang_a.zero();
    }
    
    // nominal moment input
	_M_nom = _J_B*ang_a + cross(_Omg_nom, _J_B*_Omg_nom);

	// Update previous
	_om_des_prev = _Omg_nom;
	_Euler_des_prev = _Euler_des;
        
}

/* Calculate thrust input, orientation matrix and angular velocity based on a force vector */
void
MulticopterTrajectoryControl::force_orientation_mapping(
        math::Matrix<3,3>& R_S2W, math::Vector<3>& Euler_s, math::Vector<3>& x_s, math::Vector<3>& y_s, math::Vector<3>& z_s,
        float& uT_s, float& uT1_s, math::Vector<3>& Om_s, math::Vector<3>& Euler_rate_s, math::Vector<3>& h_Omega,
        const math::Vector<3>& F_s, const float psi_s, const math::Vector<3>& Euler_s_prev,
        const float dt)
{

        // nominal thrust input
        uT_s = -dot(_z_body, F_s);
        
        // nominal body axis in world frame
        z_s = -F_s.normalized();	// (eqn 15)

        //TODO: put gimbal lock check here

        float yaw_des = psi_s;
        float pitch_des = atan2((z_s(0) * (float)cos(yaw_des) + z_s(1) * (float)sin(yaw_des)), (z_s(2)));
        float roll_des = atan2(z_s(0) * (float)sin(yaw_des) - z_s(1) * (float)cos(yaw_des), z_s(2) / (float)cos(pitch_des));

        Euler_s(0) = roll_des; Euler_s(1) = pitch_des; Euler_s(2) = yaw_des;

        R_S2W.from_euler(roll_des,pitch_des,yaw_des);
        for (int i=0;i<3;i++){
        	x_s(i) = R_S2W(i,0);
        	y_s(i) = R_S2W(i,1);
        }
        
        Euler_rate_s = (Euler_s - Euler_s_prev)/dt;

        // nominal angular velocity
        uT1_s = -_mass*dot(_jerk_nom, z_s);
        h_Omega = -(z_s*uT1_s + _jerk_nom*_mass)*(1.0f/uT_s);

        Om_s = R_om(Euler_s)*Euler_rate_s;
}

/* Calculate the nominal state variables for the trajectory at the current time */
void
MulticopterTrajectoryControl::trajectory_nominal_state(
	float cur_spline_t, float spline_term_t, int cur_seg, float cur_poly_t, float poly_term_t, float dt)
{
       
    // local variables
    math::Vector<3> x_nom;	x_nom.zero();	/**< nominal body x-axis */
    math::Vector<3> y_nom;	y_nom.zero();	/**< nominal body x-axis */
    math::Vector<3> z_nom;	z_nom.zero();	/**< nominal body x-axis */
    float uT1_nom = 0.0f;	/**< 1st deriv of nominal thrust input */
    math::Vector<3> h_Omega; 	h_Omega.zero();
    math::Vector<3> z_W;	z_W.zero();
    z_W(2) = 1.0f;
    
    
    if (cur_spline_t <= 0) {
        
        _pos_nom(0) = poly_eval(_x_coefs.at(0), 0.0f);
        _pos_nom(1) = poly_eval(_y_coefs.at(0), 0.0f);
        _pos_nom(2) = poly_eval(_z_coefs.at(0), 0.0f);
        _psi_nom = poly_eval(_yaw_coefs.at(0), 0.0f);
        
        _reset_pos_nom = false;
        _reset_alt_nom = false;
        _reset_psi_nom = false;
        
        hold_position(dt);
        
    } else if (cur_spline_t > 0 && cur_spline_t < spline_term_t) {
    
        // nominal position
        _pos_nom(0) = poly_eval(_x_coefs.at(cur_seg), cur_poly_t);
        _pos_nom(1) = poly_eval(_y_coefs.at(cur_seg), cur_poly_t);
        _pos_nom(2) = poly_eval(_z_coefs.at(cur_seg), cur_poly_t);
        
        // nominal velocity
        _vel_nom(0) = poly_eval(_xv_coefs.at(cur_seg), cur_poly_t);
        _vel_nom(1) = poly_eval(_yv_coefs.at(cur_seg), cur_poly_t);
        _vel_nom(2) = poly_eval(_zv_coefs.at(cur_seg), cur_poly_t);
        
        // nominal acceleration
        _acc_nom(0) = poly_eval(_xa_coefs.at(cur_seg), cur_poly_t);
        _acc_nom(1) = poly_eval(_ya_coefs.at(cur_seg), cur_poly_t);
        _acc_nom(2) = poly_eval(_za_coefs.at(cur_seg), cur_poly_t);

        
        // nominal force in world frame (eqn 16)
        math::Vector<3> ex = _pos - _pos_nom;
        math::Vector<3> ev = _vel - _vel_nom;

        _F_nom = _acc_nom*_mass - z_W*(_mass*GRAV) - ex*_kx - ev*_kv;

        if (_F_nom(2) > 0.0f || _F_nom.length()/_mass < _safe_params.freefall_thresh) {
            // stabilize min thrust "free fall"
            _F_nom.zero();
            _F_nom(2) = -_safe_params.thrust_min;
        }
        
        // nominal thrust, orientation, and angular velocity
        force_orientation_mapping(_R_N2W,_Euler_des, x_nom, y_nom, z_nom,
                            _uT_nom, uT1_nom, _Omg_nom, _Euler_rate_des, h_Omega,
                            _F_nom, _psi_nom, _Euler_des_prev, dt);
        
        // nominal angular acceleration
        math::Vector<3> ang_a; ang_a.zero();
		if (_entered_trajectory_feedback_controller){
			// can trust _Omg_nom & _Euler_rate_des from orientation mapping
			ang_a = (_Omg_nom - _om_des_prev)/dt;
		} else {
			_entered_trajectory_feedback_controller = true;
			// TODO: update next vars
			math::Vector<3> pos_nom_next = _pos_nom;
			math::Vector<3> vel_nom_next = _vel_nom;
			math::Vector<3> acc_nom_next = _acc_nom;
			float psi_nom_next = _psi_nom;
			feedback_fwd(pos_nom_next,vel_nom_next,acc_nom_next,psi_nom_next,dt,_Euler_rate_des,_Omg_nom);
			ang_a.zero();
		}

        // nominal moment input
        _M_nom = _J_B*ang_a + cross(_Omg_nom, _J_B*_Omg_nom);

        // Update previous
        _om_des_prev = _Omg_nom;
        _Euler_des_prev = _Euler_des;

    } else {
        
        _pos_nom(0) = poly_eval(_x_coefs.at(_x_coefs.size()-1), poly_term_t);
        _pos_nom(1) = poly_eval(_y_coefs.at(_y_coefs.size()-1), poly_term_t);
        _pos_nom(2) = poly_eval(_z_coefs.at(_z_coefs.size()-1), poly_term_t);
        _psi_nom = poly_eval(_yaw_coefs.at(_yaw_coefs.size()-1), poly_term_t);
        
        _reset_pos_nom = false;
        _reset_alt_nom = false;
        _reset_psi_nom = false;
        
        hold_position(dt);
    }
    
}

/* transition position and velocity setpoints smoothly */
void
MulticopterTrajectoryControl::trajectory_transition_pos_vel(float t_trans,
	const std::vector<float>& x_trans_coefs, const std::vector<float>& y_trans_coefs, const std::vector<float>& z_trans_coefs,
	const std::vector<float>& vx_trans_coefs, const std::vector<float>& vy_trans_coefs, const std::vector<float>& vz_trans_coefs,
	const float dt)
{
	// local variables
	math::Vector<3> x_nom;	x_nom.zero();	/**< nominal body x-axis */
	math::Vector<3> y_nom;	y_nom.zero();	/**< nominal body x-axis */
	math::Vector<3> z_nom;	z_nom.zero();	/**< nominal body x-axis */
	float uT1_nom = 0.0f;	/**< 1st deriv of nominal thrust input */
	math::Vector<3> h_Omega; 	h_Omega.zero();
	math::Vector<3> z_W;	z_W.zero();
	z_W(2) = 1.0f;
	
	// calculate setpoint in transition
	_pos_nom(0) = poly_eval(x_trans_coefs, t_trans);
    _pos_nom(1) = poly_eval(y_trans_coefs, t_trans);
	_pos_nom(2) = poly_eval(z_trans_coefs, t_trans);
	_psi_nom = _yaw_coefs.at(0).at(0);
	
	_vel_nom(0) = poly_eval(vx_trans_coefs, t_trans);
	_vel_nom(1) = poly_eval(vy_trans_coefs, t_trans);
	_vel_nom(2) = poly_eval(vz_trans_coefs, t_trans);
	
    
	// nominal force in world frame (eqn 16)
	_acc_nom.zero();
	math::Vector<3> ex = _pos - _pos_nom;
	math::Vector<3> ev = _vel - _vel_nom;

	_F_nom = _acc_nom*_mass - z_W*(_mass*GRAV) - ex*_kx - ev*_kv;

	if (_F_nom(2) > 0.0f || _F_nom.length()/_mass < _safe_params.freefall_thresh) {
		// stabilize min thrust "free fall"
	    _F_nom.zero();
	    _F_nom(2) = -_safe_params.thrust_min;
	 }

	// nominal thrust, orientation, and angular velocity
	force_orientation_mapping(_R_N2W,_Euler_des, x_nom, y_nom, z_nom,
						_uT_nom, uT1_nom, _Omg_nom, _Euler_rate_des, h_Omega,
						_F_nom, _psi_nom, _Euler_des_prev, dt);

	// nominal angular acceleration
	math::Vector<3> ang_a; ang_a.zero();
	if (_entered_trajectory_feedback_controller){
		// can trust _Omg_nom & _Euler_rate_des from orientation mapping
		ang_a = (_Omg_nom - _om_des_prev)/dt;
	} else {
		_entered_trajectory_feedback_controller = true;
		// TODO: update next vars
		math::Vector<3> pos_nom_next = _pos_nom;
		math::Vector<3> vel_nom_next = _vel_nom;
		math::Vector<3> acc_nom_next = _acc_nom;
		float psi_nom_next = _psi_nom;
		feedback_fwd(pos_nom_next,vel_nom_next,acc_nom_next,psi_nom_next,dt,_Euler_rate_des,_Omg_nom);
		ang_a.zero();
	}

	// nominal moment input
	_M_nom = _J_B*ang_a + cross(_Omg_nom, _J_B*_Omg_nom);

	// Update previous
	_om_des_prev = _Omg_nom;
	_Euler_des_prev = _Euler_des;
        
}

math::Matrix<3, 3>
MulticopterTrajectoryControl::R_om(math::Vector<3> x) {
    float a_arr[9] = { 1, 0, (float)(-sin(x(1))), 0, (float)(cos(x(0))), (float)(sin(x(0))*cos(x(1))), 0, (float)(-sin(x(0))), (float)(cos(x(0))*cos(x(1))) };
    math::Matrix<3, 3> a(a_arr);
    return a;
}

float
MulticopterTrajectoryControl::bq_1(math::Vector<3> att) {
    return sin(att(0))*sin(att(2)) + cos(att(0))*sin(att(1))*cos(att(2));
}

float
MulticopterTrajectoryControl::bq_2(math::Vector<3> att) {
    return -sin(att(0))*cos(att(2)) + cos(att(0))*sin(att(1))*sin(att(2));
}

float
MulticopterTrajectoryControl::bq_3(math::Vector<3> att) {
    return cos(att(0))*cos(att(1));
}

math::Matrix<6, 6>
MulticopterTrajectoryControl::phi_d(math::Vector<6> x) {
    math::Vector<3> att_input;
    att_input(0) = x(0);
    att_input(1) = x(1);
    att_input(2) = x(2);
    math::Matrix<3, 3> R_om_att = R_om(att_input);

    float mat_arr[36] = { 1, 0, 0, 0, 0, 0,
                          0, 1, 0, 0, 0, 0,
                          0, 0, 1, 0, 0, 0,
                          0, -x(5) * (float)cos(x(1)), 0, R_om_att(0, 0), R_om_att(0, 1), R_om_att(0, 2),
                         -x(4) * (float)sin(x(0)) + x(5) * (float)cos(x(0)) * (float)cos(x(1)), -x(5) * (float)sin(x(0)) * (float)sin(x(1)), 0, R_om_att(1, 0), R_om_att(1, 1), R_om_att(1, 2),
                         -x(4) * (float)cos(x(0)) - x(5) * (float)sin(x(0)) * (float)cos(x(1)), -x(5) * (float)cos(x(0)) * (float)sin(x(1)), 0, R_om_att(2, 0), R_om_att(2, 1), R_om_att(2, 2)};

    math::Matrix<6, 6> mat(mat_arr);
    return mat;
}

math::Vector<6>
MulticopterTrajectoryControl::f_rot(math::Vector<6> x) {
    float Jx = _J_B(0, 0);
    float Jy = _J_B(1, 1);
    float Jz = _J_B(2, 2);

    float a = (Jy-Jz)/Jx;
    float b = (Jz-Jx)/Jy;
    float c = (Jx-Jy)/Jz;

    float vec_arr[6] = { x(3) + x(4)*(float)tan(x(1))*(float)sin(x(0)) + x(5)*(float)tan(x(1))*(float)cos(x(0)),
                         x(4)*(float)cos(x(0)) - x(5)*(float)sin(x(0)),
                         x(4)/(float)cos((x(1)))*(float)sin(x(0)) + x(5)/(float)cos(x(1))*(float)cos(x(0)),
                         a*x(4)*x(5),
                         b*x(3)*x(5),
                         c*x(3)*x(4) };
    math::Vector<6> vec(vec_arr);
    return vec;
}

math::Matrix<6, 6>
MulticopterTrajectoryControl::M(math::Vector<6> x) {

    math::Matrix<6, 6> temp = phi_d(x).inversed();
    return _Mq * temp;
}

/* Apply feedback control to nominal trajectory */
void
MulticopterTrajectoryControl::trajectory_feedback_controller()
{

    float q_des_arr[6] = { _Euler_des(0), _Euler_des(1), _Euler_des(2), _Euler_rate_des(0), _Euler_rate_des(1), _Euler_rate_des(2) };
    math::Vector<6> q_des(q_des_arr);

    float q_act_arr[6] = { _Euler_angles(0), _Euler_angles(1), _Euler_angles(2), _Euler_rates(0), _Euler_rates(1), _Euler_rates(2) };
    math::Vector<6> q_act(q_act_arr);

    float xi_q_des_arr[6] = { _Euler_des(0), _Euler_des(1), _Euler_des(2), _Omg_nom(0), _Omg_nom(1), _Omg_nom(2) };
    math::Vector<6> xi_q_des(xi_q_des_arr);

    float xi_q_act_arr[6] = { _Euler_angles(0), _Euler_angles(1), _Euler_angles(2), _Omg_body(0), _Omg_body(1), _Omg_body(2) };
    math::Vector<6> xi_q_act(xi_q_act_arr);

    math::Matrix<6, 1> Xq_dot;
    Xq_dot.set_col(0, q_act-q_des);
    math::Matrix<1, 1> E_mat = Xq_dot.transposed() * _Mq * Xq_dot;
    float E = E_mat(0,0);

    math::Matrix<1, 3> A = (Xq_dot * 2.0).transposed() *
                           (M(q_act) * _B_rot);

//    float infinity = std::numeric_limits<float>::infinity();
//    float l_b = -infinity;
    float lambda = 2.5;
    float u_b = E * lambda +
                (Xq_dot.transposed() * (M(q_des) * (f_rot(xi_q_des) + _B_rot * _M_nom)))(0) -     // (0) gets float from math::Vector
                (Xq_dot.transposed() * (M(q_act) * (f_rot(xi_q_act) + _B_rot * _M_nom)))(0);      // (0) gets float from math::Vector

    float a = (-2.0f)*u_b;
    math::Vector<3> b( {A(0, 0), A(0, 1), A(0, 2)} );

    if ((a <= 0.0f) || (b.length() <= 0.001f)) {
        _M_cor.zero();
    } else {
        _M_cor = b * -(a / (A * b)(0));
    }
}

void
MulticopterTrajectoryControl::task_main()
{
    warnx("started");

    _mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
    mavlink_log_info(_mavlink_fd, "[mtc] started");

    /*
     * do subscriptions
     */
    _att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    _control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
    _arming_sub = orb_subscribe(ORB_ID(actuator_armed));
    _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    _traj_spline_sub = orb_subscribe(ORB_ID(trajectory_spline));
    _obs_force_sub = orb_subscribe(ORB_ID(obstacle_repulsive_force_ned));


    /* initialize values of critical structs until first regular update */
    _arming.armed = false;

    /* get an initial update for all sensor and status data */
    poll_subscriptions(hrt_absolute_time());

    bool was_armed = false;
    _control_trajectory_started = false;
    _entered_trajectory_feedback_controller = false;
    _started_feedback = false;

    bool was_flag_control_trajectory_enabled = false;


    /**************** OVERWRITE LATER*****************************/
    /* initialize spline information */  
    typedef std::vector<float>::size_type vecf_sz;
    typedef std::vector< std::vector<float> >::size_type vecf2d_sz;
    
    /* initialize transition trajectory variables */
    float transition_interval_sec = fabs(SPLINE_TRANS_T_SEC_REL);
    float spline_start_delay_sec = fabs(SPLINE_START_DELAY_T_SEC_REL);
    if (transition_interval_sec > spline_start_delay_sec) {
		float temp = transition_interval_sec;
		transition_interval_sec = spline_start_delay_sec;
		spline_start_delay_sec = temp;
	}
	float transition_start_t_sec_abs = 0.0;
	float t_dt_1 = transition_interval_sec;
	float t_dt_2 = t_dt_1*t_dt_1;
	float t_dt_3 = t_dt_2*t_dt_1;
	std::vector<float> x_trans_coefs (4, 0.0f);
	std::vector<float> y_trans_coefs (4, 0.0f);
	std::vector<float> z_trans_coefs (4, 0.0f);
	std::vector<float> vx_trans_coefs (3, 0.0f);
	std::vector<float> vy_trans_coefs (3, 0.0f);
	std::vector<float> vz_trans_coefs (3, 0.0f);
    
    /** TODO: change later with m and J estimators */
    _mass = MASS_TEMP;
    _J_B(0, 0) = X_INERTIA_EST;
    _J_B(1, 1) = Y_INERTIA_EST;
    _J_B(2, 2) = Z_INERTIA_EST;

    _kx = float(16) * _mass;
    _kv = float(5.6) * _mass;
    float M_q_arr[36] = { 96.5564672720308, -5.87671199263202e-23, -5.87671168055113e-23, 17.8808272728795, 1.46924245242847e-22, 1.46924213453325e-22,
                          -5.87671199263202e-23, 96.5564672720307, -5.87671167418803e-23, 1.46924185101066e-22, 17.8808272728795, 1.46924180540221e-22,
                          -5.87671168055113e-23, -5.87671167418802e-23, 96.5564672720307, 1.46924201295359e-22, 1.46924233877655e-22, 17.8808272728795,
                          17.8808272728795, 1.46924185101066e-22, 1.46924201295359e-22, 7.15233090780381, 5.87696988361298e-23, 5.87696957170912e-23,
                          1.46924245242847e-22, 17.8808272728795, 1.46924233877655e-22, 5.87696988361298e-23, 7.15233090780380, 5.87696956509190e-23,
                          1.46924213453325e-22, 1.46924180540221e-22, 17.8808272728795, 5.87696957170912e-23, 5.87696956509190e-23, 7.15233090780380 };
    _Mq.set(M_q_arr);
    float B_rot_arr[18] = { 0, 0, 0,
                            0, 0, 0,
                            0, 0, 0,
                            1.0f/_J_B(0, 0), 0, 0,
                            0, 1.0f/_J_B(1,1), 0,
                            0, 0, 1.0f/_J_B(2,2)};
    _B_rot.set(B_rot_arr);

    _alpha = THROTTLE_FILTER_SMOOTHING;
    _alpha = (_alpha < 0.0f) ? 0.0f : _alpha;
    _alpha = (_alpha > 1.0f) ? 1.0f : _alpha;
    _k_thr = 1.0f/(TRAJ_PARAMS_THROTTLE_PER_THRUST*_mass);
    _thr_prev = TRAJ_PARAMS_THROTTLE_PER_THRUST*_mass*GRAV;
    
    _safe_params.thrust_min = TRAJ_PARAMS_VERT_ACC_MIN*_mass ;
    _safe_params.thrust_max = TRAJ_PARAMS_VERT_ACC_MAX*_mass ;

    /* wakeup source */
    struct pollfd fds[1];

    fds[0].fd = _local_pos_sub;
    fds[0].events = POLLIN;
    

    while (!_task_should_exit) {
        
        /* wait for up to 500ms for data */
        int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

        /* timed out - periodic check for _task_should_exit */
        if (pret == 0) {
            continue;
        }

        /* this is undesirable but not much we can do */
        if (pret < 0) {
            warn("poll error %d, %d", pret, errno);
            continue;
        }

		/* timing variables */
        hrt_abstime t = hrt_absolute_time();
        float t_sec = ((float)t)*_MICRO_;
        static uint64_t last_run = 0;
		float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
		last_run = hrt_absolute_time();

		/* poll uORB topics */
		poll_subscriptions(t);

		/* guard against too small (< 2ms) and too large (> 20ms) dt's */
		if (dt < 0.002f) {
			dt = 0.002f;
		} else if (dt > 0.02f) {
			dt = 0.02f;
		}

        if (_control_mode.flag_armed && !was_armed) {
            /* reset setpoints, integrals and trajectory on arming */

            reset_trajectory();
        }

        was_armed = _control_mode.flag_armed;

        update_ref();
        
        /* reset trajectory start time */
        if (!_control_mode.flag_control_trajectory_enabled &&
            _control_trajectory_started){
        
            _control_trajectory_started = false;
        }
        
        /* reset trajectory data when switched out of traj control */
        if (!_control_mode.flag_control_trajectory_enabled &&
            was_flag_control_trajectory_enabled){
    
            reset_trajectory();
        }
        

        if (_control_mode.flag_control_trajectory_enabled) {

            _pos(0) = _local_pos.x;
            _pos(1) = _local_pos.y;
            _pos(2) = _local_pos.z;

            _vel(0) = _local_pos.vx;
            _vel(1) = _local_pos.vy;
            _vel(2) = _local_pos.vz;
                
                
            // Check that trajectory data is available
            if (_traj_spline.segArr[0].nSeg > 0) {
                
                // in case of interupted trajectory, make sure to 
                // hold at current position instead of going
                _reset_alt_nom = true;
                _reset_pos_nom = true;
                _reset_psi_nom = true;
            
                // Initial computations at start of trajectory
                if (!_control_trajectory_started) {
                    
                    _control_trajectory_started = true;
                    
                    
                    /* Swap existing trajectory data into _prev container */
                    _spline_delt_sec.swap(_prev_spline_delt_sec);
                    _spline_cumt_sec.swap(_prev_spline_cumt_sec);
                    _prev_spline_start_t_sec_abs = _spline_start_t_sec_abs;
                    _prev_spline_term_t_sec_rel = _spline_term_t_sec_rel;
					_x_coefs.swap(_prev_x_coefs);
					_xv_coefs.swap(_prev_xv_coefs);
					_xa_coefs.swap(_prev_xa_coefs);
                    _xj_coefs.swap(_prev_xj_coefs); // not used
                    _xs_coefs.swap(_prev_xs_coefs); // not used
					_y_coefs.swap(_prev_y_coefs);
					_yv_coefs.swap(_prev_yv_coefs);
					_ya_coefs.swap(_prev_ya_coefs);
					_yj_coefs.swap(_prev_yj_coefs);
					_ys_coefs.swap(_prev_ys_coefs);
					_z_coefs.swap(_prev_z_coefs);
					_zv_coefs.swap(_prev_zv_coefs);
					_za_coefs.swap(_prev_za_coefs);
					_zj_coefs.swap(_prev_zj_coefs);
					_zs_coefs.swap(_prev_zs_coefs);
					_yaw_coefs.swap(_prev_yaw_coefs);
                    _yaw1_coefs.swap(_prev_yaw1_coefs); // not used
                    _yaw2_coefs.swap(_prev_yaw2_coefs); // not used
                    
                    // number of segments in spline
                    int n_spline_seg = _traj_spline.segArr[0].nSeg;
                    
                    /* Resize vector of appropriate size */
                    // avoid allocation and deallocation as it can be costly
                    _spline_delt_sec.resize(n_spline_seg);
                    _spline_cumt_sec.resize(n_spline_seg);
                    _x_coefs.resize(n_spline_seg);
                    _xv_coefs.resize(n_spline_seg);
                    _xa_coefs.resize(n_spline_seg);
                    _xj_coefs.resize(n_spline_seg);
                    _xs_coefs.resize(n_spline_seg);
                    _y_coefs.resize(n_spline_seg);
                    _yv_coefs.resize(n_spline_seg);
                    _ya_coefs.resize(n_spline_seg);
                    _yj_coefs.resize(n_spline_seg);
                    _ys_coefs.resize(n_spline_seg);
                    _z_coefs.resize(n_spline_seg);
                    _zv_coefs.resize(n_spline_seg);
                    _za_coefs.resize(n_spline_seg);
                    _zj_coefs.resize(n_spline_seg);
                    _zs_coefs.resize(n_spline_seg);
                    _yaw_coefs.resize(n_spline_seg);
                    _yaw1_coefs.resize(n_spline_seg);
                    _yaw2_coefs.resize(n_spline_seg);
                    //~ _spline_delt_sec.resize(n_spline_seg);
                    //~ _spline_delt_sec = std::vector<float> (n_spline_seg, 0.0f);
                    //~ _spline_cumt_sec.resize(n_spline_seg);
                    //~ _spline_cumt_sec = std::vector<float> (n_spline_seg, 0.0f);
                    //~ _x_coefs.resize(n_spline_seg);
                    //~ _x_coefs = std::vector< std::vector<float> > (n_spline_seg, std::vector<float> (N_POLY_COEFS));
                    //~ _y_coefs.resize(n_spline_seg);
                    //~ _y_coefs = std::vector< std::vector<float> > (n_spline_seg, std::vector<float> (N_POLY_COEFS));
                    //~ _z_coefs.resize(n_spline_seg);
                    //~ _z_coefs = std::vector< std::vector<float> > (n_spline_seg, std::vector<float> (N_POLY_COEFS));
                    //~ _yaw_coefs.resize(n_spline_seg);
                    //~ _yaw_coefs = std::vector< std::vector<float> > (n_spline_seg, std::vector<float> (N_POLY_COEFS));
                        
                    
                    // Copy data from trajectory_spline topic
                    for (vecf2d_sz row = 0; row != n_spline_seg; ++row){
                        _spline_delt_sec.at(row) = _traj_spline.segArr[row].Tdel;
                        _x_coefs.at(row).resize(N_POLY_COEFS);
						_y_coefs.at(row).resize(N_POLY_COEFS);
						_z_coefs.at(row).resize(N_POLY_COEFS);
						_yaw_coefs.at(row).resize(N_POLY_COEFS);
                        for (vecf_sz col = 0; col != N_POLY_COEFS; ++col){
                            _x_coefs.at(row).at(col) = _traj_spline.segArr[row].xCoefs[col];
                            _y_coefs.at(row).at(col) = _traj_spline.segArr[row].yCoefs[col];
                            _z_coefs.at(row).at(col) = _traj_spline.segArr[row].zCoefs[col];
                            _yaw_coefs.at(row).at(col) = _traj_spline.segArr[row].yawCoefs[col];
                        }
                    }
                    
                    
                    // Generate cumulative time vector
                    _spline_start_t_sec_abs = t_sec + spline_start_delay_sec;
                    vector_cum_sum(_spline_delt_sec, 0.0f, _spline_cumt_sec);
                    _spline_term_t_sec_rel = _spline_cumt_sec.at(_spline_cumt_sec.size()-1);
                    
                    
                    // Calculate derivative coefficients
                    poly_deriv(_x_coefs, _xv_coefs);
                    poly_deriv(_xv_coefs, _xa_coefs);
                    poly_deriv(_xa_coefs, _xj_coefs);
                    poly_deriv(_xj_coefs, _xs_coefs);
                    poly_deriv(_y_coefs, _yv_coefs);
                    poly_deriv(_yv_coefs, _ya_coefs);
                    poly_deriv(_ya_coefs, _yj_coefs);
                    poly_deriv(_yj_coefs, _ys_coefs);
                    poly_deriv(_z_coefs, _zv_coefs);
                    poly_deriv(_zv_coefs, _za_coefs);
                    poly_deriv(_za_coefs, _zj_coefs);
                    poly_deriv(_zj_coefs, _zs_coefs);
                    poly_deriv(_yaw_coefs, _yaw1_coefs);
                    poly_deriv(_yaw1_coefs, _yaw2_coefs);
                    
                           
                    // calculate transition polynomial coeffs
                    transition_start_t_sec_abs = t_sec;
                    float x_i = _pos(0);
                    float y_i = _pos(1);
                    float z_i = _pos(2);
                    float vx_i = _vel(0);
                    float vy_i = _vel(1);
                    float vz_i = _vel(2);
                    float x_f = _x_coefs.at(0).at(0);
                    float y_f = _y_coefs.at(0).at(0);
                    float z_f = _z_coefs.at(0).at(0);
                    float vx_f = _x_coefs.at(0).at(1);
                    float vy_f = _y_coefs.at(0).at(1);
                    float vz_f = _z_coefs.at(0).at(1);
                    x_trans_coefs.at(0) = x_i;
                    y_trans_coefs.at(0) = y_i;
                    z_trans_coefs.at(0) = z_i;
                    x_trans_coefs.at(1) = vx_i;
                    y_trans_coefs.at(1) = vy_i;
                    z_trans_coefs.at(1) = vz_i;
                    float val1 = x_f - x_i - vx_i*t_dt_1;
                    float val2 = t_dt_1*(vx_f - vx_i);
                    x_trans_coefs.at(2) = (3.0f*val1 - val2)/t_dt_2;
                    x_trans_coefs.at(3) = (-2.0f*val1 + val2)/t_dt_3;
                    val1 = y_f - y_i - vy_i*t_dt_1;
                    val2 = t_dt_1*(vy_f - vy_i);
                    y_trans_coefs.at(2) = (3.0f*val1 - val2)/t_dt_2;
                    y_trans_coefs.at(3) = (-2.0f*val1 + val2)/t_dt_3;
                    val1 = z_f - z_i - vz_i*t_dt_1;
                    val2 = t_dt_1*(vz_f - vz_i);
                    z_trans_coefs.at(2) = (3.0f*val1 - val2)/t_dt_2;
                    z_trans_coefs.at(3) = (-2.0f*val1 + val2)/t_dt_3;
                    poly_deriv(x_trans_coefs, vx_trans_coefs);
                    poly_deriv(y_trans_coefs, vy_trans_coefs);
                    poly_deriv(z_trans_coefs, vz_trans_coefs);
                    
                }
                
                float cur_transition_t = t_sec - transition_start_t_sec_abs;
                if (cur_transition_t < transition_interval_sec && transition_interval_sec > _MICRO_) { 
					
					trajectory_transition_pos_vel (cur_transition_t,
						x_trans_coefs, y_trans_coefs, z_trans_coefs,
						vx_trans_coefs, vy_trans_coefs, vz_trans_coefs,dt);
					
				} else {
					
					/* Calculate timing parameters */
					// determine time in spline trajectory
					float cur_spline_t = t_sec - _spline_start_t_sec_abs;
					//~ float prev_spline_t = t_sec - _prev_spline_start_t_sec_abs;
					
					// determine polynomial segment being evaluated
					std::vector<float>::iterator seg_it;
					seg_it = std::lower_bound(_spline_cumt_sec.begin(), 
						_spline_cumt_sec.end(), cur_spline_t);
					int cur_seg = (int)(seg_it - _spline_cumt_sec.begin());
					
					// determine time in polynomial segment
					float cur_poly_t = cur_seg == 0 ? cur_spline_t :
								cur_spline_t - _spline_cumt_sec.at(cur_seg-1);
					float poly_term_t = cur_seg == 0 ? 0.0f : _spline_delt_sec.at(cur_seg-1);
					
					trajectory_nominal_state(cur_spline_t, _spline_term_t_sec_rel, cur_seg, cur_poly_t, poly_term_t,dt);
				
				}
            
            } else {
                // perform position hold
                hold_position(dt);
                
            }
                  
            
            /**
             * Apply feedback control to nominal trajectory
             */
            trajectory_feedback_controller();

            _uT_sp = _uT_nom; // fbck and ffwd done

            _M_sp = _M_nom + _M_cor;
            _att_control = _M_sp;
            
            /**
             * Apply filter to map thrust to throttle and apply safety
             * NOTE: assume quadrotor is in air
             */
            float zw_dot_zb = _R_B2W(2,2);	// R_W2B*z_W dot z_B = R_B2W'*z_W dot z_B = R_B2W'(:,2) dot [0,0,1]' = R_B2W(2,2)' = R_B2W(2,2)
            float k_measured = (1.0f/_thr_prev)*(GRAV*zw_dot_zb - _sensor.accelerometer_m_s2[2]);
            _k_thr = _alpha*k_measured + (1.0f - _alpha)*_k_thr;
            float throttle = _uT_sp/(_k_thr*_mass);
            //~ float throttle = _uT_sp*TRAJ_PARAMS_THROTTLE_PER_THRUST;
            throttle = (throttle > TRAJ_PARAMS_THROTTLE_MAX) ? TRAJ_PARAMS_THROTTLE_MAX : throttle;
            throttle = (throttle < TRAJ_PARAMS_THROTTLE_MIN) ? TRAJ_PARAMS_THROTTLE_MIN : throttle;
            _thr_prev = throttle;
             
            /**
             * Publish topics
             */
             
            /* fill nominal trajectory values */
            _traj_nom.timestamp = hrt_absolute_time();
            _traj_nom.x = _pos_nom(0);
            _traj_nom.y = _pos_nom(1);
            _traj_nom.z = _pos_nom(2);
            _traj_nom.vx = _vel_nom(0);
            _traj_nom.vy = _vel_nom(1);
            _traj_nom.vz = _vel_nom(2);
            _traj_nom.p = _Omg_nom(0);
            _traj_nom.q = _Omg_nom(1);
            _traj_nom.r = _Omg_nom(2);
            math::Vector<3> eul_nom = _R_N2W.to_euler();
            _traj_nom.phi = eul_nom(0);
            _traj_nom.theta = eul_nom(1);
            _traj_nom.psi = _psi_nom;
            _traj_nom.thrust = _uT_nom;
            _traj_nom.Mx = _M_nom(0);
            _traj_nom.My = _M_nom(1);
            _traj_nom.Mz = _M_nom(2);
             
            /* publish nominal trajectory values */
            if (_traj_nom_pub > 0) {
                orb_publish(ORB_ID(trajectory_nominal_values), _traj_nom_pub, &_traj_nom);

            } else {
                _traj_nom_pub = orb_advertise(ORB_ID(trajectory_nominal_values), &_traj_nom);
            }
             
            /* fill local position setpoint */
            _local_pos_nom.timestamp = hrt_absolute_time();
            _local_pos_nom.x = _pos_nom(0);
            _local_pos_nom.y = _pos_nom(1);
            _local_pos_nom.z = _pos_nom(2);
            _local_pos_nom.yaw = _psi_nom;

            /* publish local position setpoint */
            if (_local_pos_nom_pub > 0) {
                orb_publish(ORB_ID(vehicle_local_position_setpoint), _local_pos_nom_pub, &_local_pos_nom);

            } else {
                _local_pos_nom_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_local_pos_nom);
            }
            
            _global_vel_sp.vx = _vel_nom(0);
            _global_vel_sp.vy = _vel_nom(1);
            _global_vel_sp.vz = _vel_nom(2);

            /* publish velocity setpoint */
            if (_global_vel_sp_pub > 0) {
                orb_publish(ORB_ID(vehicle_global_velocity_setpoint), _global_vel_sp_pub, &_global_vel_sp);

            } else {
                _global_vel_sp_pub = orb_advertise(ORB_ID(vehicle_global_velocity_setpoint), &_global_vel_sp);
            }
            
            /* add trim bias values (empirical determined) */
            _att_control(0) += ATTC_ROLL_BIAS;
            _att_control(1) += ATTC_PITCH_BIAS;
            _att_control(2) += ATTC_YAW_BIAS;
             

            /* publish actuator controls */
            _actuators.control[0] = (isfinite(_att_control(0))) ? _att_control(0) : 0.0f;
            _actuators.control[1] = (isfinite(_att_control(1))) ? _att_control(1) : 0.0f;
            _actuators.control[2] = (isfinite(_att_control(2))) ? _att_control(2) : 0.0f;
            _actuators.control[3] = (isfinite(throttle)) ? throttle : 0.0f;
            _actuators.timestamp = hrt_absolute_time();

            if (_control_mode.flag_control_trajectory_enabled) {
                if (_actuators_0_pub > 0) {
                    orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);

                } else {
                    _actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
                }
            }


        } else {
            /* trajectory controller disabled, reset setpoints */
            _reset_alt_nom = true;
            _reset_pos_nom = true;
            _reset_psi_nom = true;
        }
        
        /* record state of trajectory control mode for next iteration */
        was_flag_control_trajectory_enabled = _control_mode.flag_control_trajectory_enabled;
    }

    warnx("stopped");
    mavlink_log_info(_mavlink_fd, "[mtc] stopped");

    _control_task = -1;
    _exit(0);
}

int
MulticopterTrajectoryControl::start()
{
    ASSERT(_control_task == -1);

    /* start the task */
    _control_task = task_spawn_cmd("mc_traj_control",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_MAX - 5,
                       2000,
                       (main_t)&MulticopterTrajectoryControl::task_main_trampoline,
                       nullptr);

    if (_control_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}

int mc_traj_control_main(int argc, char *argv[])
{
    if (argc < 1) {
        errx(1, "usage: mc_traj_control {start|stop|status}");
    }

    if (!strcmp(argv[1], "start")) {

        if (traj_control::g_control != nullptr) {
            errx(1, "already running");
        }

        traj_control::g_control = new MulticopterTrajectoryControl;

        if (traj_control::g_control == nullptr) {
            errx(1, "alloc failed");
        }

        if (OK != traj_control::g_control->start()) {
            delete traj_control::g_control;
            traj_control::g_control = nullptr;
            err(1, "start failed");
        }

        exit(0);
    }

    if (!strcmp(argv[1], "stop")) {
        if (traj_control::g_control == nullptr) {
            errx(1, "not running");
        }

        delete traj_control::g_control;
        traj_control::g_control = nullptr;
        exit(0);
    }

    if (!strcmp(argv[1], "status")) {
        if (traj_control::g_control) {
            errx(0, "running");

        } else {
            errx(1, "not running");
        }
    }

    warnx("unrecognized command");
    return 1;
}

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
 * Multicopter position controller.
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
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_velocity_feed_forward.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/trajectory_spline.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <mavlink/mavlink_log.h>
#include "mc_traj_control_asl_params.h"
#include <vector>
#include <numeric>  // partial_sum

#define TILT_COS_MAX	0.7f
#define SIGMA			0.000001f
#define MIN_DIST		0.01f

// parameters explicitly for work at ASL
#define ASL_LAB_CENTER_X    1.6283f
#define ASL_LAB_CENTER_Y    2.0630f
#define ASL_LAB_CENTER_Z    -1.5f
#define ASL_LAB_CENTER_YAW  -1.68f

#define SPLINE_START_DELAY 500000
#define N_POLY_COEFS    10


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

	bool		_task_should_exit;		/**< if true, task should exit */
	int		_control_task;			/**< task handle for task */
	int		_mavlink_fd;			/**< mavlink fd */

	int		_att_sub;				/**< vehicle attitude subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_control_mode_sub;		/**< vehicle control mode subscription */
	int		_params_sub;			/**< notification of parameter updates */
	int		_manual_sub;			/**< notification of manual control updates */
	int		_arming_sub;			/**< arming status of outputs */
	int		_local_pos_sub;			/**< vehicle local position */
	int		_pos_sp_triplet_sub;	/**< position setpoint triplet */
	int		_local_pos_sp_sub;		/**< offboard local position setpoint */
	int		_global_vel_sp_sub;		/**< offboard global velocity setpoint */
    int     _traj_spline_sub;       /**< trajectory spline */

	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_local_pos_sp_pub;		/**< vehicle local position setpoint publication */
	orb_advert_t	_global_vel_sp_pub;		/**< vehicle global velocity setpoint publication */
    orb_advert_t    _vel_ff_uorb_pub;       /**< vehicle velocity feed forward publication */

	struct vehicle_attitude_s			_att;			/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s			_control_mode;	/**< vehicle control mode */
	struct actuator_armed_s				_arming;		/**< actuator arming status */
	struct vehicle_local_position_s			_local_pos;		/**< vehicle local position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;	/**< vehicle global position setpoint triplet */
	struct vehicle_local_position_setpoint_s	_local_pos_sp;		/**< vehicle local position setpoint */
	struct vehicle_global_velocity_setpoint_s	_global_vel_sp;	/**< vehicle global velocity setpoint */
    struct vehicle_velocity_feed_forward_s      _vel_ff_uorb;   /**< vehicle velocity feed forward term */
    struct trajectory_spline_s  _traj_spline;   /**< trajectory spline */

	struct {
		param_t thr_min;
		param_t thr_max;
		param_t z_p;
		param_t z_vel_p;
		param_t z_vel_i;
		param_t z_vel_d;
		param_t z_vel_max;
		param_t z_ff;
		param_t xy_p;
		param_t xy_vel_p;
		param_t xy_vel_i;
		param_t xy_vel_d;
		param_t xy_vel_max;
		param_t xy_ff;
		param_t tilt_max_air;
		param_t land_speed;
		param_t tilt_max_land;
	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		float thr_min;
		float thr_max;
		float tilt_max_air;
		float land_speed;
		float tilt_max_land;

		math::Vector<3> pos_p;
		math::Vector<3> vel_p;
		math::Vector<3> vel_i;
		math::Vector<3> vel_d;
		math::Vector<3> vel_ff;
		math::Vector<3> vel_max;
		math::Vector<3> sp_offs_max;
	}		_params;

	struct map_projection_reference_s _ref_pos;
	float _ref_alt;
	hrt_abstime _ref_timestamp;

	bool _reset_pos_sp;
	bool _reset_alt_sp;
    bool _control_trajectory_started;
    
	math::Vector<3> _pos;
	math::Vector<3> _pos_sp;
	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Vector<3> _vel_prev;			/**< velocity on previous step */
	math::Vector<3> _vel_ff;
	math::Vector<3> _sp_move_rate;
    math::Vector<3> _acc_ff;            /**< acceleration of setpoint not including corrective terms - Ross Allen */
    
    int _n_spline_seg;      /** < number of segments in spline (not max number, necassarily) */
    
    // time vectors
    std::vector<float> _spline_delt_sec; // time step sizes for each segment
    std::vector<float> _spline_cumt_sec; // cumulative time markers for each segment

    // initialize vector of appropriate size
    std::vector< std::vector<float> > _x_coefs;
    std::vector< std::vector<float> > _y_coefs;
    std::vector< std::vector<float> > _z_coefs;
    std::vector< std::vector<float> > _yaw_coefs;
    
    std::vector< std::vector<float> > _xv_coefs;
    std::vector< std::vector<float> > _yv_coefs;
    std::vector< std::vector<float> > _zv_coefs;
    
    std::vector< std::vector<float> > _xa_coefs;
    std::vector< std::vector<float> > _ya_coefs;
    std::vector< std::vector<float> > _za_coefs;
    

	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update(bool force);

	/**
	 * Update control outputs
	 */
	void		control_update();

	/**
	 * Check for changes in subscribed topics.
	 */
	void		poll_subscriptions();

	static float	scale_control(float ctl, float end, float dz);

	/**
	 * Update reference for local position projection
	 */
	void		update_ref();
	/**
	 * Reset position setpoint to current position
	 */
	void		reset_pos_sp();

	/**
	 * Reset altitude setpoint to current altitude
	 */
	void		reset_alt_sp();

	/**
	 * Check if position setpoint is too far from current position and adjust it if needed.
	 */
	void		limit_pos_sp_offset();

	/**
	 * Set position setpoint using manual control
	 */
	void		control_manual(float dt);

	/**
	 * Set position setpoint using offboard control
	 */
	void		control_offboard(float dt);

	bool		cross_sphere_line(const math::Vector<3>& sphere_c, float sphere_r,
					const math::Vector<3> line_a, const math::Vector<3> line_b, math::Vector<3>& res);

	/**
     * Set position setpoint using trajectory control - Ross Allen
     */
    void        control_periodic_trajectory(float t, float dt);
    void        control_polynomial_trajectory(float t, float start_t, float dt);
    void        control_spline_trajectory(float t, float start_t);
    void        trajectory_hold();
    void        reset_trajectory();
    
    /**
     * Evaluate polynomials
     * */
    float       poly_eval(const float coefs[N_POLY_COEFS], float t);
    float       poly_eval(const std::vector<float>& coefs, float t);
    void        vector_cum_sum(const std::vector<float>& vec, float initval, std::vector<float>& vecsum);
    void        poly_deriv(const std::vector< std::vector<float> >& poly, std::vector< std::vector<float> >& deriv);
    
    
    /**
	 * Set position setpoint for AUTO
	 */
	void		control_auto(float dt);

	/**
	 * Select between barometric and global (AMSL) altitudes
	 */
	void		select_alt(bool global);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();
};

namespace pos_control
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
	_att_sp_sub(-1),
	_control_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_arming_sub(-1),
	_local_pos_sub(-1),
	_pos_sp_triplet_sub(-1),
	_global_vel_sp_sub(-1),
    _traj_spline_sub(-1),

/* publications */
	_att_sp_pub(-1),
	_local_pos_sp_pub(-1),
	_global_vel_sp_pub(-1),
    _vel_ff_uorb_pub(-1),

	_ref_alt(0.0f),
	_ref_timestamp(0),

	_reset_pos_sp(true),
	_reset_alt_sp(true),
{
	memset(&_att, 0, sizeof(_att));
	memset(&_att_sp, 0, sizeof(_att_sp));
	memset(&_manual, 0, sizeof(_manual));
	memset(&_control_mode, 0, sizeof(_control_mode));
	memset(&_arming, 0, sizeof(_arming));
	memset(&_local_pos, 0, sizeof(_local_pos));
	memset(&_pos_sp_triplet, 0, sizeof(_pos_sp_triplet));
	memset(&_local_pos_sp, 0, sizeof(_local_pos_sp));
	memset(&_global_vel_sp, 0, sizeof(_global_vel_sp));
    memset(&_vel_ff_uorb, 0, sizeof(_vel_ff_uorb));
    memset(&_traj_spline, 0, sizeof(_traj_spline));

	memset(&_ref_pos, 0, sizeof(_ref_pos));

	_params.pos_p.zero();
	_params.vel_p.zero();
	_params.vel_i.zero();
	_params.vel_d.zero();
	_params.vel_max.zero();
	_params.vel_ff.zero();
	_params.sp_offs_max.zero();

	_pos.zero();
	_pos_sp.zero();
	_vel.zero();
	_vel_sp.zero();
	_vel_prev.zero();
    //~ _vel_err_prev.zero();
	_vel_ff.zero();
	_sp_move_rate.zero();
    _acc_ff.zero();

	_params_handles.thr_min		= param_find("MPC_THR_MIN");
	_params_handles.thr_max		= param_find("MPC_THR_MAX");
	_params_handles.z_p			= param_find("MPC_Z_P");
	_params_handles.z_vel_p		= param_find("MPC_Z_VEL_P");
	_params_handles.z_vel_i		= param_find("MPC_Z_VEL_I");
	_params_handles.z_vel_d		= param_find("MPC_Z_VEL_D");
	_params_handles.z_vel_max	= param_find("MPC_Z_VEL_MAX");
	_params_handles.z_ff		= param_find("MPC_Z_FF");
	_params_handles.xy_p		= param_find("MPC_XY_P");
	_params_handles.xy_vel_p	= param_find("MPC_XY_VEL_P");
	_params_handles.xy_vel_i	= param_find("MPC_XY_VEL_I");
	_params_handles.xy_vel_d	= param_find("MPC_XY_VEL_D");
	_params_handles.xy_vel_max	= param_find("MPC_XY_VEL_MAX");
	_params_handles.xy_ff		= param_find("MPC_XY_FF");
	_params_handles.tilt_max_air	= param_find("MPC_TILTMAX_AIR");
	_params_handles.land_speed	= param_find("MPC_LAND_SPEED");
	_params_handles.tilt_max_land	= param_find("MPC_TILTMAX_LND");


	/* fetch initial parameter values */
	parameters_update(true);
}

MulticopterTrajectoryControl::~MulticopterTrajectoryControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
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

	pos_control::g_control = nullptr;
}

int
MulticopterTrajectoryControl::parameters_update(bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(_params_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
	}

	if (updated || force) {
            
        /* Overide certain parameters for position control 
         * Added for simplicity so as not to have to hunt down
         * where these values are set and modified by Ross Allen
         */
         
        if (USE_ASL_PARAMS) { 

            _params.thr_min = ASL_PARAMS_THR_MIN;

            _params.thr_max = ASL_PARAMS_THR_MAX;

            _params.tilt_max_air = ASL_PARAMS_TILTMAX_AIR;
            _params.tilt_max_air = math::radians(_params.tilt_max_air);

            _params.land_speed = ASL_PARAMS_LAND_SPEED;

            _params.tilt_max_land = ASL_PARAMS_TILTMAX_LND;
            _params.tilt_max_land = math::radians(_params.tilt_max_land);

            float v;

            _params.pos_p(0) = ASL_PARAMS_XY_P;
            _params.pos_p(1) = ASL_PARAMS_XY_P;

            _params.pos_p(2) = ASL_PARAMS_Z_P;

            _params.vel_p(0) = ASL_PARAMS_XY_VEL_P;
            _params.vel_p(1) = ASL_PARAMS_XY_VEL_P;

            _params.vel_p(2) = ASL_PARAMS_Z_VEL_P;

            _params.vel_i(0) = ASL_PARAMS_XY_VEL_I;
            _params.vel_i(1) = ASL_PARAMS_XY_VEL_I;

            _params.vel_i(2) = ASL_PARAMS_Z_VEL_I;

            _params.vel_d(0) = ASL_PARAMS_XY_VEL_D;
            _params.vel_d(1) = ASL_PARAMS_XY_VEL_D;

            _params.vel_d(2) = ASL_PARAMS_Z_VEL_D;

            _params.vel_max(0) = ASL_PARAMS_XY_VEL_MAX;
            _params.vel_max(1) = ASL_PARAMS_XY_VEL_MAX;

            _params.vel_max(2) = ASL_PARAMS_Z_VEL_MAX;

            v = math::constrain(ASL_PARAMS_XY_FF, 0.0f, 1.0f);
            _params.vel_ff(0) = v;
            _params.vel_ff(1) = v;

            v = math::constrain(ASL_PARAMS_Z_FF, 0.0f, 1.0f);
            _params.vel_ff(2) = v;

            _params.sp_offs_max = _params.vel_max.edivide(_params.pos_p) * 2.0f;
        
        } else {
            
            param_get(_params_handles.thr_min, &_params.thr_min);
            param_get(_params_handles.thr_max, &_params.thr_max);
            param_get(_params_handles.tilt_max_air, &_params.tilt_max_air);
            _params.tilt_max_air = math::radians(_params.tilt_max_air);
            param_get(_params_handles.land_speed, &_params.land_speed);
            param_get(_params_handles.tilt_max_land, &_params.tilt_max_land);
            _params.tilt_max_land = math::radians(_params.tilt_max_land);

            float v;
            param_get(_params_handles.xy_p, &v);
            _params.pos_p(0) = v;
            _params.pos_p(1) = v;
            
            param_get(_params_handles.z_p, &v);
            _params.pos_p(2) = v;
            
            param_get(_params_handles.xy_vel_p, &v);
            _params.vel_p(0) = v;
            _params.vel_p(1) = v;
            
            param_get(_params_handles.z_vel_p, &v);
            _params.vel_p(2) = v;
            
            param_get(_params_handles.xy_vel_i, &v);
            _params.vel_i(0) = v;
            _params.vel_i(1) = v;

            param_get(_params_handles.z_vel_i, &v);
            _params.vel_i(2) = v;

            param_get(_params_handles.xy_vel_d, &v);
            _params.vel_d(0) = v;
            _params.vel_d(1) = v;

            param_get(_params_handles.z_vel_d, &v);
            _params.vel_d(2) = v;

            param_get(_params_handles.xy_vel_max, &v);
            _params.vel_max(0) = v;
            _params.vel_max(1) = v;

            param_get(_params_handles.z_vel_max, &v);
            _params.vel_max(2) = v;

            param_get(_params_handles.xy_ff, &v);
            v = math::constrain(v, 0.0f, 1.0f);
            _params.vel_ff(0) = v;
            _params.vel_ff(1) = v;

            param_get(_params_handles.z_ff, &v);
            v = math::constrain(v, 0.0f, 1.0f);
            _params.vel_ff(2) = v;

            _params.sp_offs_max = _params.vel_max.edivide(_params.pos_p) * 2.0f;
                
        }
	}

	return OK;
}

void
MulticopterTrajectoryControl::poll_subscriptions()
{
	bool updated;

	orb_check(_att_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);
	}

	orb_check(_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
	}

	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}

	orb_check(_manual_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
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
        printf("DEBUG: trajectory recieved\n");
        orb_copy(ORB_ID(trajectory_spline), _traj_spline_sub, &_traj_spline);
        _control_trajectory_started = false;
    }
    
}

float
MulticopterTrajectoryControl::scale_control(float ctl, float end, float dz)
{
	if (ctl > dz) {
		return (ctl - dz) / (end - dz);

	} else if (ctl < -dz) {
		return (ctl + dz) / (end - dz);

	} else {
		return 0.0f;
	}
}

void
MulticopterTrajectoryControl::task_main_trampoline(int argc, char *argv[])
{
	pos_control::g_control->task_main();
}

void
MulticopterTrajectoryControl::update_ref()
{
	if (_local_pos.ref_timestamp != _ref_timestamp) {
		double lat_sp, lon_sp;
		float alt_sp = 0.0f;

		if (_ref_timestamp != 0) {
			/* calculate current position setpoint in global frame */
			map_projection_reproject(&_ref_pos, _pos_sp(0), _pos_sp(1), &lat_sp, &lon_sp);
			alt_sp = _ref_alt - _pos_sp(2);
		}

		/* update local projection reference */
		map_projection_init(&_ref_pos, _local_pos.ref_lat, _local_pos.ref_lon);
		_ref_alt = _local_pos.ref_alt;

		if (_ref_timestamp != 0) {
			/* reproject position setpoint to new reference */
			map_projection_project(&_ref_pos, lat_sp, lon_sp, &_pos_sp.data[0], &_pos_sp.data[1]);
			_pos_sp(2) = -(alt_sp - _ref_alt);
		}

		_ref_timestamp = _local_pos.ref_timestamp;
	}
}

void
MulticopterTrajectoryControl::reset_pos_sp()
{
	if (_reset_pos_sp) {
		_reset_pos_sp = false;
		/* shift position setpoint to make attitude setpoint continuous */
		_pos_sp(0) = _pos(0) + (_vel(0) - _att_sp.R_body[0][2] * _att_sp.thrust / _params.vel_p(0)
				- _params.vel_ff(0) * _sp_move_rate(0)) / _params.pos_p(0);
		_pos_sp(1) = _pos(1) + (_vel(1) - _att_sp.R_body[1][2] * _att_sp.thrust / _params.vel_p(1)
				- _params.vel_ff(1) * _sp_move_rate(1)) / _params.pos_p(1);
		mavlink_log_info(_mavlink_fd, "[mpc] reset pos sp: %.2f, %.2f", (double)_pos_sp(0), (double)_pos_sp(1));
	}
}

void
MulticopterTrajectoryControl::reset_alt_sp()
{
	if (_reset_alt_sp) {
		_reset_alt_sp = false;
		_pos_sp(2) = _pos(2) + (_vel(2) - _params.vel_ff(2) * _sp_move_rate(2)) / _params.pos_p(2);
		mavlink_log_info(_mavlink_fd, "[mpc] reset alt sp: %.2f", -(double)_pos_sp(2));
	}
}

void
MulticopterTrajectoryControl::limit_pos_sp_offset()
{
	math::Vector<3> pos_sp_offs;
	pos_sp_offs.zero();


	pos_sp_offs(0) = (_pos_sp(0) - _pos(0)) / _params.sp_offs_max(0);
	pos_sp_offs(1) = (_pos_sp(1) - _pos(1)) / _params.sp_offs_max(1);

	pos_sp_offs(2) = (_pos_sp(2) - _pos(2)) / _params.sp_offs_max(2);


	float pos_sp_offs_norm = pos_sp_offs.length();

	if (pos_sp_offs_norm > 1.0f) {
		pos_sp_offs /= pos_sp_offs_norm;
		_pos_sp = _pos + pos_sp_offs.emult(_params.sp_offs_max);
	}
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

/* Added by Ross Allen */
void
MulticopterTrajectoryControl::control_periodic_trajectory(float t, float dt)
{
    //~ /* Just force a static setpoint for now */
    //~ _pos_sp(0) = ASL_LAB_CENTER_X;
	//~ _pos_sp(1) = ASL_LAB_CENTER_Y;
	//~ _pos_sp(2) = ASL_LAB_CENTER_Z;
    //~ _att_sp.yaw_body = ASL_LAB_CENTER_YAW;
    
    float pi_f = (float)M_PI;
    
    /* Simple circular trajectory */
    //~ _pos_sp(0) = 0.5f*(float)cos((double)(2.0f*t/5.0f)*M_PI) + ASL_LAB_CENTER_X;
    //~ _pos_sp(1) = 0.5f*(float)sin((double)(2.0f*t/5.0f)*M_PI) + ASL_LAB_CENTER_Y;
    //~ _pos_sp(2) = 0.5f*(float)cos((double)(2.0f*t/20.0f)*M_PI) + ASL_LAB_CENTER_Z;
    //~ _att_sp.yaw_body = ASL_LAB_CENTER_YAW;
    //~ 
    //~ _vel_ff(0) = 0.5f*(2.0f/5.0f)*((float)M_PI)*(-(float)sin((double)(2.0f*t/5.0f)*M_PI));
    //~ _vel_ff(1) = 0.5f*(2.0f/5.0f)*((float)M_PI)*((float)cos((double)(2.0f*t/5.0f)*M_PI));
    //~ _vel_ff(2) = 0.5f*(2.0f/20.0f)*((float)M_PI)*(-(float)sin((double)(2.0f*t/20.0f)*M_PI));
    //~ _vel_ff = _vel_ff.emult(_params.vel_ff);
    float omega_xy = 2.0f*pi_f/5.0f;
    float omega_z = 2.0f*pi_f/20.0f;
    float amp_xy = 0.5f;
    float amp_z = 0.5f;
    _pos_sp(0) = amp_xy*(float)cos(omega_xy*t) + ASL_LAB_CENTER_X;
    _pos_sp(1) = amp_xy*(float)sin(omega_xy*t) + ASL_LAB_CENTER_Y;
    _pos_sp(2) = amp_xy*(float)cos(omega_z*t) + ASL_LAB_CENTER_Z;
    _att_sp.yaw_body = ASL_LAB_CENTER_YAW;
    
    _vel_ff(0) = amp_xy*omega_xy*(-(float)sin(omega_xy*t));
    _vel_ff(1) = amp_xy*omega_xy*((float)cos(omega_xy*t));
    _vel_ff(2) = amp_z*omega_z*(-(float)sin(omega_z*t));
    //~ _vel_ff = _vel_ff.emult(_params.vel_ff);
    
    _acc_ff(0) = amp_xy*omega_xy*omega_xy*(-(float)cos(omega_xy*t));
    _acc_ff(1) = amp_xy*omega_xy*omega_xy*(-(float)sin(omega_xy*t));
    _acc_ff(2) = amp_z*omega_z*omega_z*(-(float)cos(omega_z*t));
}

/* Added by Ross Allen */
void
MulticopterTrajectoryControl::control_spline_trajectory(float t, float start_t)
{
    
    /* TODO: verify this works for single polynomial segment spline */
    
    // determine time in spline trajectory
    float cur_spline_t = t - start_t;
    float spline_term_t = _spline_cumt_sec.at(_spline_cumt_sec.size()-1);
    
    // determine polynomial segment being evaluated
    std::vector<float>::iterator seg_it;
    seg_it = std::lower_bound(_spline_cumt_sec.begin(), 
        _spline_cumt_sec.end(), cur_spline_t);
    int cur_seg = (int)(seg_it - _spline_cumt_sec.begin());
    
    // determine time in polynomial segment
    float cur_poly_t = cur_seg == 0 ? cur_spline_t :
                cur_spline_t - _spline_cumt_sec.at(cur_seg-1);
    float poly_term_t = cur_seg == 0 ? 0.0f : _spline_delt_sec.at(cur_seg-1);
    
    if (cur_spline_t <= 0) {
        
        _pos_sp(0) = poly_eval(_x_coefs.at(0), 0.0f);
        _pos_sp(1) = poly_eval(_y_coefs.at(0), 0.0f);
        _pos_sp(2) = poly_eval(_z_coefs.at(0), 0.0f);
        _att_sp.yaw_body = poly_eval(_yaw_coefs.at(0), 0.0f);
        
        _vel_ff(0) = 0.0f;
        _vel_ff(1) = 0.0f;
        _vel_ff(2) = 0.0f;
        
        _acc_ff(0) = 0.0f;
        _acc_ff(1) = 0.0f;
        _acc_ff(2) = 0.0f;
        
    } else if (cur_spline_t > 0 && cur_spline_t < spline_term_t) {
    
        _pos_sp(0) = poly_eval(_x_coefs.at(cur_seg), cur_poly_t);
        _pos_sp(1) = poly_eval(_y_coefs.at(cur_seg), cur_poly_t);
        _pos_sp(2) = poly_eval(_z_coefs.at(cur_seg), cur_poly_t);
        _att_sp.yaw_body = poly_eval(_yaw_coefs.at(cur_seg), cur_poly_t);
        
        _vel_ff(0) = poly_eval(_xv_coefs.at(cur_seg), cur_poly_t);
        _vel_ff(1) = poly_eval(_yv_coefs.at(cur_seg), cur_poly_t);
        _vel_ff(2) = poly_eval(_zv_coefs.at(cur_seg), cur_poly_t);
        
        _acc_ff(0) = poly_eval(_xa_coefs.at(cur_seg), cur_poly_t);
        _acc_ff(1) = poly_eval(_ya_coefs.at(cur_seg), cur_poly_t);
        _acc_ff(2) = poly_eval(_za_coefs.at(cur_seg), cur_poly_t);
    
    } else {
        
        _pos_sp(0) = poly_eval(_x_coefs.at(_x_coefs.size()-1), poly_term_t);
        _pos_sp(1) = poly_eval(_y_coefs.at(_y_coefs.size()-1), poly_term_t);
        _pos_sp(2) = poly_eval(_z_coefs.at(_z_coefs.size()-1), poly_term_t);
        _att_sp.yaw_body = poly_eval(_yaw_coefs.at(_yaw_coefs.size()-1), poly_term_t);
        
        _vel_ff(0) = 0.0f;
        _vel_ff(1) = 0.0f;
        _vel_ff(2) = 0.0f;
        
        _acc_ff(0) = 0.0f;
        _acc_ff(1) = 0.0f;
        _acc_ff(2) = 0.0f;
    }
    
}

/* Added by Ross Allen */
void
MulticopterTrajectoryControl::trajectory_hold()
{
        
    reset_alt_sp();
    reset_pos_sp();
    
    _vel_ff(0) = 0.0f;
    _vel_ff(1) = 0.0f;
    _vel_ff(2) = 0.0f;
    
    _acc_ff(0) = 0.0f;
    _acc_ff(1) = 0.0f;
    _acc_ff(2) = 0.0f;
        
}

/* Added by Ross Allen */
void
MulticopterTrajectoryControl::reset_trajectory()
{
        
    memset(&_traj_spline, 0, sizeof(_traj_spline));
    _control_trajectory_started = false;
    reset_alt_sp();
    reset_pos_sp();
        
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


void
MulticopterTrajectoryControl::task_main()
{
	warnx("started");

	_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(_mavlink_fd, "[mpc] started");

	/*
	 * do subscriptions
	 */
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_arming_sub = orb_subscribe(ORB_ID(actuator_armed));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	_global_vel_sp_sub = orb_subscribe(ORB_ID(vehicle_global_velocity_setpoint));
    _traj_spline_sub = orb_subscribe(ORB_ID(trajectory_spline));


	parameters_update(true);

	/* initialize values of critical structs until first regular update */
	_arming.armed = false;

	/* get an initial update for all sensor and status data */
	poll_subscriptions();

	bool reset_int_z = true;
	bool reset_int_xy = true;
	bool was_armed = false;
    _control_trajectory_started = false;
    bool was_flag_control_trajectory_enabled = false;

	hrt_abstime t_prev = 0;

	math::Vector<3> thrust_int;
	thrust_int.zero();
	math::Matrix<3, 3> R;
	R.identity();
    
    /**************** OVERWRITE LATER*****************************/
    /* initialize spline information */
    
    typedef std::vector<float>::size_type vecf_sz;
    typedef std::vector< std::vector<float> >::size_type vecf2d_sz;
    
    // time vector
    float spline_start_t_sec;


	/* wakeup source */
	struct pollfd fds[1];

	fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;
	
	// DEBUG compiler and linker
	printf("Hello Sky!\n");
	return OK;

	//~ while (!_task_should_exit) {
	while (0) {
		
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

		poll_subscriptions();
		parameters_update(false);

		hrt_abstime t = hrt_absolute_time();
        float t_sec = ((float)t)*0.000001f;
		float dt = t_prev != 0 ? (t - t_prev) * 0.000001f : 0.0f;
		t_prev = t;

		if (_control_mode.flag_armed && !was_armed) {
			/* reset setpoints, integrals and trajectory on arming */
			_reset_pos_sp = true;
			_reset_alt_sp = true;
			reset_int_z = true;
			reset_int_xy = true;
            reset_trajectory();
		}

		was_armed = _control_mode.flag_armed;

		update_ref();
        

		if (_control_mode.flag_control_trajectory_enabled) {

			_pos(0) = _local_pos.x;
			_pos(1) = _local_pos.y;
			_pos(2) = _local_pos.z;

			_vel(0) = _local_pos.vx;
			_vel(1) = _local_pos.vy;
			_vel(2) = _local_pos.vz;

			_vel_ff.zero();
			_sp_move_rate.zero();
            _acc_ff.zero();
                
                
			// Check that trajectory data is available
			if (_traj_spline.segArr[0].nSeg > 0) {
				
				// in case of interupted trajectory, make sure to 
				// hold at current position instead of going
				_reset_alt_sp = true;
				_reset_pos_sp = true;
			
				// Initial computations at start of trajectory
				if (!_control_trajectory_started) {
					
					_control_trajectory_started = true;
					
					// number of segments in spline
					_n_spline_seg = _traj_spline.segArr[0].nSeg;
					
					// initialize vector of appropriate size
					_spline_delt_sec = std::vector<float> (_n_spline_seg, 0.0f); // time step sizes for each segment
					_spline_cumt_sec = std::vector<float> (_n_spline_seg, 0.0f); // cumulative time markers for each segment
					_x_coefs = std::vector< std::vector<float> > (_n_spline_seg, 
						std::vector<float> (N_POLY_COEFS));
					_y_coefs = std::vector< std::vector<float> > (_n_spline_seg, 
						std::vector<float> (N_POLY_COEFS));
					_z_coefs = std::vector< std::vector<float> > (_n_spline_seg, 
						std::vector<float> (N_POLY_COEFS));
					_yaw_coefs = std::vector< std::vector<float> > (_n_spline_seg, 
						std::vector<float> (N_POLY_COEFS));
						
					
					// Copy data from trajectory_spline topic
					for (vecf2d_sz row = 0; row != _n_spline_seg; ++row){
						_spline_delt_sec.at(row) = _traj_spline.segArr[row].Tdel;
						for (vecf_sz col = 0; col != N_POLY_COEFS; ++col){
							_x_coefs.at(row).at(col) = _traj_spline.segArr[row].xCoefs[col];
							_y_coefs.at(row).at(col) = _traj_spline.segArr[row].yCoefs[col];
							_z_coefs.at(row).at(col) = _traj_spline.segArr[row].zCoefs[col];
							_yaw_coefs.at(row).at(col) = _traj_spline.segArr[row].yawCoefs[col];
						}
					}
					
					
					// Generate cumulative time vector
					spline_start_t_sec = ((float)(t + SPLINE_START_DELAY))*0.000001f;
					vector_cum_sum(_spline_delt_sec, 0.0f, _spline_cumt_sec);
					
					
					// Calculate derivative coefficients
					poly_deriv(_x_coefs, _xv_coefs);
					poly_deriv(_xv_coefs, _xa_coefs);
					poly_deriv(_y_coefs, _yv_coefs);
					poly_deriv(_yv_coefs, _ya_coefs);
					poly_deriv(_z_coefs, _zv_coefs);
					poly_deriv(_zv_coefs, _za_coefs);
				}
				
				/* call trajectory controller */
				control_spline_trajectory(t_sec, spline_start_t_sec);
			
			} else {
				// perform position hold
				trajectory_hold();
				
			}

            
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

			/* fill local position setpoint */
			_local_pos_sp.timestamp = hrt_absolute_time();
			_local_pos_sp.x = _pos_sp(0);
			_local_pos_sp.y = _pos_sp(1);
			_local_pos_sp.z = _pos_sp(2);
			_local_pos_sp.yaw = _att_sp.yaw_body;

			/* publish local position setpoint */
			if (_local_pos_sp_pub > 0) {
				orb_publish(ORB_ID(vehicle_local_position_setpoint), _local_pos_sp_pub, &_local_pos_sp);

			} else {
				_local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_local_pos_sp);
			}


			/**
			 * Start of primary control calculations
			 */

			/* run position & altitude controllers */
			math::Vector<3> pos_err = _pos_sp - _pos;

			_vel_sp = pos_err.emult(_params.pos_p) + _vel_ff;
			
			/* publish velocity feed forward term */
			// Added by Ross Allen
			_vel_ff_uorb.vx = _vel_ff(0);
			_vel_ff_uorb.vy = _vel_ff(1);
			_vel_ff_uorb.vz = _vel_ff(2);
			
			if (_vel_ff_uorb_pub > 0) {
				orb_publish(ORB_ID(vehicle_velocity_feed_forward), _vel_ff_uorb_pub, &_vel_ff_uorb);
			} else {
				_vel_ff_uorb_pub = orb_advertise(ORB_ID(vehicle_velocity_feed_forward), &_vel_ff_uorb);
			}


			_global_vel_sp.vx = _vel_sp(0);
			_global_vel_sp.vy = _vel_sp(1);
			_global_vel_sp.vz = _vel_sp(2);

			/* publish velocity setpoint */
			if (_global_vel_sp_pub > 0) {
				orb_publish(ORB_ID(vehicle_global_velocity_setpoint), _global_vel_sp_pub, &_global_vel_sp);

			} else {
				_global_vel_sp_pub = orb_advertise(ORB_ID(vehicle_global_velocity_setpoint), &_global_vel_sp);
			}

			/* reset integrals if needed */
			if (reset_int_z) {
				reset_int_z = false;
				float i = _params.thr_min;

				thrust_int(2) = -i;
			}

			if (reset_int_xy) {
				reset_int_xy = false;
				thrust_int(0) = 0.0f;
				thrust_int(1) = 0.0f;
			}


			/* velocity error */
			math::Vector<3> vel_err = _vel_sp - _vel;

			/* derivative of velocity error */
			math::Vector<3> vel_err_d;
			vel_err_d.zero();
			if (_control_mode.flag_control_trajectory_enabled){
				/* make use of analytical knowledge of trajectory - Added by Ross Allen*/
				vel_err_d  = _acc_ff - (_vel - _vel_prev) / dt + (_vel_ff - _vel).emult(_params.pos_p);
			} else {
				/* do not include setpoint acceleration for modes were you don't know it analytically*/
				vel_err_d = (_sp_move_rate - _vel).emult(_params.pos_p) - (_vel - _vel_prev) / dt;
			}
			//~ math::Vector<3> vel_err_d = (vel_err - _vel_err_prev) / dt;     // Added by Ross Allen
			_vel_prev = _vel;
			//~ _vel_err_prev = vel_err;    // Added by Ross Allen

			/* thrust vector in NED frame */
			math::Vector<3> thrust_sp = vel_err.emult(_params.vel_p) + vel_err_d.emult(_params.vel_d) + thrust_int;

			/* limit thrust vector and check for saturation */
			bool saturation_xy = false;
			bool saturation_z = false;

			/* limit min lift */
			float thr_min = _params.thr_min;


			float tilt_max = _params.tilt_max_air;


			/* limit min lift */
			if (-thrust_sp(2) < thr_min) {
				thrust_sp(2) = -thr_min;
				saturation_z = true;
			}

			/* limit max tilt */
			if (thr_min >= 0.0f && tilt_max < M_PI_F / 2 - 0.05f) {
				/* absolute horizontal thrust */
				float thrust_sp_xy_len = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();

				if (thrust_sp_xy_len > 0.01f) {
					/* max horizontal thrust for given vertical thrust*/
					float thrust_xy_max = -thrust_sp(2) * tanf(tilt_max);

					if (thrust_sp_xy_len > thrust_xy_max) {
						float k = thrust_xy_max / thrust_sp_xy_len;
						thrust_sp(0) *= k;
						thrust_sp(1) *= k;
						saturation_xy = true;
					}
				}
			}


			/* limit max thrust */
			float thrust_abs = thrust_sp.length();

			if (thrust_abs > _params.thr_max) {
				if (thrust_sp(2) < 0.0f) {
					if (-thrust_sp(2) > _params.thr_max) {
						/* thrust Z component is too large, limit it */
						thrust_sp(0) = 0.0f;
						thrust_sp(1) = 0.0f;
						thrust_sp(2) = -_params.thr_max;
						saturation_xy = true;
						saturation_z = true;

					} else {
						/* preserve thrust Z component and lower XY, keeping altitude is more important than position */
						float thrust_xy_max = sqrtf(_params.thr_max * _params.thr_max - thrust_sp(2) * thrust_sp(2));
						float thrust_xy_abs = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();
						float k = thrust_xy_max / thrust_xy_abs;
						thrust_sp(0) *= k;
						thrust_sp(1) *= k;
						saturation_xy = true;
					}

				} else {
					/* Z component is negative, going down, simply limit thrust vector */
					float k = _params.thr_max / thrust_abs;
					thrust_sp *= k;
					saturation_xy = true;
					saturation_z = true;
				}

				thrust_abs = _params.thr_max;
			}

			/* update integrals */
			if (!saturation_xy) {
				thrust_int(0) += vel_err(0) * _params.vel_i(0) * dt;
				thrust_int(1) += vel_err(1) * _params.vel_i(1) * dt;
			}

			if (!saturation_z) {
				thrust_int(2) += vel_err(2) * _params.vel_i(2) * dt;

				/* protection against flipping on ground when landing */
				if (thrust_int(2) > 0.0f) {
					thrust_int(2) = 0.0f;
				}
			}

			/* calculate attitude setpoint from thrust vector */
			/* desired body_z axis = -normalize(thrust_vector) */
			math::Vector<3> body_x;
			math::Vector<3> body_y;
			math::Vector<3> body_z;

			if (thrust_abs > SIGMA) {
				body_z = -thrust_sp / thrust_abs;

			} else {
				/* no thrust, set Z axis to safe value */
				body_z.zero();
				body_z(2) = 1.0f;
			}

			/* vector of desired yaw direction in XY plane, rotated by PI/2 */
			math::Vector<3> y_C(-sinf(_att_sp.yaw_body), cosf(_att_sp.yaw_body), 0.0f);

			if (fabsf(body_z(2)) > SIGMA) {
				/* desired body_x axis, orthogonal to body_z */
				body_x = y_C % body_z;

				/* keep nose to front while inverted upside down */
				if (body_z(2) < 0.0f) {
					body_x = -body_x;
				}

				body_x.normalize();

			} else {
				/* desired thrust is in XY plane, set X downside to construct correct matrix,
				 * but yaw component will not be used actually */
				body_x.zero();
				body_x(2) = 1.0f;
			}

			/* desired body_y axis */
			body_y = body_z % body_x;

			/* fill rotation matrix */
			for (int i = 0; i < 3; i++) {
				R(i, 0) = body_x(i);
				R(i, 1) = body_y(i);
				R(i, 2) = body_z(i);
			}

			/* copy rotation matrix to attitude setpoint topic */
			memcpy(&_att_sp.R_body[0][0], R.data, sizeof(_att_sp.R_body));
			_att_sp.R_valid = true;

			/* calculate euler angles, for logging only, must not be used for control */
			math::Vector<3> euler = R.to_euler();
			_att_sp.roll_body = euler(0);
			_att_sp.pitch_body = euler(1);
			/* yaw already used to construct rot matrix, but actual rotation matrix can have different yaw near singularity */


			_att_sp.thrust = thrust_abs;

			_att_sp.timestamp = hrt_absolute_time();

			/* publish attitude setpoint */
			if (_att_sp_pub > 0) {
				orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_att_sp);

			} else {
				_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
			}


		} else {
			/* trajectory controller disabled, reset setpoints */
			_reset_alt_sp = true;
			_reset_pos_sp = true;
			reset_int_z = true;
			reset_int_xy = true;
		}
        
        /* record state of trajectory control mode for next iteration */
        was_flag_control_trajectory_enabled = _control_mode.flag_control_trajectory_enabled;
	}

	warnx("stopped");
	mavlink_log_info(_mavlink_fd, "[mpc] stopped");

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

		if (pos_control::g_control != nullptr) {
			errx(1, "already running");
		}

		pos_control::g_control = new MulticopterTrajectoryControl;

		if (pos_control::g_control == nullptr) {
			errx(1, "alloc failed");
		}

		if (OK != pos_control::g_control->start()) {
			delete pos_control::g_control;
			pos_control::g_control = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (pos_control::g_control == nullptr) {
			errx(1, "not running");
		}

		delete pos_control::g_control;
		pos_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (pos_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}

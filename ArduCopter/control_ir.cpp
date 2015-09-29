/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "Copter.h"


/*
 * control_althold.pde - init and run calls for althold, flight mode
 */

struct PACKED log_ir_pid {
    uint8_t head1;
    uint8_t head2;
    uint8_t msgid;
    uint64_t time_us;
    float desired_x;
    float desired_y;
    float desired_x_speed;
    float desired_y_speed;
    float x_speed_error;
    float y_speed_error;
    float roll_p_pos;
    float pitch_p_pos;
    float roll_p_speed;
    float pitch_p_speed;
    float roll_i_pos;
    float pitch_i_pos;
    float target_roll;
    float target_pitch;
};

// althold_init - initialise althold controller
bool Copter::ir_init(bool ignore_checks)
{
    printf("Init IR\n");
    
    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    // stop takeoff if running
    takeoff_stop();

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Copter::ir_run()
{
    locationT location;
    ipc->getLocation(location);
    
    AltHoldModeState althold_state;
    float takeoff_climb_rate = 0.0f;

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->control_in, channel_pitch->control_in, target_roll, target_pitch);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->control_in);
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

#if FRAME_CONFIG == HELI_FRAME
    bool takeoff_triggered = (ap.land_complete && (channel_throttle->control_in > get_takeoff_trigger_throttle()) && motors.rotor_runup_complete());
#else
    bool takeoff_triggered = (ap.land_complete && (channel_throttle->control_in > get_takeoff_trigger_throttle()));
#endif

    // Alt Hold State Machine Determination
    if(!ap.auto_armed || !motors.get_interlock()) {
        althold_state = AltHold_Disarmed;
    } else if (takeoff_state.running || takeoff_triggered){
        althold_state = AltHold_Takeoff;
    } else if (ap.land_complete){
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_Disarmed:

#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // Multirotors do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif  // HELI_FRAME
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->control_in)-throttle_average);
        break;

    case AltHold_Takeoff:

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt, false);
        pos_control.add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control.update_z_controller();
        break;

    case AltHold_Landed:

#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        attitude_control.set_throttle_out(get_throttle_pre_takeoff(channel_throttle->control_in),false,g.throttle_filt);
#else   // Multirotors do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(get_throttle_pre_takeoff(channel_throttle->control_in),true,g.throttle_filt);
#endif
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->control_in)-throttle_average);
        break;

    case AltHold_Flying:
        
        
        if (location.num_waypoints >= 1) {
            //printf("iTR: %f, ", target_roll);
            AP_Notify::flags.gps_status = 3;
            float max_angle = 1500;
            float max_speed = 0.100; //0.300
            float p_desired_speed = 0.75;
            float p_pos = 14; // 1.5
            float p_speed = 6; //3.5; // 6.0
            float i_pos = 0.001; // 0.03
            float maxI = 2; // 2
            
            static float roll_i_pos = 0;
            static float pitch_i_pos = 0;
            
            float desired_x = location.desiredx / 100.f;
            float desired_y = location.desiredy / 100.f;
            
            //printf("%f\t%f\n", desired_x, desired_y);
            
            float position_x_error = desired_x - location.x;
            float position_y_error = desired_y - location.y;
            
            // compute proportional roll and pitch
            float desired_x_speed = position_x_error * p_desired_speed;
            float desired_y_speed = position_y_error * p_desired_speed;
            if (desired_x_speed > max_speed) desired_x_speed = max_speed;
            else if (desired_x_speed < -max_speed) desired_x_speed = -max_speed;
            if (desired_y_speed > max_speed) desired_y_speed = max_speed;
            else if (desired_y_speed < -max_speed) desired_y_speed = -max_speed;
            
            float x_speed_error = desired_x_speed - location.velx;
            float y_speed_error = desired_y_speed - location.vely;
            
            float roll_p_speed = x_speed_error * p_speed;
            float pitch_p_speed = -y_speed_error * p_speed;
            
            // Not used
            float roll_p_pos = position_x_error * p_pos;
            float pitch_p_pos = -position_y_error * p_pos;
            
            // compute integrative roll and pitch
            roll_i_pos += position_x_error * i_pos;
            pitch_i_pos += -position_y_error * i_pos;
            
            if (roll_i_pos > maxI) roll_i_pos = maxI;
            else if (roll_i_pos < -maxI) roll_i_pos = -maxI;
            if (pitch_i_pos > maxI) pitch_i_pos = maxI;
            else if (pitch_i_pos < -maxI) pitch_i_pos = -maxI;
            
            // Join proportional and integrative roll and pitch
            float r = roll_p_speed + roll_i_pos + roll_p_pos;
            float p = pitch_p_speed + pitch_i_pos + pitch_p_pos;
            
            // Rotate the orders into the multirotor 
            // reference system using its heading
            float rad = radians(location.heading);
            float sinHdg = sin(rad);
            float cosHdg = cos(rad);
            target_roll += (cosHdg * r + sinHdg * p) * 100;
            target_pitch += (cosHdg * p - sinHdg * r) * 100;
            
            if (target_roll > max_angle)
                target_roll = max_angle;
            else if (target_roll < -max_angle)
                target_roll = -max_angle;
            if (target_pitch > max_angle)
                target_pitch = max_angle;
            else if (target_pitch < -max_angle)
                target_pitch = -max_angle;
            
            /*printf("ms: %d, pxe: %f, dxs: %f, xse: %f, rPPos: %f, rPSpeed: %f, rIPos: %f, r: %f, p: %f, tR: %f,\n",\
                (int)(hal.scheduler->micros64() / 1000), position_x_error, desired_x_speed,\
                x_speed_error, roll_p_pos, roll_p_speed, roll_i_pos, r, p, target_roll);*/
            
            // Log data
            struct log_ir_pid pkt;
            pkt.head1 = HEAD_BYTE1;
            pkt.head2 = HEAD_BYTE2;
            pkt.msgid = LOG_IR_PID_MSG;
            pkt.time_us = hal.scheduler->micros64(),
            pkt.desired_x = desired_x;
            pkt.desired_y = desired_y;
            pkt.desired_x_speed = desired_x_speed;
            pkt.desired_y_speed = desired_y_speed;
            pkt.x_speed_error = x_speed_error;
            pkt.y_speed_error = y_speed_error;
            pkt.roll_p_pos = roll_p_pos;
            pkt.pitch_p_pos = pitch_p_pos;
            pkt.roll_p_speed = roll_p_speed;
            pkt.pitch_p_speed = pitch_p_speed;
            pkt.roll_i_pos = roll_i_pos;
            pkt.pitch_i_pos = pitch_i_pos;
            pkt.target_roll = target_roll;
            pkt.target_pitch = target_pitch;
            DataFlash.WriteBlock(&pkt, sizeof(pkt));
        }    
        
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call throttle controller
        if (sonar_enabled && (sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
            // if sonar is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }

        // call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt, false);
        pos_control.update_z_controller();
        
        break;
    }
}

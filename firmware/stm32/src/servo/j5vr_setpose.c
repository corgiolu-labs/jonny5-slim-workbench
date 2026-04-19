/*
 * j5vr_setpose.c — Traiettorie SETPOSE / SETPOSE_T
 *
 * Implementazione estratta da j5vr_actuation.c.
 * Contiene:
 *   - Profili di moto: j5_profile_rtr3/rtr5/bb/bcb
 *   - Stato interno g_setpose_state (static, privato)
 *   - j5vr_setpose_tick  (tick RT loop, 1 kHz)
 *   - j5vr_go_setpose    (comando posa con vel%)
 *   - j5vr_go_setpose_time (comando posa con durata ms)
 *
 * Dipende da:
 *   - desired_positions[] e step_accumulator[] (extern, definiti in actuation.c)
 *   - g_rt_loop_ticks (extern in core/rt_loop.h)
 *   - servo_get_angle / servo_set_angle (servo/servo_control.h)
 *   - uart_send_unsolicited (uart/uart_control.h)
 *
 * JONNY5-4.0 — Step 3.3 refactor (2026-02-25)
 */

#include "servo/j5vr_setpose.h"
#include "servo/servo_control.h"
#include "core/rt_loop.h"
#include "uart/uart_control.h"
#include <zephyr/sys/printk.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Limiti runtime autorevoli condivisi con MANUAL/HEAD/HYBRID/ASSIST.
 * Definiti in j5vr_actuation.c e aggiornati da SET_JOINT_LIMITS. */
extern float joint_min_deg[SERVO_COUNT];
extern float joint_max_deg[SERVO_COUNT];

/* -----------------------------------------------------------------------
 * Helper: angolo fisico polso (clamp 0-180, arrotondamento)
 * Duplicato static da j5vr_actuation.c (3 righe, nessuno stato).
 * ----------------------------------------------------------------------- */
static uint8_t sp_wrist_physical_angle(float logical_deg)
{
    if (logical_deg < 0.0f)   logical_deg = 0.0f;
    if (logical_deg > 180.0f) logical_deg = 180.0f;
    return (uint8_t)(int)(logical_deg + 0.5f);
}

static float sp_clamp_joint_runtime_deg(int joint, float angle_deg)
{
    float runtime_min = J5SP_SAFETY_MIN_DEG;
    float runtime_max = J5SP_SAFETY_MAX_DEG;

    if (joint >= 0 && joint < SERVO_COUNT)
    {
        runtime_min = joint_min_deg[joint];
        runtime_max = joint_max_deg[joint];

        if (runtime_min < J5SP_SAFETY_MIN_DEG) { runtime_min = J5SP_SAFETY_MIN_DEG; }
        if (runtime_min > J5SP_SAFETY_MAX_DEG) { runtime_min = J5SP_SAFETY_MAX_DEG; }
        if (runtime_max < J5SP_SAFETY_MIN_DEG) { runtime_max = J5SP_SAFETY_MIN_DEG; }
        if (runtime_max > J5SP_SAFETY_MAX_DEG) { runtime_max = J5SP_SAFETY_MAX_DEG; }
        if (runtime_min > runtime_max)
        {
            runtime_min = J5SP_SAFETY_MIN_DEG;
            runtime_max = J5SP_SAFETY_MAX_DEG;
        }
    }

    if (angle_deg < runtime_min) { angle_deg = runtime_min; }
    if (angle_deg > runtime_max) { angle_deg = runtime_max; }
    if (angle_deg < J5SP_SAFETY_MIN_DEG) { angle_deg = J5SP_SAFETY_MIN_DEG; }
    if (angle_deg > J5SP_SAFETY_MAX_DEG) { angle_deg = J5SP_SAFETY_MAX_DEG; }
    return angle_deg;
}

/* -----------------------------------------------------------------------
 * Profili di moto normalizzati: tau ∈ [0,1] → s ∈ [0,1]
 * ----------------------------------------------------------------------- */

/* RTR3 — Hermite cubico S-curve (C¹, derivata zero agli estremi) */
static float j5_profile_rtr3(float tau)
{
    return 3.0f*tau*tau - 2.0f*tau*tau*tau;
}

/* RTR5 — Quintico minimum-jerk (C²) */
static float j5_profile_rtr5(float tau)
{
    float t2 = tau * tau;
    float t3 = t2 * tau;
    float t4 = t3 * tau;
    float t5 = t4 * tau;
    return 10.0f*t3 - 15.0f*t4 + 6.0f*t5;
}

/* BCB — raised-cosine s(τ) = (1-cos(π·τ))/2 (C², nessun punto di giunzione) */
static float j5_profile_bcb(float tau)
{
    if (tau <= 0.0f) return 0.0f;
    if (tau >= 1.0f) return 1.0f;
    return (1.0f - cosf(3.14159265f * tau)) * 0.5f;
}

/* -----------------------------------------------------------------------
 * Stato interno SETPOSE (privato a questo modulo)
 * ----------------------------------------------------------------------- */
typedef struct {
    bool     active;
    uint32_t start_tick;
    uint32_t duration_ticks;
    j5_profile_t profile;
    float    q_start[SERVO_COUNT];
    float    q_target[SERVO_COUNT];
    /* Telemetria esecuzione */
    float    prev_angle[SERVO_COUNT];
    float    prev_vel[SERVO_COUNT];
    float    max_velocity_deg_s;
    float    max_accel_deg_s2;
    /* Opt-in: rilascia i servo digitali (PITCH/ROLL) quando il trajectory
     * finisce. Usato dal comando HOME per evitare surriscaldamento dei due
     * servo più stressati al termine del centraggio. */
    bool     relax_digital_on_finish;
} j5_setpose_state_t;

static j5_setpose_state_t g_setpose_state = { .active = false };

/* -----------------------------------------------------------------------
 * j5vr_setpose_tick — chiamato dal RT loop a ogni ciclo (1 kHz)
 * ----------------------------------------------------------------------- */
bool j5vr_setpose_tick(uint32_t rt_tick)
{
    if (!g_setpose_state.active)
    {
        return false;
    }

    uint32_t dt = rt_tick - g_setpose_state.start_tick;
    bool     finished = false;
    float    tau;
    if (dt >= g_setpose_state.duration_ticks)
    {
        tau      = 1.0f;
        finished = true;
        g_setpose_state.active = false;
    }
    else
    {
        tau = (float)dt / (float)g_setpose_state.duration_ticks;
    }

    float s;
    switch (g_setpose_state.profile)
    {
        case J5_PROFILE_RTR3:  s = j5_profile_rtr3(tau); break;
        case J5_PROFILE_RTR5:  s = j5_profile_rtr5(tau); break;
        case J5_PROFILE_BB:    s = j5_profile_rtr3(tau);  break;
        case J5_PROFILE_BCB:   s = j5_profile_bcb(tau);  break;
        default:               s = j5_profile_rtr3(tau);  break;
    }

    for (int i = 0; i < SERVO_COUNT; i++)
    {
        const float interpolated = g_setpose_state.q_start[i]
                                 + s * (g_setpose_state.q_target[i]
                                        - g_setpose_state.q_start[i]);
        desired_positions[i] = sp_clamp_joint_runtime_deg(i, interpolated);
    }

    /* Applica direttamente la posizione interpolata ai servo, bypassando
     * il velocity cap di apply_desired_positions_to_servos().
     * La pipeline SETPOSE ha il proprio controllo temporale tramite duration_ticks. */
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        float new_pos = sp_clamp_joint_runtime_deg(i, desired_positions[i]);
        desired_positions[i] = new_pos;

        /* Telemetria: derivata su desired_positions[] (float, alta risoluzione) */
        {
            float vel = (new_pos - g_setpose_state.prev_angle[i]) * 1000.0f;
            float acc = (vel     - g_setpose_state.prev_vel[i])   * 1000.0f;

            float abs_vel = vel < 0.0f ? -vel : vel;
            float abs_acc = acc < 0.0f ? -acc : acc;

            if (abs_vel > g_setpose_state.max_velocity_deg_s)
            {
                g_setpose_state.max_velocity_deg_s = abs_vel;
            }
            if (abs_acc > g_setpose_state.max_accel_deg_s2)
            {
                g_setpose_state.max_accel_deg_s2 = abs_acc;
            }

            g_setpose_state.prev_angle[i] = new_pos;
            g_setpose_state.prev_vel[i]   = vel;
        }

        uint8_t new_angle = (uint8_t)roundf(new_pos);
        uint8_t cur_angle = servo_get_angle((servo_joint_t)i);
        uint16_t last_pulse_us = servo_get_last_pulse_us((servo_joint_t)i);

        /* Dopo STOP/SAFE i PWM possono essere spenti (pulse=0) pur con angolo logico
         * già uguale al target. In quel caso va forzata comunque la riapplicazione
         * del comando per riagganciare davvero il giunto. */
        if (new_angle != cur_angle || last_pulse_us == 0U)
        {
            uint16_t send_cmd;

            if (i == SERVO_YAW || i == SERVO_PITCH || i == SERVO_ROLL) {
                send_cmd = sp_wrist_physical_angle((float)new_angle);
            } else {
                send_cmd = new_angle;
            }
            servo_set_angle((servo_joint_t)i, send_cmd);
        }

        /* Reset accumulatore: nessun residuo deve passare alla pipeline VR */
        step_accumulator[i] = 0.0f;
    }

    /* Invio telemetria finale via UART non-solicitato */
    if (finished)
    {
        uint32_t elapsed_ms = dt;
        char msg[72];
        snprintf(msg, sizeof(msg),
                 "SETPOSE_DONE time_ms=%u vel_max=%.1f acc_max=%.1f",
                 (unsigned)elapsed_ms,
                 (double)g_setpose_state.max_velocity_deg_s,
                 (double)g_setpose_state.max_accel_deg_s2);
        uart_send_unsolicited(msg);

        /* Post-completion relax, opt-in. Only HOME sets this flag so SETPOSE /
         * SETPOSE_T / TELEOPPOSE / PARK keep their PWM engaged as before. */
        if (g_setpose_state.relax_digital_on_finish)
        {
            g_setpose_state.relax_digital_on_finish = false;
            servo_relax_digital();
            uart_send_unsolicited("RELAX_DIGITAL pitch roll");
        }
    }

    return true;
}

/* -----------------------------------------------------------------------
 * j5vr_go_setpose — posa assoluta 6-DOF con vel% e profilo di moto
 * ----------------------------------------------------------------------- */
void j5vr_go_setpose(
    uint8_t base_deg,
    uint8_t spalla_deg,
    uint8_t gomito_deg,
    uint8_t yaw_deg,
    uint8_t pitch_deg,
    uint8_t roll_deg,
    uint8_t vel_pct,
    j5_profile_t profile
)
{
    uint8_t vp = vel_pct;
    if (vp == 0U)  { vp = 10U; }
    if (vp > 100U) { vp = 100U; }

    float vel_frac = (float)vp / 100.0f;
    float vel_deg_s = J5SP_VEL_MIN_DEG_S
                      + vel_frac * (J5SP_VEL_MAX_DEG_S - J5SP_VEL_MIN_DEG_S);

    float q_target[SERVO_COUNT];
    q_target[SERVO_BASE]   = sp_clamp_joint_runtime_deg(SERVO_BASE,   (float)base_deg);
    q_target[SERVO_SPALLA] = sp_clamp_joint_runtime_deg(SERVO_SPALLA, (float)spalla_deg);
    q_target[SERVO_GOMITO] = sp_clamp_joint_runtime_deg(SERVO_GOMITO, (float)gomito_deg);
    q_target[SERVO_YAW]    = sp_clamp_joint_runtime_deg(SERVO_YAW,    (float)yaw_deg);
    q_target[SERVO_PITCH]  = sp_clamp_joint_runtime_deg(SERVO_PITCH,  (float)pitch_deg);
    q_target[SERVO_ROLL]   = sp_clamp_joint_runtime_deg(SERVO_ROLL,   (float)roll_deg);

    float q_start[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        q_start[i] = sp_clamp_joint_runtime_deg(i, desired_positions[i]);
    }

    float dq_max = 0.0f;
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        float dq = q_target[i] - q_start[i];
        if (dq < 0.0f) { dq = -dq; }
        if (dq > dq_max) { dq_max = dq; }
    }

    float T_s = (dq_max > 0.1f) ? (dq_max / vel_deg_s) : 0.020f;
    uint32_t dur = (uint32_t)(T_s * 1000.0f + 0.5f);
    if (dur < 20U) { dur = 20U; }

    g_setpose_state.active         = false;
    g_setpose_state.profile        = profile;
    g_setpose_state.duration_ticks = dur;
    g_setpose_state.start_tick     = g_rt_loop_ticks;

    for (int i = 0; i < SERVO_COUNT; i++)
    {
        g_setpose_state.q_start[i]  = q_start[i];
        g_setpose_state.q_target[i] = q_target[i];
    }

    for (int i = 0; i < SERVO_COUNT; i++)
    {
        step_accumulator[i] = 0.0f;
        g_setpose_state.prev_angle[i] = q_start[i];
        g_setpose_state.prev_vel[i]   = 0.0f;
    }
    g_setpose_state.max_velocity_deg_s = 0.0f;
    g_setpose_state.max_accel_deg_s2   = 0.0f;
    /* Clear the opt-in relax flag; callers that want it (HOME) must re-request
     * via j5vr_setpose_request_relax_digital_on_finish() AFTER this call. */
    g_setpose_state.relax_digital_on_finish = false;

    g_setpose_state.active = true;
}

void j5vr_setpose_request_relax_digital_on_finish(void)
{
    /* Must be called strictly AFTER j5vr_go_setpose()/j5vr_go_setpose_time() so
     * the start-of-motion reset doesn't clobber the request. Safe to call even
     * if no setpose is active (the flag will just be cleared at next start). */
    g_setpose_state.relax_digital_on_finish = true;
}

/* -----------------------------------------------------------------------
 * j5vr_go_setpose_time — posa assoluta 6-DOF con durata fissa in ms
 * ----------------------------------------------------------------------- */
void j5vr_go_setpose_time(
    const uint32_t *q_target_deg,
    int             count,
    uint32_t        time_ms,
    j5_profile_t    prof)
{
    if (count != SERVO_COUNT || q_target_deg == NULL) { return; }

    if (time_ms < 20U)    { time_ms = 20U; }
    if (time_ms > 60000U) { time_ms = 60000U; }

    volatile bool *pactive = &g_setpose_state.active;
    *pactive = false;

    g_setpose_state.profile        = prof;
    g_setpose_state.duration_ticks = time_ms;
    g_setpose_state.start_tick     = g_rt_loop_ticks;

    for (int i = 0; i < SERVO_COUNT; i++)
    {
        const float target = sp_clamp_joint_runtime_deg(i, (float)q_target_deg[i]);
        const float start  = sp_clamp_joint_runtime_deg(i, desired_positions[i]);
        g_setpose_state.q_target[i]   = target;
        g_setpose_state.q_start[i]    = start;
        g_setpose_state.prev_angle[i] = start;
        g_setpose_state.prev_vel[i]   = 0.0f;
        step_accumulator[i] = 0.0f;
    }
    g_setpose_state.max_velocity_deg_s = 0.0f;
    g_setpose_state.max_accel_deg_s2   = 0.0f;
    g_setpose_state.relax_digital_on_finish = false;

    *pactive = true;
}

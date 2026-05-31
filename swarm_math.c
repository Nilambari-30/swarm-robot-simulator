/*
 * =============================================================================
 * swarm_math.c  —  Core Vector Math & Boids Physics (C Library)
 * =============================================================================
 * Compiled into a shared library and called from Python via ctypes.
 *
 * Compile (Linux / Mac):
 *   gcc -O2 -shared -fPIC -o swarm_math.so swarm_math.c -lm
 *
 * Compile (Windows with MinGW):
 *   gcc -O2 -shared -o swarm_math.dll swarm_math.c -lm
 * =============================================================================
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ══════════════════════════════════════════════════════════════════════════════
   SECTION 1 — Basic 2-D Vector Utilities
   ══════════════════════════════════════════════════════════════════════════════ */

/* Euclidean length of vector (vx, vy) */
double vec2_len(double vx, double vy) {
    return sqrt(vx * vx + vy * vy);
}

/* Euclidean distance between two points */
double vec2_dist(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1, dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

/* Normalise vector in-place; returns original magnitude */
double vec2_normalize(double *vx, double *vy) {
    double l = sqrt((*vx) * (*vx) + (*vy) * (*vy));
    if (l > 1e-9) { *vx /= l; *vy /= l; }
    return l;
}

/* Clamp vector magnitude to max, in-place */
void vec2_limit(double *vx, double *vy, double max_len) {
    double s = sqrt((*vx) * (*vx) + (*vy) * (*vy));
    if (s > max_len && s > 1e-9) {
        *vx = (*vx) / s * max_len;
        *vy = (*vy) / s * max_len;
    }
}

/* Set vector magnitude to exactly |mag|, preserving direction */
void vec2_set_len(double *vx, double *vy, double mag) {
    double s = sqrt((*vx) * (*vx) + (*vy) * (*vy));
    if (s > 1e-9) {
        *vx = (*vx) / s * mag;
        *vy = (*vy) / s * mag;
    }
}

/* Dot product of two vectors */
double vec2_dot(double ax, double ay, double bx, double by) {
    return ax * bx + ay * by;
}

/* Angle of vector in radians (atan2 wrapper) */
double vec2_angle(double vx, double vy) {
    return atan2(vy, vx);
}

/* ══════════════════════════════════════════════════════════════════════════════
   SECTION 2 — Shared Structs (must match Python ctypes definitions exactly)
   ══════════════════════════════════════════════════════════════════════════════ */

/* One robot agent */
typedef struct {
    double x, y;    /* position            */
    double vx, vy;  /* velocity            */
    int    id;      /* unique identifier   */
    int    ci;      /* colour index 0-2    */
    int    _pad;    /* alignment padding   */
} CRobot;

/* One circular or rectangular obstacle */
typedef struct {
    double x, y;      /* centre position     */
    double radius;    /* bounding radius     */
    double w, h;      /* width/height (rect) */
    int    shape;     /* 0 = circle, 1 = rect */
    int    _pad;
} CObs;

/* Simulation parameters */
typedef struct {
    double sep;      /* separation weight   */
    double ali;      /* alignment weight    */
    double coh;      /* cohesion weight     */
    double avoid_r;  /* obstacle avoid radius */
    double nr;       /* neighbour radius    */
    double speed;    /* target speed        */
    int    sim_w;    /* canvas width        */
    int    sim_h;    /* canvas height       */
    int    r_rad;    /* robot radius        */
    int    _pad;
} CParams;

/* ══════════════════════════════════════════════════════════════════════════════
   SECTION 3 — Boids Physics Step
   ══════════════════════════════════════════════════════════════════════════════
   Implements Craig Reynolds' three boids rules for every agent:
     1. Separation  — steer away from close neighbours
     2. Alignment   — steer toward average heading of neighbours
     3. Cohesion    — steer toward average position of neighbours
   Plus: obstacle avoidance and boundary bouncing.
   ══════════════════════════════════════════════════════════════════════════════ */
void boids_step(CRobot       *robots,  int n_robots,
                const CObs   *obs,     int n_obs,
                const CParams *params,
                const double  *rand_buf)   /* length = n_robots * 2, values in [-0.5, 0.5] */
{
    /* Temporary velocity buffers — avoid modifying robots mid-loop */
    double *nvx = (double *)malloc(n_robots * sizeof(double));
    double *nvy = (double *)malloc(n_robots * sizeof(double));

    /* ── Per-robot force accumulation ── */
    for (int i = 0; i < n_robots; i++) {
        double sep_x=0, sep_y=0;   /* Rule 1: separation  */
        double ali_x=0, ali_y=0;   /* Rule 2: alignment   */
        double coh_x=0, coh_y=0;   /* Rule 3: cohesion    */
        double obs_x=0, obs_y=0;   /* Obstacle avoidance  */
        int    neighbours = 0, sep_count = 0;

        double rx = robots[i].x,  ry = robots[i].y;
        double rvx = robots[i].vx, rvy = robots[i].vy;

        /* ── Rule 1-3: Neighbour scan ──────────────────────────────────── */
        for (int j = 0; j < n_robots; j++) {
            if (robots[j].id == robots[i].id) continue;

            double dx = rx - robots[j].x;
            double dy = ry - robots[j].y;
            double d  = vec2_len(dx, dy);
            if (d < 1e-3) d = 1e-3;

            if (d < params->nr) {
                /* Rule 2 — alignment: accumulate neighbours' velocities */
                ali_x += robots[j].vx;
                ali_y += robots[j].vy;
                /* Rule 3 — cohesion: accumulate neighbours' positions */
                coh_x += robots[j].x;
                coh_y += robots[j].y;
                neighbours++;

                /* Rule 1 — separation: push away from very close neighbours */
                if (d < params->r_rad * 3.5) {
                    double weight = 1.0 / (d + 0.01);
                    sep_x += (dx / d) * weight;
                    sep_y += (dy / d) * weight;
                    sep_count++;
                }
            }
        }

        /* ── Obstacle avoidance force ──────────────────────────────────── */
        for (int k = 0; k < n_obs; k++) {
            double dx2 = rx - obs[k].x;
            double dy2 = ry - obs[k].y;
            double d2  = vec2_len(dx2, dy2);
            if (d2 < 1e-3) d2 = 1e-3;

            /* Soft push — proportional to how close the robot is */
            if (d2 < params->avoid_r) {
                double ratio = (params->avoid_r - d2) / params->avoid_r;
                double force = ratio * ratio * 3.5;
                obs_x += (dx2 / d2) * force;
                obs_y += (dy2 / d2) * force;
            }
            /* Hard push — if robot is almost inside the obstacle */
            if (d2 < obs[k].radius + params->r_rad + 10.0) {
                obs_x += (dx2 / d2) * 8.0;
                obs_y += (dy2 / d2) * 8.0;
            }
        }

        /* ── Combine all forces into new velocity ──────────────────────── */
        double vx = rvx, vy = rvy;

        /* Separation */
        if (sep_count > 0) {
            vx += sep_x * params->sep * 0.12;
            vy += sep_y * params->sep * 0.12;
        }

        /* Alignment + Cohesion */
        if (neighbours > 0) {
            /* Alignment: normalise average velocity and steer toward it */
            double avg_len = vec2_len(ali_x / neighbours, ali_y / neighbours);
            if (avg_len < 1e-3) avg_len = 1e-3;
            vx += (ali_x / neighbours / avg_len) * params->ali * 0.06;
            vy += (ali_y / neighbours / avg_len) * params->ali * 0.06;

            /* Cohesion: steer toward centre of mass of neighbours */
            double cdx = coh_x / neighbours - rx;
            double cdy = coh_y / neighbours - ry;
            double coh_len = vec2_len(cdx, cdy);
            if (coh_len < 1e-3) coh_len = 1e-3;
            vx += (cdx / coh_len) * params->coh * 0.04;
            vy += (cdy / coh_len) * params->coh * 0.04;
        }

        /* Obstacle force + tiny random noise (for natural movement) */
        vx += obs_x * 0.25 + rand_buf[i * 2    ] * 0.15;
        vy += obs_y * 0.25 + rand_buf[i * 2 + 1] * 0.15;

        /* Clamp to max speed, enforce min speed */
        vec2_limit(&vx, &vy, params->speed * 1.5);
        double spd = vec2_len(vx, vy);
        if (spd < params->speed * 0.5) {
            vec2_set_len(&vx, &vy, params->speed * 0.5);
        }

        nvx[i] = vx;
        nvy[i] = vy;
    }

    /* ── Integrate positions + wall bouncing ────────────────────────── */
    double sw = (double)params->sim_w;
    double sh = (double)params->sim_h;
    double rr = (double)params->r_rad;

    for (int i = 0; i < n_robots; i++) {
        double nx = robots[i].x + nvx[i];
        double ny = robots[i].y + nvy[i];
        double vx = nvx[i];
        double vy = nvy[i];

        if (nx < rr)      { nx = rr;      vx =  fabs(vx); }
        if (nx > sw - rr) { nx = sw - rr; vx = -fabs(vx); }
        if (ny < rr)      { ny = rr;      vy =  fabs(vy); }
        if (ny > sh - rr) { ny = sh - rr; vy = -fabs(vy); }

        robots[i].x  = nx;   robots[i].y  = ny;
        robots[i].vx = vx;   robots[i].vy = vy;
    }

    free(nvx);
    free(nvy);
}

/* ══════════════════════════════════════════════════════════════════════════════
   SECTION 4 — Statistics
   ══════════════════════════════════════════════════════════════════════════════ */

/* Compute average speed and cluster count across all robots */
void compute_stats(const CRobot *robots, int n,
                   double nr_thresh,
                   double *out_avg_speed,
                   int    *out_clustered)
{
    double total = 0.0;
    int    cl    = 0;

    for (int i = 0; i < n; i++) {
        total += vec2_len(robots[i].vx, robots[i].vy);

        int near = 0;
        for (int j = 0; j < n; j++) {
            if (robots[j].id == robots[i].id) continue;
            if (vec2_dist(robots[i].x, robots[i].y,
                          robots[j].x, robots[j].y) < nr_thresh * 0.8)
                near++;
        }
        if (near >= 2) cl++;
    }

    *out_avg_speed = (n > 0) ? total / n : 0.0;
    *out_clustered = cl;
}

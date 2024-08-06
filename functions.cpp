// functions for the N-body simulation

#include <header.h>

// Functions for computing x, y, and z accelerations of a body alongside the 4th order Runge-Kutta method

// Accelerations function
void accelerations(body *bodies, int curr_body, int n, double G) {
    bodies[curr_body].ax = 0;
    bodies[curr_body].ay = 0;
    bodies[curr_body].az = 0;

    for (int i = 0; i < n; i++) {
        if (i != curr_body) {
            double dx = bodies[i].x - bodies[curr_body].x;
            double dy = bodies[i].y - bodies[curr_body].y;
            double dz = bodies[i].z - bodies[curr_body].z;
            double r = sqrt(pow(dx,2) + pow(dy,2) + pow(dz,3));
            bodies[curr_body].ax += G * bodies[i].mass * dx / pow(r,3);
            bodies[curr_body].ay += G * bodies[i].mass * dy / pow(r,3);
            bodies[curr_body].az += G * bodies[i].mass * dz / pow(r,3);
        }
    }
}

// Runge-Kutta function
void RK4(body *bodies, int n, double t, double dt, double G) {
    // Initialising coefficients for 4th order Runge Kutta Scheme
    long double k1x, k1y, k1z, k1vx, k1vy, k1vz;
    long double k2x, k2y, k2z, k2vx, k2vy, k2vz;
    long double k3x, k3y, k3z, k3vx, k3vy, k3vz;
    long double k4x, k4y, k4z, k4vx, k4vy, k4vz;

    // Cycle through all bodies
    for (int j = 0; j < n; j++) {
        // Compute acceleration for each body
        accelerations(bodies, j, n, G);

        // Compute k1 coefficients
        k1x = dt * bodies[j].vx;
        k1y = dt * bodies[j].vy;
        k1z = dt * bodies[j].vz;
        k1vx = dt * bodies[j].ax;
        k1vy = dt * bodies[j].ay;
        k1vz = dt * bodies[j].az;

        // Update positions and velocities
        bodies[j].x += 0.5 * k1x;
        bodies[j].y += 0.5 * k1y;
        bodies[j].z += 0.5 * k1z;
        bodies[j].vx += 0.5 * k1vx;
        bodies[j].vy += 0.5 * k1vy;
        bodies[j].vz += 0.5 * k1vz;

        // Recompute accelerations
        accelerations(bodies, j, n, G);

        // Compute k2 coefficients
        k2x = dt * bodies[j].vx;
        k2y = dt * bodies[j].vy;
        k2z = dt * bodies[j].vz;
        k2vx = dt * bodies[j].ax;
        k2vy = dt * bodies[j].ay;
        k2vz = dt * bodies[j].az;

        // Update positions and velocities
        bodies[j].x += 0.5 * k2x - 0.5 * k1x;
        bodies[j].y += 0.5 * k2y - 0.5 * k1y;
        bodies[j].z += 0.5 * k2z - 0.5 * k1z;
        bodies[j].vx += 0.5 * k2vx - 0.5 * k1vx;
        bodies[j].vy += 0.5 * k2vy - 0.5 * k1vy;
        bodies[j].vz += 0.5 * k2vz - 0.5 * k1vz;

        // Recompute accelerations
        accelerations(bodies, j, n, G);

        // Compute k3 coefficients
        k3x = dt * bodies[j].vx;
        k3y = dt * bodies[j].vy;
        k3z = dt * bodies[j].vz;
        k3vx = dt * bodies[j].ax;
        k3vy = dt * bodies[j].ay;
        k3vz = dt * bodies[j].az;

        // Update positions and velocities
        bodies[j].x += k3x - 0.5*k2x;
        bodies[j].y += k3y - 0.5*k2y;
        bodies[j].z += k3z - 0.5*k2z;
        bodies[j].vx += k3vx - 0.5*k2vx;
        bodies[j].vy += k3vy - 0.5*k2vy;
        bodies[j].vz += k3vz - 0.5*k2vz;

        // Recompute accelerations
        accelerations(bodies, j, n, G);

        // Compute k4 coefficients
        k4x = dt * bodies[j].vx;
        k4y = dt * bodies[j].vy;
        k4z = dt * bodies[j].vz;
        k4vx = dt * bodies[j].ax;
        k4vy = dt * bodies[j].ay;
        k4vz = dt * bodies[j].az;

        // Update positions and velocities
        bodies[j].x += (k1x + 2*k2x + 2*k3x + k4x) / 6;
        bodies[j].y += (k1y + 2*k2y + 2*k3y + k4y) / 6;
        bodies[j].z += (k1z + 2*k2z + 2*k3z + k4z) / 6;
        bodies[j].vx += (k1vx + 2*k2vx + 2*k3vx + k4vx) / 6;
        bodies[j].vy += (k1vy + 2*k2vy + 2*k3vy + k4vy) / 6;
        bodies[j].vz += (k1vz + 2*k2vz + 2*k3vz + k4vz) / 6;

    }
}


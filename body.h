//
// Created by Devraj Katkoria on 30/07/2024.
//

#ifndef BODY_H
#define BODY_H

// Class to store the properties of a body - inclusive of initial and current positions, velocities, accelerations, and mass
class Body {
public:
    // Initial positions
    double x0, y0, z0;

    // Initial velocities
    double vx0, vy0, vz0;

    // Mass
    double mass;

    // Current positions
    double x, y, z;

    // Current velocities
    double vx, vy, vz;

    // Accelerations
    double ax, ay, az;
};

#endif //BODY_H

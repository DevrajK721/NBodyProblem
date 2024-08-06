#include <iostream>

using namespace std;

#include "body.h"

void accelerations(body *bodies, int n, double G);

void rungeKutta(body *bodies, int n, double t, double dt, double G);

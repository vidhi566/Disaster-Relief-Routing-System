#ifndef HELICOPTER_OPTIMIZER_H
#define HELICOPTER_OPTIMIZER_H

#include "structures.h"
#include <vector>
#include <chrono>

class HelicopterOptimizer {
public:
    HelicopterOptimizer(const ProblemData& problem, long long timeLimitMs);

    Solution optimize(const Solution& initialSolution);

    static double evaluateSolution(const ProblemData& problem, const Solution& solution);

private:
    const ProblemData& problem;
    Solution currentSolution;
    double currentValue;
    std::chrono::steady_clock::time_point startTime;
    long long timeLimit; // milliseconds

    bool shouldTerminate() const;

    // Neighbor generation
    Solution generateNeighbor(const Solution& base);
    Solution swapVillagesWithinTrip(const Solution& sol);
    Solution relocateVillage(const Solution& sol);
    Solution reallocateSupplies(const Solution& sol);
    Solution mergeTrips(const Solution& sol);
    Solution splitTrip(const Solution& sol);

    // Helpers
    void recomputeTrip(Trip& trip, const Helicopter& h);
    bool isFeasible(const Solution& sol);
};

#endif // HELICOPTER_OPTIMIZER_H

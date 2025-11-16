#include "HelicopterOptimizer.h"
#include <algorithm>
#include <chrono>
#include <random>
#include <iostream>

using namespace std;
using namespace std::chrono;

// random generator
static std::mt19937 rng(std::random_device{}());

inline int cityIndexFromId(const ProblemData &problem, int cityId) {
    // cities have ids 1..m and are stored at indices 0..m-1
    cout << "[DEBUG] cityIndexFromId called with cityId=" << cityId << endl;
    return problem.location_to_index[cityId]; // should be valid if precomputeDistM set it
}

inline int villageIndexFromId(const ProblemData &problem, int villageId) {
    // villages were mapped to numCities + (villageId-1)
    // retrieve numCities to be safe:
    int numCities = problem.cities.size();
    int idx = numCities + (villageId - 1);
    cout << "[DEBUG] villageIndexFromId called with villageId=" << villageId << ", index=" << idx << endl;
    // mapping approach used: problem.location_to_index[numCities + villageId]
    // but safer to compute index directly:
    return numCities + (villageId - 1);
}

HelicopterOptimizer::HelicopterOptimizer(const ProblemData& problem, long long timeLimitMs)
    : problem(problem), timeLimit(timeLimitMs) {
        cout << "[DEBUG] HelicopterOptimizer created with timeLimitMs=" << timeLimitMs << endl;
    }


Solution HelicopterOptimizer::optimize(const Solution& initialSolution) {
    cout << "[Optimizer] Starting optimization with initial greedy solution...\n";
    Solution bestSolution = initialSolution.copy();
    double bestValue = evaluateSolution(problem, bestSolution);

    cout << "[DEBUG] Initial solution value: " << bestValue << endl;
    currentSolution = initialSolution.copy();
    currentValue = bestValue;
    startTime = steady_clock::now();

    const int NUM_NEIGHBORS = 50;

    while (!shouldTerminate()) {
        Solution bestNeighbor;
        double bestNeighborValue = currentValue;

        for (int k = 0; k < NUM_NEIGHBORS; k++) {
            Solution neighbor = generateNeighbor(currentSolution);
            if (!isFeasible(neighbor)) {cout << "[DEBUG] Neighbor not feasible" << endl;continue;}
            
            double neighborValue = evaluateSolution(problem, neighbor);
            cout << "[DEBUG] Neighbor " << k << " value: " << neighborValue << endl;
            if (neighborValue > bestNeighborValue) {
                bestNeighborValue = neighborValue;
                bestNeighbor = neighbor;
                cout << "[DEBUG] New best neighbor found with value: " << bestNeighborValue << endl;
            }else{
                if ((rng() % 100) < 5) { // 5% chance
                currentSolution = neighbor;
                currentValue = neighborValue;
                cout << "[DEBUG] Random jump to neighbor with value: " << neighborValue << endl;
    }
            }
        }

        if (!bestNeighbor.plans.empty()) {
            currentSolution = bestNeighbor;
            currentValue = bestNeighborValue;
            cout << "[DEBUG] Updated current solution to best neighbor, value=" << bestNeighborValue << endl;
            if (currentValue > bestValue) {
                bestSolution = currentSolution.copy();
                bestValue = currentValue;
                cout << "[DEBUG] New overall best solution value: " << bestValue << endl;
            }
        } else {
            cout << "[DEBUG] No feasible neighbors found, breaking loop" << endl;
            break; 
        }
    }

    cout << "[Optimizer] Optimization finished. Best value: " << bestValue << endl;
    return bestSolution;
}

bool HelicopterOptimizer::shouldTerminate() const {
    auto now = steady_clock::now();
    long long elapsed = duration_cast<milliseconds>(now - startTime).count();
    if (elapsed > static_cast<long long>(timeLimit * 0.95)) {
        cout << "[DEBUG] Terminating optimization, elapsed time: " << elapsed << "ms" << endl;
        return true;
    }
    return false;
}

// double HelicopterOptimizer::evaluateSolution(const ProblemData& problem, const Solution& solution) {
//     double totalValue = 0.0, totalCost = 0.0;

//     vector<int> foodDelivered(problem.villages.size() + 1, 0);
//     vector<int> otherDelivered(problem.villages.size() + 1, 0);
//     vector<int> maxFood(problem.villages.size() + 1, 0);
//     vector<int> maxOther(problem.villages.size() + 1, 0);

//     for (const auto& v : problem.villages) {
//         maxFood[v.id] = 9 * v.population;
//         maxOther[v.id] = v.population;
//     }

//     for (const auto& plan : solution.plans) {
//         const Helicopter& h = problem.helicopters[plan.helicopter_id - 1];
//         for (const auto& trip : plan.trips) {
//             totalCost += h.fixed_cost + h.alpha * trip.trip_dist;
//             for (const auto& drop : trip.drops) {
//                 int vid = drop.village_id;
//                 int foodCapLeft = maxFood[vid] - foodDelivered[vid];
//                 int otherCapLeft = maxOther[vid] - otherDelivered[vid];

//                 int usePer = min(drop.perishable_food, foodCapLeft);
//                 int useDry = min(drop.dry_food, foodCapLeft - usePer);
//                 int useOther = min(drop.other_supplies, otherCapLeft);

//                 totalValue += usePer * problem.vPer +
//                               useDry * problem.vDry +
//                               useOther * problem.vOther;

//                 foodDelivered[vid] += usePer + useDry;
//                 otherDelivered[vid] += useOther;
//             }
//         }
//     }

//     return totalValue - totalCost;
// }

double HelicopterOptimizer::evaluateSolution(const ProblemData& problem, const Solution& solution) {
    double totalValue = 0.0, totalCost = 0.0;

    for (const auto& plan : solution.plans) {
        const Helicopter& h = problem.helicopters[plan.helicopter_id - 1];
        for (const auto& trip : plan.trips) {
            totalCost += h.fixed_cost + h.alpha * trip.trip_dist;
            for (const auto& drop : trip.drops) {
                totalValue += drop.perishable_food * problem.vPer +
                              drop.dry_food * problem.vDry +
                              drop.other_supplies * problem.vOther;
            }
        }
    }

    double netValue = totalValue - totalCost;
    cout << "[DEBUG] Evaluated solution, totalValue=" << totalValue << ", totalCost=" << totalCost
         << ", netValue=" << netValue << endl;
    return netValue;
}


// Neighbours 
Solution HelicopterOptimizer::generateNeighbor(const Solution& base) {
    uniform_int_distribution<int> dist(0, 4);
    int operatorId = dist(rng);

    cout << "[DEBUG] Generating neighbor using operator " << operatorId << endl;
    switch (operatorId) {
        case 0: return swapVillagesWithinTrip(base);
        case 1: return relocateVillage(base);
        case 2: return reallocateSupplies(base);
        case 3: return mergeTrips(base);
        case 4: return splitTrip(base);
        default: return base.copy();
    }
}

Solution HelicopterOptimizer::swapVillagesWithinTrip(const Solution& sol) {
    cout << "[DEBUG] swapVillagesWithinTrip called" << endl;
    Solution newSol = sol.copy();
    if (newSol.plans.empty()) return newSol;

    uniform_int_distribution<int> heliDist(0, (int)newSol.plans.size() - 1);
    HelicopterPlan& plan = newSol.plans[heliDist(rng)];
    if (plan.trips.empty()) return newSol;

    uniform_int_distribution<int> trip_dist(0, (int)plan.trips.size() - 1);
    Trip& trip = plan.trips[trip_dist(rng)];
    if (trip.drops.size() < 2) return newSol;

    int i = rng() % trip.drops.size();
    int j;
    do { j = rng() % trip.drops.size(); } while (j == i);
    swap(trip.drops[i], trip.drops[j]);

    recomputeTrip(trip, problem.helicopters[plan.helicopter_id - 1]);
    return newSol;
}

Solution HelicopterOptimizer::relocateVillage(const Solution& sol) {
    Solution newSol = sol.copy();
    if (newSol.plans.empty()) return newSol;

    int srcHeliIdx = rng() % newSol.plans.size();
    HelicopterPlan& srcHeli = newSol.plans[srcHeliIdx];
    if (srcHeli.trips.empty()) return newSol;

    int srcTripIdx = rng() % srcHeli.trips.size();
    Trip& srcTrip = srcHeli.trips[srcTripIdx];
    if (srcTrip.drops.empty()) return newSol;

    int dropIdx = rng() % srcTrip.drops.size();
    Drop dropToMove = srcTrip.drops[dropIdx];
    srcTrip.drops.erase(srcTrip.drops.begin() + dropIdx);
    recomputeTrip(srcTrip, problem.helicopters[srcHeli.helicopter_id - 1]);

    int destHeliIdx = rng() % newSol.plans.size();
    HelicopterPlan& destHeli = newSol.plans[destHeliIdx];

    if (destHeli.trips.empty()) {
        Trip newTrip;
        newTrip.drops.push_back(dropToMove);
        recomputeTrip(newTrip, problem.helicopters[destHeli.helicopter_id - 1]);
        destHeli.trips.push_back(newTrip);
    } else {
        Trip& destTrip = destHeli.trips[rng() % destHeli.trips.size()];
        destTrip.drops.push_back(dropToMove);
        recomputeTrip(destTrip, problem.helicopters[destHeli.helicopter_id - 1]);
    }

    return newSol;
}

Solution HelicopterOptimizer::reallocateSupplies(const Solution& sol) {
    Solution newSol = sol.copy();

    for (auto& heli : newSol.plans) {
        for (auto& trip : heli.trips) {
            if (!trip.drops.empty()) {
                int dropIdx = rng() % trip.drops.size();
                Drop& drop = trip.drops[dropIdx];

                int totalFood = drop.dry_food + drop.perishable_food;
                if (totalFood > 0) {
                    int newDry = rng() % (totalFood + 1);
                    int newPer = totalFood - newDry;
                    drop.dry_food = newDry;
                    drop.perishable_food = newPer;
                    recomputeTrip(trip, problem.helicopters[heli.helicopter_id - 1]);
                }
                return newSol;
            }
        }
    }
    return newSol;
}

Solution HelicopterOptimizer::mergeTrips(const Solution& sol) {
    Solution newSol = sol.copy();

    for (auto& heli : newSol.plans) {
        if (heli.trips.size() >= 2) {
            int t1 = rng() % heli.trips.size();
            int t2;
            do { t2 = rng() % heli.trips.size(); } while (t2 == t1);

            Trip& trip1 = heli.trips[t1];
            Trip& trip2 = heli.trips[t2];

            trip1.drops.insert(trip1.drops.end(), trip2.drops.begin(), trip2.drops.end());
            recomputeTrip(trip1, problem.helicopters[heli.helicopter_id - 1]);
            heli.trips.erase(heli.trips.begin() + t2);
            return newSol;
        }
    }
    return newSol;
}

Solution HelicopterOptimizer::splitTrip(const Solution& sol) {
    Solution newSol = sol.copy();

    for (auto& heli : newSol.plans) {
        for (auto& trip : heli.trips) {
            if (trip.drops.size() > 1) {
                int splitPoint = 1 + (rng() % (trip.drops.size() - 1));
                Trip newTrip;
                newTrip.drops.insert(newTrip.drops.end(),
                                     trip.drops.begin() + splitPoint,
                                     trip.drops.end());
                trip.drops.erase(trip.drops.begin() + splitPoint, trip.drops.end());

                recomputeTrip(trip, problem.helicopters[heli.helicopter_id - 1]);
                recomputeTrip(newTrip, problem.helicopters[heli.helicopter_id - 1]);
                heli.trips.push_back(newTrip);
                return newSol;
            }
        }
    }
    return newSol;
}

// void HelicopterOptimizer::recomputeTrip(Trip& trip, const Helicopter& h) {
//     double dist = 0.0;
//     int dry = 0, per = 0, other = 0;

//     int currentIdx = problem.location_to_index[h.home_city_id];
//     for (const auto& d : trip.drops) {
//         int vIdx = problem.location_to_index[d.village_id];
//         dist += problem.dist_matrix[currentIdx][vIdx];
//         currentIdx = vIdx;

//         dry += d.dry_food;
//         per += d.perishable_food;
//         other += d.other_supplies;
//     }
//     dist += problem.dist_matrix[currentIdx][problem.location_to_index[h.home_city_id]];

//     trip.trip_dist = dist;
//     trip.dry_food_pickup = dry;
//     trip.perishable_food_pickup = per;
//     trip.other_supplies_pickup = other;
// }
void HelicopterOptimizer::recomputeTrip(Trip& trip, const Helicopter& h) {
    double dist = 0.0;
    int dry = 0, per = 0, other = 0;

    // home city index in dist matrix
    int homeIdx = problem.location_to_index[h.home_city_id];

    int currentIdx = homeIdx;
    for (const auto& d : trip.drops) {
        // get the correct index for the village in the distance matrix
        int vIdx = villageIndexFromId(problem, d.village_id); // <-- FIXED
        // Bound-check (optional but helpful during debugging)
        if (vIdx < 0 || vIdx >= (int)problem.dist_matrix.size()) {
            // optional: print debug and return a large dist to mark infeasible
            // cout << "Invalid village index in recomputeTrip: " << vIdx << endl;
            trip.trip_dist = 1e12;
            return;
        }
        dist += problem.dist_matrix[currentIdx][vIdx];
        currentIdx = vIdx;

        dry += d.dry_food;
        per += d.perishable_food;
        other += d.other_supplies;
    }

    // return leg
    dist += problem.dist_matrix[currentIdx][homeIdx];

    trip.trip_dist = dist;
    trip.dry_food_pickup = dry;
    trip.perishable_food_pickup = per;
    trip.other_supplies_pickup = other;
}

bool HelicopterOptimizer::isFeasible(const Solution& sol) {
    vector<int> foodDelivered(problem.villages.size() + 1, 0);
    vector<int> otherDelivered(problem.villages.size() + 1, 0);

    for (const auto& plan : sol.plans) {
        const Helicopter& h = problem.helicopters[plan.helicopter_id - 1];
        double totalDist = 0.0;

        for (const auto& trip : plan.trips) {
            double tripWeight =
                trip.dry_food_pickup * problem.wDry +
                trip.perishable_food_pickup * problem.wPer +
                trip.other_supplies_pickup * problem.wOther;

            if (tripWeight > h.weight_capacity) return false;
            if (trip.trip_dist > h.distance_capacity) return false;

            totalDist += trip.trip_dist;
            if (totalDist > problem.d_max) return false;

            for (const auto& d : trip.drops) {
                const Village& v = problem.villages[d.village_id - 1];
                int maxFood = 9 * v.population;
                int maxOther = v.population;
                foodDelivered[d.village_id] += d.dry_food + d.perishable_food;
                otherDelivered[d.village_id] += d.other_supplies;
                if (foodDelivered[d.village_id] > maxFood) return false;
                if (otherDelivered[d.village_id] > maxOther) return false;
            }
        }
    }
    return true;
}



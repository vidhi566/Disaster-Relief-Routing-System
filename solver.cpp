#include "solver.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include "HelicopterOptimizer.h"

using namespace std;

static vector<Point> allLocations;

// TripResult struct for returning a trip + distance
struct TripResult {
    Trip trip;
    double distance;
    TripResult(const Trip& t = Trip(), double d = 0.0) : trip(t), distance(d) {}
};


static void precomputeDistM(ProblemData& problem);
static TripResult buildTripWithDistMatrix(
    const Helicopter& h,
    const ProblemData& problem,
    int homeIndex,
    vector<int>& remainingFood,
    vector<int>& remainingOther,
    const vector<Village>& villagesArray
,const std::chrono::steady_clock::time_point& stop_time
);

// Solution solve(ProblemData& problem) {
//     long long timeLimitMs = static_cast<long long>(problem.time_limit_minutes * 60.0 * 1000.0);
//     int i=0;
//     //   greedy solution
//     Solution initialSolution = buildGreedy(problem);
//     cout << "Greedy solution value: "
//          << HelicopterOptimizer::evaluateSolution(problem, initialSolution)
//          << endl;
//     i++;
//     // Hill climbing 
//     HelicopterOptimizer optimizer(problem, timeLimitMs);
//     Solution localSearchAns = optimizer.optimize(initialSolution);
//     i++;
//     cout << "Final optimized value: "
//          << HelicopterOptimizer::evaluateSolution(problem, localSearchAns)
//          << endl;
//     i++;
//     // if (i>10) break;
//     return localSearchAns;
// }

Solution solve(ProblemData& problem) {
    using Clock = std::chrono::steady_clock;

    long long totalBudgetMs = static_cast<long long>(problem.time_limit_minutes * 60.0 * 1000.0);
    long long allowedBudget = static_cast<long long>(totalBudgetMs * 0.85);

    // --- Run Greedy ---
    auto greedyStart = Clock::now();
    Solution initialSolution = buildGreedy(problem);
    auto greedyEnd = Clock::now();

    long long greedyTime = chrono::duration_cast<chrono::milliseconds>(greedyEnd - greedyStart).count();

    cout << "Greedy solution value: "
         << HelicopterOptimizer::evaluateSolution(problem, initialSolution)
         << " (finished in " << greedyTime/1000.0 << " sec)\n";

    // --- Optimizer with remaining budget ---
    long long optimizerBudget = max(0LL, allowedBudget - greedyTime);
    HelicopterOptimizer optimizer(problem, optimizerBudget);

    Solution localSearchAns = optimizer.optimize(initialSolution);

    cout << "Final optimized value: "
         << HelicopterOptimizer::evaluateSolution(problem, localSearchAns)
         << endl;

    return localSearchAns;
}


Solution buildGreedy(ProblemData &problem)
{
    Solution sol;
    using Clock = std::chrono::steady_clock;
    auto start_time = Clock::now();
    auto total_ms = std::chrono::milliseconds(
        static_cast<long long>(problem.time_limit_minutes * 60.0 * 1000.0));
    auto stop_time = start_time + std::chrono::duration_cast<std::chrono::milliseconds>(total_ms * 0.9);

    double totalValue = 0.0, totalCost = 0.0;

    // Precompute full distance matrix
    precomputeDistM(problem);


    int numVillages = problem.villages.size();
    vector<int> remainingFood(numVillages + 1, 0);
    vector<int> remainingOther(numVillages + 1, 0);
    vector<int> foodDelivered(numVillages + 1, 0);
    vector<int> otherDelivered(numVillages + 1, 0);
    vector<int> maxFoodPerVillage(numVillages + 1, 0);
    vector<int> maxOtherPerVillage(numVillages + 1, 0);

    for (const auto &v : problem.villages) {
        maxFoodPerVillage[v.id] = 9 * v.population;
        maxOtherPerVillage[v.id] = v.population;
        remainingFood[v.id] = maxFoodPerVillage[v.id];
        remainingOther[v.id] = maxOtherPerVillage[v.id];
    }

    vector<int> homeIndices(problem.helicopters.size());
    for (const auto &h : problem.helicopters) {
        homeIndices[h.id - 1] = problem.location_to_index[h.home_city_id];
    }

    for (const auto &h : problem.helicopters) {
        if (Clock::now() >= stop_time) break;

        HelicopterPlan plan(h.id);
        int homeIndex = homeIndices[h.id - 1];
        double usedDist = 0.0;

        while (usedDist < problem.d_max && Clock::now() < stop_time) {
            TripResult result = buildTripWithDistMatrix(
                h, problem, homeIndex, remainingFood, remainingOther,
                problem.villages, stop_time);

            if (!result.trip.drops.empty()) {
                plan.trips.push_back(result.trip);
                usedDist += result.distance;
                
                for (auto &drop : result.trip.drops)
                    cout << drop.village_id << " ";
                cout << "\nTrip distance (round trip): " << result.distance << endl;
                cout << "Weight delivered: Perishable=" << result.trip.perishable_food_pickup
                     << " Dry=" << result.trip.dry_food_pickup
                     << " Other=" << result.trip.other_supplies_pickup << endl;
            } else {
                break;
            }

            if (usedDist > problem.d_max) {
                usedDist -= result.distance;
                plan.trips.pop_back();
                break;
            }
        }

        sol.plans.push_back(plan);

        // Evaluate value/cost
        for (const auto &trip : plan.trips) {
            totalCost += h.fixed_cost + h.alpha * trip.trip_dist;

            for (const auto &drop : trip.drops) {
                int villageId = drop.village_id;
                int foodCapLeft = maxFoodPerVillage[villageId] - foodDelivered[villageId];
                int otherCapLeft = maxOtherPerVillage[villageId] - otherDelivered[villageId];

                int usePer = min(drop.perishable_food, foodCapLeft);
                int useDry = min(drop.dry_food, foodCapLeft - usePer);
                int useOther = min(drop.other_supplies, otherCapLeft);

                totalValue += usePer * problem.vPer +
                              useDry * problem.vDry +
                              useOther * problem.vOther;

                foodDelivered[villageId] += usePer + useDry;
                otherDelivered[villageId] += useOther;
            }
        }
    }

    sol.objective_value = totalValue - totalCost;
    return sol;
}



static void precomputeDistM(ProblemData& problem) {
    allLocations.clear();

    int numCities = problem.cities.size();
    int numVillages = problem.villages.size();

    // Add cities first
    for (const auto& city : problem.cities) {
        allLocations.push_back(city);
    }

    // Add villages next
    for (const auto& village : problem.villages) {
        allLocations.push_back(village.coords);
    }
// Needs to hold both cities and villages in shifted ID space
problem.location_to_index.assign(numCities + numVillages + 1, -1);

// Cities: IDs 1..m
for (int i = 0; i < numCities; i++) {
    problem.location_to_index[i + 1] = i; // city id → index
}

// Villages: IDs 1..n, shifted by numCities
for (int i = 0; i < numVillages; i++) {
    problem.location_to_index[numCities + (i + 1)] = numCities + i;
}



    // Build distance matrix
    int n = allLocations.size();
    problem.dist_matrix.assign(n, vector<double>(n, 0.0));

    // Only compute needed distances:
    // City -> Village, Village -> City, Village -> Village
    for (int i = 0; i < n; i++) {
        for (int j = i; j < n; j++) {
            bool skip = (i < numCities && j < numCities); // both are cities → skip
            if (skip) continue;

            double dist = distance(allLocations[i], allLocations[j]);
            problem.dist_matrix[i][j] = problem.dist_matrix[j][i] = dist;
        }
    }
}



static TripResult buildTripWithDistMatrix(
    const Helicopter& h,
    const ProblemData& problem,
    int homeIndex,                    
    vector<int>& remainingFood,
    vector<int>& remainingOther,
    const vector<Village>& villagesArray,
    const std::chrono::steady_clock::time_point& stop_time
) {
    Trip trip;
    trip.dry_food_pickup = trip.perishable_food_pickup = trip.other_supplies_pickup = 0;

    double remainingWeight = h.weight_capacity;
    double distanceSoFar = 0.0;
    int currentIndex = homeIndex;
    vector<bool> visited(villagesArray.size() + 1, false);

    int safetyCounter = 0;

    while (remainingWeight > 0 && std::chrono::steady_clock::now() < stop_time) {
        if (++safetyCounter > (int)villagesArray.size() * 2) break;

        const Village* bestVillage = nullptr;
        double bestScore = 0.0;
        int bestVillageIndex = -1;
        double distToBest = 0.0;

        for (const auto& v : villagesArray) {
            if (remainingFood[v.id] <= 0 && remainingOther[v.id] <= 0) continue;
            if (visited[v.id]) continue;

            int villageIndex = problem.location_to_index[problem.cities.size()+v.id];
            double toVillage = problem.dist_matrix[currentIndex][villageIndex];
            double toHome = problem.dist_matrix[villageIndex][homeIndex];

            // Check if round-trip within helicopter distance capacity
            if (distanceSoFar + toVillage + toHome > h.distance_capacity) continue;

            double perDensity = problem.vPer / problem.wPer;
            double dryDensity = problem.vDry / problem.wDry;
            double otherDensity = problem.vOther / problem.wOther;
            double bestTypeDensity = max({perDensity, dryDensity, otherDensity});

            double estValue = bestTypeDensity * remainingWeight;
            double score = estValue / (toVillage + 1e-6);

            if (score > bestScore) {
                bestScore = score;
                bestVillage = &v;
                bestVillageIndex = villageIndex;
                distToBest = toVillage;
            }
        }

        if (!bestVillage) break;

        int deliveredPer = 0, deliveredDry = 0, deliveredOther = 0;
        int foodCapLeft = remainingFood[bestVillage->id];

        vector<pair<double, string>> order = {
            {problem.vPer / problem.wPer, "per"},
            {problem.vDry / problem.wDry, "dry"},
            {problem.vOther / problem.wOther, "other"}};
        sort(order.begin(), order.end(),
             [](auto& a, auto& b) { return a.first > b.first; });

        for (auto& [density, type] : order) {
            if (remainingWeight <= 0) break;

            if (type == "per" && foodCapLeft > 0) {
                int maxPer = (int)(remainingWeight / problem.wPer);
                deliveredPer = min(foodCapLeft, maxPer);
                remainingWeight -= deliveredPer * problem.wPer;
                foodCapLeft -= deliveredPer;
            } else if (type == "dry" && foodCapLeft > 0) {
                int maxDry = (int)(remainingWeight / problem.wDry);
                deliveredDry = min(foodCapLeft, maxDry);
                remainingWeight -= deliveredDry * problem.wDry;
                foodCapLeft -= deliveredDry;
            } else if (type == "other" && remainingOther[bestVillage->id] > 0) {
                int maxOther = (int)(remainingWeight / problem.wOther);
                deliveredOther = min(remainingOther[bestVillage->id], maxOther);
                remainingWeight -= deliveredOther * problem.wOther;
                remainingOther[bestVillage->id] -= deliveredOther;
            }
        }

        if (deliveredPer + deliveredDry + deliveredOther == 0) break;

        remainingFood[bestVillage->id] = foodCapLeft;

        trip.drops.push_back(Drop(bestVillage->id, deliveredDry, deliveredPer, deliveredOther));


        distanceSoFar += distToBest;
        currentIndex = bestVillageIndex;
        visited[bestVillage->id] = true;
    }

    // Add final distance back to home city for round-trip
    distanceSoFar += problem.dist_matrix[currentIndex][homeIndex];
    trip.trip_dist = distanceSoFar;


    for (const auto& drop : trip.drops) {
        trip.dry_food_pickup += drop.dry_food;
        trip.perishable_food_pickup += drop.perishable_food;
        trip.other_supplies_pickup += drop.other_supplies;
    }

    return TripResult(trip, distanceSoFar);
}

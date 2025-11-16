#ifndef STRUCTURES_H
#define STRUCTURES_H

#include <vector>
#include <cmath>
using namespace std;

// --- GEOMETRIC & ENTITY STRUCTURES ---

struct Point {
    double x, y;
};

// --- UTILITY FUNCTIONS ---

inline double distance(const Point& p1, const Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

// --- PROBLEM & SOLUTION STRUCTURES ---

struct PackageInfo {
    double weight, value;
};

struct Village {
    int id;
    Point coords;
    int population;
    Village() : id(0), coords{0, 0}, population(0) {}
    Village(int id, Point coords, int population)
        : id(id), coords(coords), population(population) {}
};

struct Helicopter {
    int id;
    int home_city_id;
    double weight_capacity;
    double distance_capacity;
    double fixed_cost; // F
    double alpha;
    Helicopter() : id(0), home_city_id(0), weight_capacity(0), distance_capacity(0), fixed_cost(0), alpha(0) {}
    Helicopter(int id, int home_city_id, double weight_capacity,
               double distance_capacity, double fixed_cost, double alpha)
        : id(id), home_city_id(home_city_id), weight_capacity(weight_capacity),
          distance_capacity(distance_capacity), fixed_cost(fixed_cost), alpha(alpha) {}
};

struct ProblemData {
    double time_limit_minutes;
    double d_max;

    // food package types
    double wDry, vDry;
    double wPer, vPer;
    double wOther, vOther;

    vector<PackageInfo> packages;
    vector<Point> cities;
    vector<Village> villages;
    vector<Helicopter> helicopters;

    vector<int> location_to_index;
    vector<vector<double> > dist_matrix;
};

struct Drop {
    int village_id;
    int dry_food;
    int perishable_food;
    int other_supplies;

    Drop(int vid, int d, int p, int o)
        : village_id(vid), dry_food(d), perishable_food(p), other_supplies(o) {}

    Drop copy() const {
        return Drop(village_id, dry_food, perishable_food, other_supplies);
    }
};

struct Trip {
    int dry_food_pickup ;
    int perishable_food_pickup ;
    int other_supplies_pickup ;
    vector<Drop> drops;
    double trip_dist;

    Trip copy() const {
        Trip t;
        t.trip_dist = trip_dist;
        t.dry_food_pickup = dry_food_pickup;
        t.perishable_food_pickup = perishable_food_pickup;
        t.other_supplies_pickup = other_supplies_pickup;
        for (const auto& d : drops) t.drops.push_back(d.copy());
        return t;
    }
};

struct HelicopterPlan {
    int helicopter_id;
    vector<Trip> trips;
    
    HelicopterPlan(int id) : helicopter_id(id) {}

    HelicopterPlan copy() const {
        HelicopterPlan p(helicopter_id);
        for (const auto& t : trips) p.trips.push_back(t.copy());
        return p;
    }
};

struct Solution {
    vector<HelicopterPlan> plans;
    double objective_value;

    Solution copy() const {
        Solution s;
        s.objective_value = objective_value;
        for (const auto& p : plans) s.plans.push_back(p.copy());
        return s;
    }
};

#endif // STRUCTURES_H

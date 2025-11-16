# Disaster-Relief-Routing-System

## 🚨 Overview  
Disaster-Relief-Routing-System is a C++/Makefile-based project designed to optimise routing of relief resources (e.g. helicopters) in disaster-hit zones. The system ingests data about resources, locations, constraints and computes optimal (or near-optimal) routes for swift delivery and distribution. The core algorithm is implemented in `HelicopterOptimizer.cpp` and related modules.

## 🧩 Features  
- Modular architecture: `io_handler`, `structures`, `solver`, `HelicopterOptimizer`  
- Flexible input formats via `io_handler` (reads mission data, resource specs, zone maps)  
- Route optimisation engine (implements heuristics or exact methods)  
- Makefile build system for ease of compilation on Unix/macOS/Linux  
- Extensible — you can plug in different resource types, constraints, or objective functions  

## 📁 Repository structure  

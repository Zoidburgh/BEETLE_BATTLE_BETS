import sys
print("Starting test compilation...", flush=True)
sys.stdout.flush()

print("Importing Taichi...", flush=True)
import taichi as ti
print("Taichi imported successfully", flush=True)

print("Importing simulation module...", flush=True)
import simulation
print("Simulation imported successfully!", flush=True)
print("Grid size:", simulation.n_grid, flush=True)

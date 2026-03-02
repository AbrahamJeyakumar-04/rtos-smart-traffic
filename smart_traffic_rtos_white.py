# rtos_simulation.py
import matplotlib.pyplot as plt
import random
import time

def run_simulation_and_plot():
    tasks = {
        "T1_Sensor": {"period":50, "wcet":5},
        "T2_Emergency": {"period":20, "wcet":3},
        "T3_Decision": {"period":100, "wcet":15},
        "T4_Signal": {"period":200, "wcet":8},
        "T5_Logger": {"period":500, "wcet":10},
    }

    gantt_data = []
    density_history = []
    next_release = {t: 0 for t in tasks}

    colors = {
        "T1_Sensor":"#e67e22",
        "T2_Emergency":"#e74c3c",
        "T3_Decision":"#9b59b6",
        "T4_Signal":"#3498db",
        "T5_Logger":"#2ecc71"
    }

    for sim_time in range(600):
        ready = []

        for t in tasks:
            if sim_time >= next_release[t]:
                ready.append(t)
                next_release[t] += tasks[t]["period"]

        if ready:
            running = min(ready, key=lambda t: tasks[t]["period"])
            wcet = tasks[running]["wcet"]
            gantt_data.append((sim_time, wcet, running))

        density_history.append(random.randint(5, 25))
        time.sleep(0.005)

    # Final report plot
    fig, ax = plt.subplots(figsize=(10, 4))
    for start, dur, name in gantt_data:
        ax.broken_barh([(start, dur)], (0, 1), facecolors=colors[name])

    ax.set_title("RTOS Scheduling Timeline")
    ax.set_xlabel("Time (ms)")

    plt.savefig("smart_traffic_rtos_report.png")
    plt.close(fig)

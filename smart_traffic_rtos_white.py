import streamlit as st
import matplotlib.pyplot as plt
import pandas as pd
import random
import time

st.set_page_config(layout="wide")
st.title("🚦 RTOS-Based Smart Traffic Intersection System")

# ----------------------------------------------------------
# RTOS TASK MODEL TABLE
# ----------------------------------------------------------
task_data = {
    "Task": [
        "T1 – Sensor Task",
        "T2 – Emergency Task",
        "T3 – Decision Task",
        "T4 – Signal Control",
        "T5 – Logging Task"
    ],
    "Function": [
        "Read vehicle density",
        "Detect ambulance",
        "Compute signal timing",
        "Update traffic lights",
        "Store system data"
    ],
    "Period (ms)": [50, 20, 100, 200, 500],
    "WCET (ms)": [5, 3, 15, 8, 10],
    "Deadline (ms)": [50, 20, 100, 200, 500],
    "Criticality": ["High", "Critical", "High", "Medium", "Low"]
}

task_df = pd.DataFrame(task_data)

st.sidebar.subheader("📋 RTOS Task Model")
st.sidebar.dataframe(
    task_df.style.apply(
        lambda row: ["background-color:#ffcccc"
                     if row["Criticality"]=="Critical" else "" for _ in row],
        axis=1
    ),
    use_container_width=True
)

# ----------------------------------------------------------
# TASK DEFINITIONS (for scheduler)
# ----------------------------------------------------------
tasks = {
    "T1_Sensor": {"period":50, "wcet":5},
    "T2_Emergency": {"period":20, "wcet":3},
    "T3_Decision": {"period":100, "wcet":15},
    "T4_Signal": {"period":200, "wcet":8},
    "T5_Logger": {"period":500, "wcet":10},
}

task_states = {t:"READY" for t in tasks}
execution_count = {t:0 for t in tasks}

# ----------------------------------------------------------
# SIDEBAR CONTROLS
# ----------------------------------------------------------
algo = st.sidebar.selectbox(
    "Scheduling Algorithm",
    ["EDF", "RMS"]
)

speed = st.sidebar.slider("Simulation Speed", 0.005, 0.1, 0.01)

# ----------------------------------------------------------
# LAYOUT
# ----------------------------------------------------------
left, right = st.columns([2,1])

density_plot = left.empty()
gantt_plot = left.empty()
rtos_panel = right.empty()

# ----------------------------------------------------------
# SCHEDULER FUNCTIONS
# ----------------------------------------------------------
def choose_edf(sim_time, ready):
    return min(ready, key=lambda t: sim_time + tasks[t]["period"])

def choose_rms(ready):
    return min(ready, key=lambda t: tasks[t]["period"])

# ----------------------------------------------------------
# START SIMULATION
# ----------------------------------------------------------
if st.button("▶ Start RTOS Simulation"):

    deadline_miss = 0
    gantt_data = []
    density_history = []

    next_release = {t:0 for t in tasks}

    colors = {
        "T1_Sensor":"#e67e22",
        "T2_Emergency":"#e74c3c",
        "T3_Decision":"#9b59b6",
        "T4_Signal":"#3498db",
        "T5_Logger":"#2ecc71"
    }

    for sim_time in range(600):

        update_ui = (sim_time % 5 == 0)

        # ---------------- TASK RELEASE ----------------
        ready_queue = []

        for t in tasks:
            if sim_time >= next_release[t]:
                ready_queue.append(t)
                task_states[t] = "READY"
                next_release[t] += tasks[t]["period"]

        # ---------------- SCHEDULING ----------------
        if ready_queue:

            running = choose_edf(sim_time, ready_queue) if algo=="EDF" \
                      else choose_rms(ready_queue)

            for t in task_states:
                if t == running:
                    task_states[t] = "RUNNING"
                elif t in ready_queue:
                    task_states[t] = "READY"
                else:
                    task_states[t] = "BLOCKED"

            execution_count[running] += 1

            wcet = tasks[running]["wcet"]
            finish_time = sim_time + wcet
            deadline = sim_time + tasks[running]["period"]

            if finish_time > deadline:
                deadline_miss += 1

            gantt_data.append((sim_time, wcet, running))


        # ---------------- VEHICLE DENSITY ----------------
        density = random.randint(5,25)
        density_history.append(density)

        fig1, ax1 = plt.subplots()
        ax1.plot(density_history[-80:])
        ax1.set_title("Vehicle Density (Real-Time)")
        ax1.set_ylim(0,30)
        density_plot.pyplot(fig1)
        plt.close(fig1)
        if update_ui:
            density_plot.pyplot(fig1)
    

        # ---------------- PROFESSIONAL GANTT ----------------
        fig2, ax2 = plt.subplots(figsize=(10,4))

        task_order = list(tasks.keys())
        y_pos = {task:i for i,task in enumerate(task_order)}

        for start, dur, name in gantt_data:
            ax2.broken_barh(
                [(start, dur)],
                (y_pos[name]-0.3, 0.6),
                facecolors=colors[name]
            )

        ax2.set_yticks(range(len(task_order)))
        ax2.set_yticklabels(task_order)
        ax2.set_xlabel("Simulation Time (ms)")
        ax2.set_title(f"{algo} Scheduler — RTOS Timeline")
        ax2.grid(True, axis='x', linestyle='--', alpha=0.4)
        ax2.set_xlim(max(0, sim_time-200), sim_time+20)

        gantt_plot.pyplot(fig2)
        plt.close(fig2)
        if update_ui:
           gantt_plot.pyplot(fig2)

        # ---------------- RTOS MONITOR PANEL ----------------
        cpu_util = sum(v["wcet"]/v["period"] for v in tasks.values())*100
        emergency = (200 < sim_time < 300)

        with rtos_panel.container():

            st.subheader("🖥 RTOS Monitor")

            st.metric("Scheduler", algo)
            st.metric("System Time (ms)", sim_time)
            st.metric("CPU Utilization (%)", f"{cpu_util:.1f}")
            st.metric("Deadline Misses", deadline_miss)

            st.write("### Emergency Status")
            st.warning("🚑 ACTIVE") if emergency else st.success("Normal")

            st.write("### Task States")
            for t,s in task_states.items():
                if s=="RUNNING":
                    st.error(f"{t} : RUNNING")
                elif s=="READY":
                    st.info(f"{t} : READY")
                else:
                    st.write(f"{t} : BLOCKED")

            st.write("### Execution Count")
            for t,c in execution_count.items():
                st.write(f"{t} → {c}")

        time.sleep(speed)

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

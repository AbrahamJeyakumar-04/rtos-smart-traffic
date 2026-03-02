import streamlit as st
from smart_traffic_rtos_white import run_simulation_and_plot
import os

st.set_page_config(page_title="RTOS Smart Traffic", layout="wide")

st.title("🚦 RTOS-Based Smart Traffic Intersection")

if st.button("▶ Run Simulation"):
    with st.spinner("Running simulation..."):
        run_simulation_and_plot()
    st.success("Simulation finished")

if os.path.exists("smart_traffic_rtos_report.png"):
    st.image("smart_traffic_rtos_report.png", use_container_width=True)
else:
    st.info("Click 'Run Simulation' to generate report")

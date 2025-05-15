import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import mopeds
import copy

def initialize_problem():
    variable_list = mopeds.VariableList()

    # Define variables
    variable_list.add_variable(mopeds.VariableAlgebraic("e0_F_d_in", 0.0, -1e9, 1e9))
    variable_list.add_variable(mopeds.VariableAlgebraic("e0_p", 0.0, -1e9, 1e9))
    variable_list.add_variable(mopeds.VariableState("e0_n", 1.0))

    variable_list.add_variable(mopeds.VariableControl("e0_F_in", 0.1156, -1e9, 1e9))
    variable_list.add_variable(mopeds.VariableControl("e0_p_max", 3e6, -1e9, 1e9))
    variable_list.add_variable(mopeds.VariableControl("e0_F_out", 0.0, -1e9, 1e9))
    variable_list.add_variable(mopeds.VariableControl("e0_R", 8.314, -1e9, 1e9))
    variable_list.add_variable(mopeds.VariableControl("e0_T", 293.15, -1e9, 1e9))
    variable_list.add_variable(mopeds.VariableControl("e0_V", 5.0, -1e9, 1e9))

    m = mopeds.Model(variable_list)

    # CasADi variables
    e0_F_in = m.varlist_all["e0_F_in"].casadi_var
    e0_p_max = m.varlist_all["e0_p_max"].casadi_var
    e0_F_out = m.varlist_all["e0_F_out"].casadi_var
    e0_R = m.varlist_all["e0_R"].casadi_var
    e0_T = m.varlist_all["e0_T"].casadi_var
    e0_V = m.varlist_all["e0_V"].casadi_var
    e0_F_d_in = m.varlist_all["e0_F_d_in"].casadi_var
    e0_p = m.varlist_all["e0_p"].casadi_var
    e0_n = m.varlist_all["e0_n"].casadi_var

    # Equations
    EQ_diff1 = e0_F_d_in - e0_F_out
    EQ_alg2 = e0_F_d_in - (e0_F_in * (1.0 - (e0_p / e0_p_max)))
    EQ_alg3 = e0_p - ((e0_n * e0_R * e0_T) / e0_V)

    m.add_equations_differential([EQ_diff1])
    m.add_equations_algebraic([EQ_alg2, EQ_alg3])

    # Set consistent initial values
    R = float(m.varlist_all["e0_R"].value[0])
    T = float(m.varlist_all["e0_T"].value[0])
    V = float(m.varlist_all["e0_V"].value[0])
    F_in = float(m.varlist_all["e0_F_in"].value[0])
    p_max = float(m.varlist_all["e0_p_max"].value[0])

    n0 = 1.0
    p0 = (n0 * R * T) / V
    F_d_in = F_in * (1 - p0 / p_max)

    m.varlist_all["e0_n"].dataframe["e0_n"].iloc[0] = n0
    m.varlist_all["e0_p"].dataframe["e0_p"].iloc[0] = p0
    m.varlist_all["e0_F_d_in"].dataframe["e0_F_d_in"].iloc[0] = F_d_in
    m.varlist_all["e0_F_out"].dataframe["e0_F_out"].iloc[0] = F_d_in

    return variable_list, m


if __name__ == "__main__":
    # Create time grid
    time_grid = np.linspace(10, 1000000, 100)
    time_grid = np.insert(time_grid, 0, 0)

    # First simulation (constant F_out)
    var_list, m = initialize_problem()
    sim1 = mopeds.Simulator(m, time_grid, var_list)
    res1 = sim1.generate_exp_data(algebraic=True)
    print("First simulation result:")
    print(res1.dataframe)
    res1.plot(algebraic=True)
    res1.dataframe.to_csv("sim1_results.csv", index=False)

    # Save data to CSV or pickle
    res1.dataframe.to_csv("sim1_results.csv", index=False)

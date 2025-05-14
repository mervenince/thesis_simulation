import copy
import casadi as ca
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import mopeds


def initialize_problem():  # noqa: C901

    variable_list = mopeds.VariableList()
    # fmt:off


    variable_list.add_variable(mopeds.VariableAlgebraic("e0_F_d_in", 0.0, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableAlgebraic("e0_b", 1.661E-5, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableAlgebraic("e0_a", 0.00184, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableAlgebraic("e0_a_c", 0.02694, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableAlgebraic("e0_greek_kappa", 0.3748, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableAlgebraic("e0_p", 101325.0, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableAlgebraic("e0_V_m", 0.025, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableAlgebraic("e0_greek_alpha", 0.0, -1.0E9, 1.0E9))  # noqa: E501

    variable_list.add_variable(mopeds.VariableState("e0_n", 200.0))  # noqa: E501

    variable_list.add_variable(mopeds.VariableControl("e0_F_out", 0.0, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableControl("e0_R", 8.314, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableControl("e0_T", 293.15, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableControl("e0_T_c", 33.2, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableControl("e0_p_c", 1293000.0, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableControl("e0_greek_omega", 1.0E-4, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableControl("e0_F_in", 0.1156, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableControl("e0_p_max", 3000000.0, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableControl("e0_V", 5.0, -1.0E9, 1.0E9))  # noqa: E501


    m = mopeds.Model(variable_list)

    e0_F_out = m.varlist_all["e0_F_out"].casadi_var  # noqa: E501
    e0_R = m.varlist_all["e0_R"].casadi_var  # noqa: E501
    e0_T = m.varlist_all["e0_T"].casadi_var  # noqa: E501
    e0_T_c = m.varlist_all["e0_T_c"].casadi_var  # noqa: E501
    e0_p_c = m.varlist_all["e0_p_c"].casadi_var  # noqa: E501
    e0_greek_omega = m.varlist_all["e0_greek_omega"].casadi_var  # noqa: E501
    e0_F_in = m.varlist_all["e0_F_in"].casadi_var  # noqa: E501
    e0_p_max = m.varlist_all["e0_p_max"].casadi_var  # noqa: E501
    e0_V = m.varlist_all["e0_V"].casadi_var  # noqa: E501
    e0_F_d_in = m.varlist_all["e0_F_d_in"].casadi_var  # noqa: E501
    e0_b = m.varlist_all["e0_b"].casadi_var  # noqa: E501
    e0_a = m.varlist_all["e0_a"].casadi_var  # noqa: E501
    e0_a_c = m.varlist_all["e0_a_c"].casadi_var  # noqa: E501
    e0_greek_kappa = m.varlist_all["e0_greek_kappa"].casadi_var  # noqa: E501
    e0_n = m.varlist_all["e0_n"].casadi_var  # noqa: E501
    e0_p = m.varlist_all["e0_p"].casadi_var  # noqa: E501
    e0_V_m = m.varlist_all["e0_V_m"].casadi_var  # noqa: E501
    e0_greek_alpha = m.varlist_all["e0_greek_alpha"].casadi_var  # noqa: E501

    EQ_diff1 = (e0_F_d_in-e0_F_out)  # noqa: E501,E226

    EQ_alg2 = (e0_F_d_in-((e0_F_in*((1.0-(e0_p/e0_p_max))))))  # noqa: E501,E226
    EQ_alg3 = (e0_V_m-((e0_V/e0_n)))  # noqa: E501,E226
    EQ_alg4 = (e0_p-((((e0_R*e0_T)/(e0_V_m-e0_b))-(e0_greek_alpha/((e0_V_m*((e0_V_m+e0_b)))+(e0_b*((e0_V_m-e0_b))))))))  # noqa: E501,E226
    EQ_alg5 = (e0_a-((e0_a_c*e0_greek_alpha)))  # noqa: E501,E226
    EQ_alg6 = (e0_a_c-((0.45724*((((e0_R))**(1.0*2.0)*((e0_T_c))**(1.0*2.0))/e0_p_c))))  # noqa: E501,E226
    EQ_alg7 = (e0_greek_alpha-((((1.0+(e0_greek_kappa*((1.0-((e0_T/e0_T_c))**(1.0/(2.0))))))))**(1.0*2.0)))  # noqa: E501,E226
    EQ_alg8 = (e0_greek_kappa-(((0.37464+(1.54226*e0_greek_omega))-(0.26992*((e0_greek_omega))**(1.0*2.0)))))  # noqa: E501,E226
    EQ_alg9 = (e0_b-((0.0778*((e0_R*e0_T_c)/e0_p_c))))  # noqa: E501,E226

    order_state_var = ["e0_n", ]  # noqa: E501
    order_eqs_diff = {"e0_n": EQ_diff1, }  # noqa: E501

    order_eqs_corrected = list(order_eqs_diff[i] for i in order_state_var)
    m.add_equations_differential(order_eqs_corrected)
    list_algebraic_equations = [EQ_alg2, EQ_alg3, EQ_alg4, EQ_alg5, EQ_alg6, EQ_alg7, EQ_alg8, EQ_alg9, ]  # noqa: E501


    # fmt:on

    m.add_equations_algebraic(list_algebraic_equations)

    return variable_list, m


if __name__ == "__main__":
    var_list, m = initialize_problem()

    # Create time-grid. Zero should be first
    time_grid = np.linspace(10, 1000000, 100)
    time_grid = np.insert(time_grid, 0, 0)

    # Create simulation Object
    sim = mopeds.Simulator(m, time_grid, var_list)
    res = sim.generate_exp_data(algebraic=True)
    print(res.dataframe)
    res.plot(algebraic=True)

    # Load sim1 results
    df1 = pd.read_csv("sim1_results.csv")

    # Run or load sim2 results
    df2 = res.dataframe  # assuming you ran another simulation

    # Set time as index if needed
    df1.index = time_grid
    df2.index = time_grid

    # Overlay plot
    plt.plot(df1.index, df1["e0_p"], label="H2 - Ideal Gas")
    plt.plot(df2.index, df2["e0_p"], label="H2 - Real Gas", linestyle="--")
    plt.xlabel("Time [s]")
    plt.ylabel("Pressure [Pa]")
    plt.legend()
    plt.grid(True)
    plt.title("Comparison of ideal and real gas models for H2")
    plt.show()
import copy
import casadi as ca
import numpy as np

import mopeds


def initialize_problem():  # noqa: C901

    variable_list = mopeds.VariableList()
    # fmt:off


    variable_list.add_variable(mopeds.VariableAlgebraic("e0_F_d_in", 0.0, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableAlgebraic("e0_p", 0.0, -1.0E9, 1.0E9))  # noqa: E501

    variable_list.add_variable(mopeds.VariableState("e0_n", 0.0))  # noqa: E501

    variable_list.add_variable(mopeds.VariableControl("e0_F_in", 0.0578, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableControl("e0_p_max", 3000000.0, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableControl("e0_F_out", 0.0, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableControl("e0_R", 8.314, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableControl("e0_T", 293.15, -1.0E9, 1.0E9))  # noqa: E501
    variable_list.add_variable(mopeds.VariableControl("e0_V", 5.0, -1.0E9, 1.0E9))  # noqa: E501


    m = mopeds.Model(variable_list)

    e0_F_in = m.varlist_all["e0_F_in"].casadi_var  # noqa: E501
    e0_p_max = m.varlist_all["e0_p_max"].casadi_var  # noqa: E501
    e0_F_out = m.varlist_all["e0_F_out"].casadi_var  # noqa: E501
    e0_R = m.varlist_all["e0_R"].casadi_var  # noqa: E501
    e0_T = m.varlist_all["e0_T"].casadi_var  # noqa: E501
    e0_V = m.varlist_all["e0_V"].casadi_var  # noqa: E501
    e0_F_d_in = m.varlist_all["e0_F_d_in"].casadi_var  # noqa: E501
    e0_p = m.varlist_all["e0_p"].casadi_var  # noqa: E501
    e0_n = m.varlist_all["e0_n"].casadi_var  # noqa: E501

    EQ_diff1 = (e0_F_d_in-e0_F_out)  # noqa: E501,E226

    EQ_alg2 = (e0_F_d_in-((e0_F_in*((1.0-(e0_p/e0_p_max))))))  # noqa: E501,E226
    EQ_alg3 = (e0_p-(((e0_n*e0_V)/(e0_R*e0_T))))  # noqa: E501,E226

    order_state_var = ["e0_n", ]  # noqa: E501
    order_eqs_diff = {"e0_n": EQ_diff1, }  # noqa: E501

    order_eqs_corrected = list(order_eqs_diff[i] for i in order_state_var)
    m.add_equations_differential(order_eqs_corrected)
    list_algebraic_equations = [EQ_alg2, EQ_alg3, ]  # noqa: E501


    # fmt:on

    m.add_equations_algebraic(list_algebraic_equations)

    return variable_list, m


if __name__ == "__main__":
    var_list, m = initialize_problem()

    # Create time-grid. Zero should be first
    time_grid = np.linspace(10, 1000000000, 40)
    time_grid = np.insert(time_grid, 0, 0)

    # Create simulation Object
    sim = mopeds.Simulator(m, time_grid, var_list)
    res = sim.generate_exp_data(algebraic=True)
    print(res.dataframe)
    res.plot(algebraic=True)
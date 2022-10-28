within OpenIPSL.DataSets.Anderson.ES_Data;
record ExcSystemData_F12
  extends ExcSystemData_Template(VR_type = "A",
    Name = "WMA",
    RR = 0.50,
    T_R = 0.000,
    K_A = 400.000,
    T_A1 = 0.050,
    T_A2 = 0.000,
    V_RMAX = 3.810,
    V_RMIN = -3.810,
    K_E = -0.170,
    T_E = 0.950,
    E_1 = 3.6675,
    E_2 = 4.890,
    S_EE_1 = 0.220,
    S_EE_2 = 0.950,
    A_ex = 0.0027,
    B_ex = 0.3857,
    Efd_max = 4.890,
    Efd_min = -4.890,
    K_F = 0.040,
    T_F1 = 1.000,
    T_F2 = 0.000);
end ExcSystemData_F12;

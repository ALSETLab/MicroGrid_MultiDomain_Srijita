within OpenIPSL.DataSets.Anderson.Steam.Fossil;
record Anderson_F12 "Anderson Fossil Steam Unit 12 (330 MVA)"

  extends DataSets.GU_Dynamics_Template;

  replaceable record Machine = Machine_Data.MachineData_F12 constrainedby
    Machine_Data.MachineData_Template   "Machine data";
  Machine machine;

  replaceable record ExcSystem = ES_Data.ExcSystemData_F12 constrainedby
    ES_Data.ExcSystemData_Template   "Excitation system data";
  ExcSystem excSystem;

  replaceable record TurbGovernor = TurbGov_Data.TurbGovernorData_F12 constrainedby
    TurbGov_Data.TurbGovernorData_Template   "Turbine-Governor data";
  TurbGovernor turbGovernor;

  replaceable record PSS = PSS_Data.PSSData_ND constrainedby
    PSS_Data.PSSData_Template   "Power system stabilizer data";
      PSS pss;

end Anderson_F12;
